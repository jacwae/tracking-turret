"""
Pan/tilt tracking loop.

    camera -> detect -> centre error -> P regulator -> serial command -> Arduino -> OK

The loop is deliberately small: every stage is a separate object so you can
swap the detector, test the regulator without hardware, or replace the motor
controller without touching the rest.

Protocol (one line per command, LF terminated, ASCII):

    P-12 T3      <- "pan -12, tilt +3",  sign is always written explicitly
    OK           <- Arduino's acknowledgement

Run:

    python tracker.py --port /dev/ttyACM0 --target person
"""

from __future__ import annotations

import argparse
import logging
import time
from dataclasses import dataclass
from typing import Optional, Tuple

log = logging.getLogger("tracker")

Box = Tuple[float, float, float, float]  # x1, y1, x2, y2
Command = Tuple[int, int]  # pan, tilt


# --------------------------------------------------------------------------- #
# configuration
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Config:
    # serial link
    port: str = "/dev/ttyACM0"
    baud: int = 115200
    ack_timeout_s: float = 1.5

    # camera
    camera_index: int = 0
    width: int = 640
    height: int = 480
    target_fps: int = 30

    # detection
    model: str = "yolov8n.pt"
    target_class: Optional[str] = "person"
    conf: float = 0.4

    # P regulator
    pan_gain: float = 0.05  # command units per pixel of error
    tilt_gain: float = 0.05
    pan_sign: int = 1  # flip to -1 if the axes fight each other
    tilt_sign: int = -1  # tilt cameras usually need the opposite sign to pan
    deadband_px: int = 12  # inside this the motors are considered on target
    max_step: int = 25  # never ask for more than this per axis

    # behaviour
    lost_frames_before_stop: int = 5
    log_interval_s: float = 2.0


# --------------------------------------------------------------------------- #
# stage 1: camera
# --------------------------------------------------------------------------- #


class Camera:
    """Opens the capture device, preferring V4L2 on the Pi but not requiring it."""

    def __init__(self, cfg: Config):
        import cv2

        self._cv2 = cv2
        backend = getattr(cv2, "CAP_V4L2", cv2.CAP_ANY)
        self.cap = cv2.VideoCapture(cfg.camera_index, backend)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cfg.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera index {cfg.camera_index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
        self.cap.set(cv2.CAP_PROP_FPS, cfg.target_fps)

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("camera stopped delivering frames")
        return frame

    def close(self) -> None:
        self.cap.release()


# --------------------------------------------------------------------------- #
# stage 2: detection
# --------------------------------------------------------------------------- #


class YoloDetector:
    """Wraps Ultralytics YOLO and returns the single box we care about."""

    def __init__(self, cfg: Config):
        from ultralytics import YOLO

        self._model = YOLO(cfg.model)
        self._conf = cfg.conf
        self._target = cfg.target_class
        # Resolve the class name to an id once, so we do a cheap int compare later.
        self._target_id = None
        if self._target:
            names = self._model.names
            for cls_id, name in names.items():
                if str(name).lower() == self._target.lower():
                    self._target_id = int(cls_id)
                    break
            if self._target_id is None:
                raise ValueError(
                    f"class {self._target!r} not in model; available: {list(names.values())}"
                )

    def detect(self, frame) -> Optional[Box]:
        result = self._model(frame, conf=self._conf, verbose=False)[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return None

        xyxy = boxes.xyxy
        best, best_area = None, -1.0
        for i in range(len(xyxy)):
            if self._target_id is not None and int(boxes.cls[i]) != self._target_id:
                continue
            x1, y1, x2, y2 = (float(v) for v in xyxy[i])
            area = (x2 - x1) * (y2 - y1)
            if area > best_area:
                best, best_area = (x1, y1, x2, y2), area
        return best


# --------------------------------------------------------------------------- #
# stage 3 + 4: error and P regulator
# --------------------------------------------------------------------------- #


def centre_error(box: Box, frame_w: int, frame_h: int) -> Tuple[float, float]:
    """Pixels the box centre sits right of / below the image centre."""
    x1, y1, x2, y2 = box
    return ((x1 + x2) / 2.0 - frame_w / 2.0, (y1 + y2) / 2.0 - frame_h / 2.0)


class PController:
    """Error -> motor step. Returns None when there is nothing new to send."""

    def __init__(self, cfg: Config):
        self._cfg = cfg
        self._last: Optional[Command] = None
        self._parked = False

    def reset(self) -> None:
        self._last = None
        self._parked = False

    def _clamp(self, value: int, limit: int) -> int:
        return max(-limit, min(limit, value))

    def update(self, err_x: float, err_y: float) -> Optional[Command]:
        cfg = self._cfg
        if abs(err_x) <= cfg.deadband_px and abs(err_y) <= cfg.deadband_px:
            # Inside the deadband we send one stop command, then stay quiet.
            if not self._parked:
                self._parked = True
                self._last = (0, 0)
                return (0, 0)
            return None

        self._parked = False
        command = (
            self._clamp(int(round(cfg.pan_gain * cfg.pan_sign * err_x)), cfg.max_step),
            self._clamp(int(round(cfg.tilt_gain * cfg.tilt_sign * err_y)), cfg.max_step),
        )
        if command == self._last:
            return None
        self._last = command
        return command


# --------------------------------------------------------------------------- #
# stage 5 + 6: serial link
# --------------------------------------------------------------------------- #


class SerialLink:
    """Non-blocking writer. Acknowledgements are drained, never waited for.

    Blocking on "OK" would put the camera and the detector to sleep every frame.
    Instead we count acknowledgements and only complain if they stop arriving.
    """

    def __init__(self, cfg: Config):
        # Imported here so the regulator can be imported and tested without
        # pyserial installed.
        import serial

        self.ser = serial.Serial(
            cfg.port, cfg.baud, timeout=0, write_timeout=cfg.ack_timeout_s
        )
        self.acks = 0
        self.timeouts = 0
        self.error: Optional[str] = None
        self._outstanding = 0
        self._last_rx = time.monotonic()
        self._ever_rx = False
        self._rxbuf = b""

    @property
    def stalled(self) -> bool:
        if not self._ever_rx:
            # Nothing seen yet; only a real problem once we have sent a few.
            return self._outstanding >= 3
        return (time.monotonic() - self._last_rx) > 3.0

    def send(self, command: Command) -> None:
        pan, tilt = command
        self.ser.write(f"P{pan:+d} T{tilt:+d}\n".encode("ascii"))
        self._outstanding += 1

    def poll(self) -> None:
        waiting = self.ser.in_waiting
        if not waiting:
            return
        # Split on newlines: the Arduino also sends READY / HOMED / PAN / ERR
        # lines, and a partial read can slice a line in half, so keep a buffer.
        self._rxbuf += self.ser.read(waiting)
        self._last_rx = time.monotonic()
        self._ever_rx = True

        while b"\n" in self._rxbuf:
            line, self._rxbuf = self._rxbuf.split(b"\n", 1)
            text = line.strip().decode("ascii", "replace")
            if not text:
                continue
            if text == "OK":
                self.acks += 1
                self._outstanding = max(0, self._outstanding - 1)
            elif text in ("READY", "HOMED"):
                log.info("arduino: %s", text)
            elif text.startswith("ERR"):
                log.error("arduino: %s", text)
                self.error = text
            else:
                # Calibration report and anything else; not a protocol error.
                log.info("arduino: %s", text)

    def close(self) -> None:
        try:
            self.send((0, 0))
            time.sleep(0.05)
        except Exception:
            pass
        self.ser.close()


# --------------------------------------------------------------------------- #
# the loop
# --------------------------------------------------------------------------- #


def run(cfg: Config) -> None:
    camera = Camera(cfg)
    detector = YoloDetector(cfg)
    link = SerialLink(cfg)
    controller = PController(cfg)

    log.info("tracking started: port=%s target=%s", cfg.port, cfg.target_class)

    frames = 0
    lost = 0
    loop_start = time.monotonic()
    log_start = loop_start
    frame_w = cfg.width
    frame_h = cfg.height

    try:
        while True:
            frame = camera.read()
            frame_h, frame_w = frame.shape[:2]
            link.poll()

            box = detector.detect(frame)

            if box is None:
                lost += 1
                if lost == cfg.lost_frames_before_stop:
                    controller.reset()
                    link.send((0, 0))
                    log.info("target lost, motors stopped")
            else:
                lost = 0
                err_x, err_y = centre_error(box, frame_w, frame_h)
                command = controller.update(err_x, err_y)
                if command is not None:
                    link.send(command)

            if link.error:
                log.error("arduino refused motion (%s), aborting", link.error)
                break

            frames += 1
            now = time.monotonic()
            if now - log_start >= cfg.log_interval_s:
                elapsed = now - loop_start
                log.info(
                    "%.1f fps | box=%s | out=%d acks=%d",
                    frames / elapsed,
                    "none" if box is None else "yes",
                    link._outstanding,
                    link.acks,
                )
                frames, loop_start = 0, now
                log_start = now
                if link.stalled:
                    log.warning("no OK from Arduino for 3s - check wiring and baud")

    except KeyboardInterrupt:
        log.info("interrupted")
    finally:
        link.close()
        camera.close()
        log.info("cleaned up")


# --------------------------------------------------------------------------- #
# cli
# --------------------------------------------------------------------------- #


def main() -> None:
    p = argparse.ArgumentParser(description="Pan/tilt tracking loop")
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--model", default="yolov8n.pt")
    p.add_argument("--target", default="person", help="class name, or 'any'")
    p.add_argument("--pan-gain", type=float, default=0.05)
    p.add_argument("--tilt-gain", type=float, default=0.05)
    p.add_argument("--pan-sign", type=int, choices=(-1, 1), default=1)
    p.add_argument("--tilt-sign", type=int, choices=(-1, 1), default=-1)
    p.add_argument("--deadband", type=int, default=12)
    p.add_argument("--max-step", type=int, default=25)
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )

    cfg = Config(
        port=args.port,
        camera_index=args.camera,
        model=args.model,
        target_class=None if args.target.lower() == "any" else args.target,
        pan_gain=args.pan_gain,
        tilt_gain=args.tilt_gain,
        pan_sign=args.pan_sign,
        tilt_sign=args.tilt_sign,
        deadband_px=args.deadband,
        max_step=args.max_step,
    )
    run(cfg)


if __name__ == "__main__":
    main()
