"""
Simulator for the tracking loop.

Runs tracker.py's logic against a fake turret and a fake target so you can
check the regulator, the axis signs and the serial protocol without any
hardware. Nothing here talks to a camera or a real serial port.

What it models
--------------
  * The turret as two axes that integrate speed into position, with travel
    limits and endstops, so an inverted sign shows up as the turret driving
    into a stop instead of chasing the target.
  * A target that moves in a simple way, so you can watch gain and deadband
    behave.
  * The real command format and the "OK" acknowledgement.

What it does NOT model
----------------------
  * Camera latency, motion blur, detection dropout, or a slow Pi. Those are
    usually what make a real rig oscillate, so passing here does not mean
    passing on hardware.
  * Backlash, stepper resonance, or mechanical inertia.
"""

from __future__ import annotations

import argparse
import logging
import time
from typing import List, Optional, Tuple

from tracker import PController, SerialLink, centre_error, Config

log = logging.getLogger("sim")

Box = Tuple[float, float, float, float]

# Must match UNITS_TO_SPS in TURRET.ino, otherwise the simulated turret
# responds at a different rate than the real one.
UNITS_TO_SPS = 40.0


# --------------------------------------------------------------------------- #
# fake turret: the Arduino side, in Python
# --------------------------------------------------------------------------- #


class FakeTurret:
    """Integrates commands into motion, like the stepper sketch does.

    Speeds are in steps/second and positions in steps, so the numbers in the
    log line up with what you would see with a scope on the step pin.
    """

    def __init__(
        self,
        width_px: int = 640,
        height_px: int = 480,
        units_to_sps: float = 40.0,
        max_speed_sps: float = 400.0,
        travel: int = 4000,
    ):
        self.width_px = width_px
        self.height_px = height_px
        self.units_to_sps = units_to_sps
        self.max_speed_sps = max_speed_sps
        self.travel = travel

        self.pan = 0  # steps, 0 = centre
        self.tilt = 0
        self.pan_speed = 0.0
        self.tilt_speed = 0.0
        self.last_command: Optional[Tuple[int, int]] = None
        self.acked: List[Tuple[int, int]] = []
        self.at_limit = False
        # Tilt only moves in one direction on most real mounts.
        self.tilt_limits = (0, travel)

    def send(self, command: Tuple[int, int]) -> None:
        pan_units, tilt_units = command
        self.last_command = command
        self.acked.append(command)

        # Mirrors the constrain() in the sketch.
        pan_units = max(-12, min(12, pan_units))
        tilt_units = max(-12, min(12, tilt_units))

        self.pan_speed = pan_units * self.units_to_sps
        self.tilt_speed = tilt_units * self.units_to_sps
        if abs(self.pan_speed) > self.max_speed_sps:
            self.pan_speed = max(-self.max_speed_sps, self.max_speed_sps)
        if abs(self.tilt_speed) > self.max_speed_sps:
            self.tilt_speed = max(-self.max_speed_sps, self.max_speed_sps)

    def step(self, dt: float) -> None:
        self.pan += self.pan_speed * dt
        self.tilt += self.tilt_speed * dt

        # Endstops. A real rig trips these and either stops or errors out.
        hit = False
        if self.pan < -self.travel or self.pan > self.travel:
            self.pan = max(-self.travel, min(self.travel, self.pan))
            self.pan_speed = 0.0
            hit = True
        lo, hi = self.tilt_limits
        if self.tilt < lo or self.tilt > hi:
            self.tilt = max(lo, min(hi, self.tilt))
            self.tilt_speed = 0.0
            hit = True
        self.at_limit = hit

    # --- the geometry the regulator sees ----------------------------------- #
    #
    # The target has a fixed position in the room. The turret has an
    # orientation. The image error is the difference between them, so the
    # regulator has to chase a target that does not move when the camera
    # does. Modelling it the other way round would make the error always
    # zero and hide every sign error.

    def px_per_step(self) -> float:
        return self.width_px / (self.travel * 2.0)

    def seen_box(self, target: Tuple[float, float], size: int = 80) -> Box:
        """Where a world-space target appears in the image.

        `target` is (world_x_steps, world_y_steps), measured from the centre of
        the reachable area with positive x to the right and positive y up.
        """
        scale = self.px_per_step()
        cx = self.width_px / 2.0 + (target[0] - self.pan) * scale
        # Tilt reads positive downwards in the image, hence the negation.
        cy = self.height_px / 2.0 - (target[1] - self.tilt) * scale
        return (cx - size / 2, cy - size / 2, cx + size / 2, cy + size / 2)


# --------------------------------------------------------------------------- #
# fake target
# --------------------------------------------------------------------------- #


class Target:
    """A world-space position, in steps, that the camera has to follow.

    Positive x is to the right of the turret's centre, positive y is up. The
    turret starts at (0, 0) pointing at the origin, so a target away from the
    origin is immediately off-centre in the image.
    """

    def __init__(self, x: float, y: float, path: str = "circle", reach: int = 4000):
        self.x = x
        self.y = y
        self.path = path
        self.reach = reach
        self.t = 0.0
        self.visible = True

    def update(self, dt: float) -> None:
        import math
        import random

        self.t += dt
        if self.path == "static":
            return
        if self.path == "circle":
            r = self.reach * 0.6
            self.x = r * math.cos(self.t * 0.6)
            self.y = r * 0.5 * math.sin(self.t * 0.6)
        elif self.path == "jitter":
            self.x += random.uniform(-0.4, 0.4) * self.reach * 0.01
            self.y += random.uniform(-0.3, 0.3) * self.reach * 0.01
            self.x = max(-self.reach, min(self.reach, self.x))
            self.y = max(-self.reach, min(self.reach, self.y))

    def hide(self) -> None:
        """Simulate a detection dropout."""
        self.visible = False

    def show(self) -> None:
        self.visible = True


# --------------------------------------------------------------------------- #
# the loop, wired to the simulator
# --------------------------------------------------------------------------- #


class FakeSerial:
    """Stands in for SerialLink so tracker.py's code path stays the same."""

    def __init__(self, turret: FakeTurret):
        self.turret = turret
        self.acks = 0
        self.error = None
        self.sent = 0

    def send(self, command) -> None:
        self.sent += 1
        self.turret.send(command)

    def poll(self) -> None:
        # Acknowledge everything immediately, as a real sketch would.
        self.acks += len(self.turret.acked)
        self.turret.acked.clear()

    def close(self) -> None:
        self.send((0, 0))


def final_error_px(turret: FakeTurret, target: Target) -> float:
    """How far the target ended up from the image centre, in pixels."""
    err_x, _ = centre_error(
        turret.seen_box((target.x, target.y)), turret.width_px, turret.height_px
    )
    return abs(err_x)


def run_sim(cfg: Config, seconds: float, path: str, dt: float, verbose: bool) -> int:
    turret = FakeTurret(
        width_px=cfg.width,
        height_px=cfg.height,
        units_to_sps=UNITS_TO_SPS,
    )
    # Start the target well off-axis so the regulator has real work to do.
    target = Target(turret.travel * 0.5, turret.travel * 0.3, path, turret.travel)
    link = FakeSerial(turret)
    controller = PController(cfg)

    log.info(
        "simulating %.0fs, path=%s, gain pan=%.3f tilt=%.3f, sign pan=%+d tilt=%+d",
        seconds, path, cfg.pan_gain, cfg.tilt_gain, cfg.pan_sign, cfg.tilt_sign,
    )

    steps = int(seconds / dt)
    limits_hit = 0
    log_every = max(1, int(0.5 / dt))

    for i in range(steps):
        # A dropout around t=2s and t=4s, to exercise the lost-target path.
        if path == "dropout" and abs(i * dt - 2.0) < dt / 2:
            target.hide()
        if path == "dropout" and abs(i * dt - 4.0) < dt / 2:
            target.show()

        target.update(dt)
        link.poll()

        box = turret.seen_box((target.x, target.y)) if target.visible else None
        if box is None:
            controller.reset()
            link.send((0, 0))
        else:
            err_x, err_y = centre_error(box, cfg.width, cfg.height)
            command = controller.update(err_x, err_y)
            if command is not None:
                link.send(command)

        turret.step(dt)
        if turret.at_limit:
            limits_hit += 1

        if verbose and i % log_every == 0:
            ex, ey = centre_error(
                turret.seen_box((target.x, target.y)), cfg.width, cfg.height
            )
            log.info(
                "t=%4.1f pan=%6.0f tilt=%6.0f err=(%6.1f,%6.1f) cmd=%s",
                i * dt, turret.pan, turret.tilt, ex, ey, turret.last_command,
            )

    final = final_error_px(turret, target)
    log.info("---")
    log.info("commands sent: %d", link.sent)
    log.info("final error: %.1f px (deadband is %d px)", final, cfg.deadband_px)
    if limits_hit:
        log.info("endstop reached: %d times", limits_hit)

    # Diagnostic, not a verdict. Each pattern points at a different mistake,
    # but a real rig can also hit a stop simply because the target walked
    # out of reach, so read the pattern rather than the message.
    if limits_hit >= 2 and final <= cfg.deadband_px:
        log.warning(
            "an axis is pinned against a stop yet the error is small. That is "
            "the signature of an inverted sign: the turret is facing the wrong "
            "way and only looks centred because it is driven into the stop. "
            "Try flipping that axis sign."
        )
    elif final > 100:
        log.warning(
            "ended %.0f px off target, far outside the %.0f px deadband. The "
            "regulator is not closing the loop: try much lower gain, or an "
            "inverted sign.", final, cfg.deadband_px
        )
    elif final > cfg.deadband_px * 3:
        log.warning(
            "ended %.0f px off target, outside the %.0f px deadband. Often just "
            "too little time, or gain too low to settle.",
            final, cfg.deadband_px
        )
    else:
        log.info("settled inside the deadband")
    return limits_hit


def main() -> None:
    p = argparse.ArgumentParser(description="Simulate the tracking loop")
    p.add_argument("--seconds", type=float, default=12.0)
    p.add_argument("--path", default="circle",
                   choices=["circle", "static", "jitter", "dropout"])
    p.add_argument("--dt", type=float, default=1 / 30)
    p.add_argument("--pan-gain", type=float, default=0.05)
    p.add_argument("--tilt-gain", type=float, default=0.05)
    p.add_argument("--pan-sign", type=int, choices=(-1, 1), default=1)
    p.add_argument("--tilt-sign", type=int, choices=(-1, 1), default=-1)
    p.add_argument("--deadband", type=int, default=12)
    p.add_argument("--max-step", type=int, default=25)
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(message)s",
    )

    cfg = Config(
        pan_gain=args.pan_gain,
        tilt_gain=args.tilt_gain,
        pan_sign=args.pan_sign,
        tilt_sign=args.tilt_sign,
        deadband_px=args.deadband,
        max_step=args.max_step,
    )
    run_sim(cfg, args.seconds, args.path, args.dt, args.verbose)


if __name__ == "__main__":
    main()
