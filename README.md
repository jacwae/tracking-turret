# tracking-turret

Pan/tilt turret that keeps a camera pointed at a detected object.

```
camera -> detect -> centre error -> P regulator -> serial -> stepper motors
```

## Layout

```
arduino/TURRET/TURRET.ino    motor control, homing, command parsing
python_detection/
  tracker.py                 the loop: camera, detection, regulator, serial
  simulate.py                runs the same regulator against a fake turret
  requirements.txt
  test.py                    scratch file, unused
```

## Simulation

No hardware needed. It runs the real `PController` from `tracker.py` against a
simulated turret and target, so gain, deadband and axis signs can be checked
before anything is wired up.

```bash
pip install -r requirements.txt   # only opencv/pyserial/ultralytics are needed on the Pi
python simulate.py --path circle -v          # follow a moving target
python simulate.py --path static --seconds 20 # check that it settles
python simulate.py --path jitter            # noisy target
python simulate.py --path dropout           # detection lost and regained
```

Useful flags: `--pan-gain`, `--tilt-gain`, `--pan-sign`, `--tilt-sign`,
`--deadband`, `--max-step`.

What the simulator does *not* model, and what will therefore still surprise you
on real hardware: camera latency, motion blur, detection dropouts at speed,
frame rate below 30 fps, stepper resonance, backlash, and mechanical inertia.
A rig that behaves here can still oscillate on the bench, usually because the
Pi cannot keep up with the camera.

## Running on the Pi

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
sudo usermod -aG dialout $USER    # access to /dev/ttyACM0, needs re-login
python tracker.py --port /dev/ttyACM0 --target person
```

Needs the `AccelStepper` library in the Arduino IDE (Boards Manager).

## Protocol

One command per line, ASCII, LF terminated.

| Direction | Line | Meaning |
|---|---|---|
| Pi -> Arduino | `P-12 T3` | pan -12, tilt +3, in speed units (steps/s) |
| Arduino -> Pi | `OK` | one per command received |
| Arduino -> Pi | `READY`, `HOMED` | startup and homing status |
| Arduino -> Pi | `PAN 3820` | measured travel in steps |
| Arduino -> Pi | `ERR not homed` | homing failed, no motion accepted |

A command is a *speed*, not a distance. The Pi resends every frame, so the
motors hold the last commanded speed until the next command arrives or the
Arduino's 250 ms stall timeout stops them.

## Tuning order

Tune in this order, and do not move to the next step until the current one is
stable.

1. **Axis signs.** `pan_sign` and `tilt_sign` in `tracker.py`. If an axis
   drives away from the target, flip it. A wrong sign makes the loop diverge,
   not merely perform badly.
2. **`max_step`** and `UNITS_TO_SPS` in the sketch. Set these so the turret
   covers the full frame in roughly a second, then work the gain back from
   there.
3. **`deadband_px`.** Around 10-15. Below about 8 the turret will visibly
   jitter; above about 25 it stops early enough to look lazy.
4. **Gain**, `pan_gain` and `tilt_gain`. Start at 0.02. Raise slowly. Sign of
   overshoot means too high.

If it oscillates, the cause is nearly always loop delay, not gain. Check that
the Pi reaches 30 fps before touching the gain further.

## Homing

The sketch homes both axes against their endstops in `setup()` and reports the
measured travel as `PAN <steps>` / `TILT <steps>`. Put those numbers into
`MAX_TRAVEL`. The initial value of 4000 is a placeholder, and homing fails with
`ERR homing travel limit` if the real travel is larger.

The turret stays disabled and ignores all motion until homing succeeds, so it
will not move to a position it has no reference for. Homing runs immediately at
power-on, which means the motors move for a few seconds while the camera is
still warming up.
