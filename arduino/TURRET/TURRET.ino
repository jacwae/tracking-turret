/*
  TurretStepper.ino  -  pan/tilt stepper motors with endstop homing

  Companion to tracker.py on the Raspberry Pi.

  Protocol
  --------
  In:   "P-12 T3\n"     pan speed -12..12, tilt speed -12..3, signs explicit
  Out:  "OK\n"          one ack per command received

  The Pi re-sends a command every camera frame, so a command is a *speed*
  (steps per second), not a distance. The motors keep turning at that speed
  until the next command arrives or STALL_TIMEOUT expires. That makes the Pi
  loop the outer loop and this sketch only the inner one.

  Wiring (per axis)
  ----------------
    DRV   -> STEP pin
    DIR   -> DIR pin
    EN    -> driver enable (active low on most A4988/DRV8825 boards)
    MIN   -> endstop to GND, INPUT_PULLUP
    MAX   -> endstop to GND, INPUT_PULLUP
    supply -> motor driver VMOT, common GND with the Arduino

  IMPORTANT: keep the Arduino and the Pi on a common ground, and power the
  motors from a supply that can take the stall current. Do not power steppers
  from the Arduino's 5V or 3V3 pin.
*/

#include <AccelStepper.h>

// --- pins -----------------------------------------------------------------
constexpr uint8_t PAN_STEP   = 2;
constexpr uint8_t PAN_DIR    = 3;
constexpr uint8_t PAN_EN     = 4;   // active low
constexpr uint8_t PAN_MIN    = 5;   // endstops to GND, INPUT_PULLUP
constexpr uint8_t PAN_MAX    = 6;

constexpr uint8_t TILT_STEP  = 7;
constexpr uint8_t TILT_DIR   = 8;
constexpr uint8_t TILT_EN    = 9;   // active low
constexpr uint8_t TILT_MIN   = 10;
constexpr uint8_t TILT_MAX   = 11;

// --- motion ----------------------------------------------------------------
constexpr float MAX_SPEED      = 400.0f;  // steps/second, tune to your motor
constexpr float ACCELERATION   = 200.0f;  // steps/second^2
constexpr int   MAX_TRAVEL     = 4000;   // steps between endstops, per axis
constexpr float HOMING_SPEED   = 200.0f;
constexpr long  STALL_TIMEOUT  = 250;     // ms without a command, then stop

// One command unit == this many steps/second.
constexpr float UNITS_TO_SPS = 40.0f;

// Guard rails on the incoming values.
constexpr int MAX_PAN_UNITS  = 12;
constexpr int MAX_TILT_UNITS = 12;

AccelStepper panMotor(AccelStepper::DRIVER, PAN_STEP, PAN_DIR);
AccelStepper tiltMotor(AccelStepper::DRIVER, TILT_STEP, TILT_DIR);

long  lastCommandMs = 0;
bool  homed         = false;
bool  ackPending    = false;

uint8_t pinForEndstop(int axis, int end) {  // end: 0 = min, 1 = max
  return (axis == 0) ? (end == 0 ? PAN_MIN : PAN_MAX)
                     : (end == 0 ? TILT_MIN : TILT_MAX);
}

AccelStepper& motorFor(int axis) {
  return (axis == 0) ? panMotor : tiltMotor;
}

// An endstop reads LOW because the switch shorts to GND with INPUT_PULLUP.
bool atEndstop(int axis, int end) {
  return digitalRead(pinForEndstop(axis, end)) == LOW;
}

// Block until the switch trips, the travel limit is hit, or time runs out.
// Returns the number of steps actually travelled, or -1 on failure.
long homeAxis(int axis, int end) {
  motorFor(axis).setSpeed((end == 0) ? HOMING_SPEED : -HOMING_SPEED);
  motorFor(axis).setAcceleration(ACCELERATION);
  const long start = millis();
  long travelled = 0;

  while (!atEndstop(axis, end)) {
    travelled += (end == 0) ? 1 : -1;
    if (abs(travelled) > MAX_TRAVEL) {
      Serial.println("ERR homing travel limit");
      return -1;
    }
    if (millis() - start > 10000UL) {
      Serial.println("ERR homing timeout");
      return -1;
    }
    motorFor(axis).run();
  }
  motorFor(axis).stop();
  return abs(travelled);
}

void setEnables(bool on) {
  digitalWrite(PAN_EN,  on ? HIGH : LOW);   // active low
  digitalWrite(TILT_EN, on ? HIGH : LOW);
}

// --- command parsing -------------------------------------------------------
//
// "P-12 T3"  ->  pan -12, tilt 3. Both signs are always written by the Pi,
// but accept a bare "12" as well so you can test by hand from a serial monitor.
bool readSigned(char* buf, int& out) {
  char* end = nullptr;
  long v = strtol(buf, &end, 10);
  if (end == buf) return false;
  out = (int)v;
  return true;
}

void handleCommand(char* line) {
  char* pPan  = nullptr;
  char* pTilt = nullptr;

  for (char* p = line; *p; ++p) {
    if (*p == 'P' || *p == 'p') pPan = p + 1;
    else if (*p == 'T' || *p == 't') pTilt = p + 1;
  }

  int panUnits = 0, tiltUnits = 0;
  if (pPan  && !readSigned(pPan,  panUnits))  return;
  if (pTilt && !readSigned(pTilt, tiltUnits)) return;

  panUnits  = constrain(panUnits,  -MAX_PAN_UNITS,  MAX_PAN_UNITS);
  tiltUnits = constrain(tiltUnits, -MAX_TILT_UNITS, MAX_TILT_UNITS);

  // Ignore motion until the axes have a known position, otherwise the turret
  // has no idea where "centre" is and will drive into the stops.
  if (homed) {
    panMotor.setSpeed(panUnits * UNITS_TO_SPS);
    tiltMotor.setSpeed(tiltUnits * UNITS_TO_SPS);
  }

  lastCommandMs = millis();
  ackPending = true;
}

// --- setup -----------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PAN_EN, OUTPUT);
  pinMode(TILT_EN, OUTPUT);
  for (int axis = 0; axis < 2; ++axis) {
    for (int end = 0; end < 2; ++end) pinMode(pinForEndstop(axis, end), INPUT_PULLUP);
  }
  setEnables(false);

  panMotor.setMaxSpeed(MAX_SPEED);
  panMotor.setAcceleration(ACCELERATION);
  tiltMotor.setMaxSpeed(MAX_SPEED);
  tiltMotor.setAcceleration(ACCELERATION);

  Serial.println("READY");
  setEnables(true);
  delay(50);

  // Home immediately. The Pi sends nothing until it has a target to chase, so
  // waiting for the first command here would deadlock the whole rig.
  const long panMinSteps = homeAxis(0, 0);
  const long panMaxSteps = homeAxis(0, 1);
  const long tiltMinSteps = homeAxis(1, 0);
  const long tiltMaxSteps = homeAxis(1, 1);

  if (panMinSteps < 0 || panMaxSteps < 0 || tiltMinSteps < 0 || tiltMaxSteps < 0) {
    Serial.println("ERR not homed");   // motors stay disabled, no motion accepted
    return;
  }

  panMotor.setCurrentPosition(0);
  tiltMotor.setCurrentPosition(0);
  homed = true;

  // Report measured travel so the constants can be set from real numbers.
  Serial.print("PAN ");
  Serial.println(panMinSteps + panMaxSteps);
  Serial.print("TILT ");
  Serial.println(tiltMinSteps + tiltMaxSteps);
  Serial.println("HOMED");
}

// --- loop ------------------------------------------------------------------
void loop() {
  // If the Pi goes away, stop rather than run the motors into the stops.
  if (millis() - lastCommandMs > STALL_TIMEOUT) {
    panMotor.setSpeed(0);
    tiltMotor.setSpeed(0);
  }

  panMotor.run();
  tiltMotor.run();

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      static char buf[32];
      static uint8_t len = 0;
      if (len > 0) {
        buf[len] = '\0';
        handleCommand(buf);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  // One ack per command. Batched, so a burst of commands does not flood the Pi.
  if (ackPending && !Serial.available()) {
    Serial.println("OK");
    ackPending = false;
  }
}
