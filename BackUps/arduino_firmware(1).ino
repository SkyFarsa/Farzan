// -------------------------
// ARDUINO FIRMWARE (debugged)
// For serial_odom_node.py
//
// Fixes applied vs original:
//   1. Encoder print bug fixed (stray "x" char, missing "l" print)
//   2. Added "R" serial command to zero tick counters (needed by /reset_odom service)
// -------------------------

// Motor pins
#define L_IN1 7
#define L_IN2 6
#define R_IN1 5
#define R_IN2 4

// ENABLE PINS (PWM SPEED CONTROL)
#define ENA 8   // Left motor enable
#define ENB 9   // Right motor enable

// ── Minimum effective PWM (motor stiction compensation) ─────────────────────
// These motors are direct-drive with NO gearbox, so static friction is high
// enough that small PWM values just make the motor buzz/whine without
// actually turning - not enough torque to break stiction. This matters most
// with Nav2, since its velocity_smoother deliberately RAMPS commands up
// smoothly from 0, meaning it spends real time commanding tiny velocities
// (and therefore tiny PWM) right at the start of every motion - precisely
// where this problem bites. Plain teleop doesn't hit this because it sends
// large, immediate velocity steps that are already above the stall point.
//
// PLACEHOLDER VALUE - measure your actual motors' real stall threshold:
// send raw "P,pwm,pwm" commands over serial (bypass everything else) at
// increasing pwm from ~30 upward until the motor visibly/audibly starts
// turning smoothly (not just buzzing), then set this a little above that.
#define MIN_EFFECTIVE_PWM 75

// Encoders
#define ENC_L_A 18
#define ENC_L_B 19
#define ENC_R_A 3
#define ENC_R_B 2

volatile long left_ticks = 0;
volatile long right_ticks = 0;
unsigned long last_send = 0;

void leftISR() {
  if (digitalRead(ENC_L_B)) left_ticks++;
  else left_ticks--;
}

void rightISR() {
  if (digitalRead(ENC_R_B)) right_ticks++;
  else right_ticks--;
}

void setup() {
  // Configure + zero motor pins FIRST, before Serial.begin() or anything
  // else — minimizes the boot-time window where pins are floating/undefined
  // and can cause a brief unintended spin on power-up/reset.
  pinMode(L_IN1, OUTPUT);
  digitalWrite(L_IN1, LOW);
  pinMode(L_IN2, OUTPUT);
  digitalWrite(L_IN2, LOW);
  pinMode(R_IN1, OUTPUT);
  digitalWrite(R_IN1, LOW);
  pinMode(R_IN2, OUTPUT);
  digitalWrite(R_IN2, LOW);
  pinMode(ENA, OUTPUT);
  analogWrite(ENA, 0);
  pinMode(ENB, OUTPUT);
  analogWrite(ENB, 0);

  Serial.begin(115200);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, RISING);
}

void setMotor(int in1, int in2, int enPin, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    // Below the stall threshold, boost to the minimum that actually turns
    // the motor - a small-but-nonzero command should still MOVE, not buzz.
    if (pwm < MIN_EFFECTIVE_PWM) pwm = MIN_EFFECTIVE_PWM;
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enPin, pwm);
  } else if (pwm < 0) {
    if (pwm > -MIN_EFFECTIVE_PWM) pwm = -MIN_EFFECTIVE_PWM;
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enPin, -pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enPin, 0);
  }
}

void setMotors(int pwmL, int pwmR) {
  setMotor(L_IN1, L_IN2, ENA, pwmL);
  setMotor(R_IN1, R_IN2, ENB, pwmR);
}

void loop() {
  // ---- Read Serial Commands ----
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();

    if (s.startsWith("P")) {
      int first = s.indexOf(',');
      int second = s.indexOf(',', first + 1);
      if (first > 0 && second > first) {
        int pwmL = s.substring(first + 1, second).toInt();
        int pwmR = s.substring(second + 1).toInt();
        setMotors(pwmL, pwmR);
      }
    } else if (s == "R") {
      // Reset tick counters (called by serial_odom_node's /reset_odom service)
      noInterrupts();
      left_ticks = 0;
      right_ticks = 0;
      interrupts();
    }
  }

  // ---- Send Encoder Data Every 50ms ----
  unsigned long now = millis();
  if (now - last_send > 50) {
    noInterrupts();
    long l = left_ticks;
    long r = right_ticks;
    interrupts();

    Serial.print("E,");
    Serial.print(l);
    Serial.print(",");
    Serial.print(r);
    Serial.print(",");
    Serial.println(now);

    last_send = now;
  }
}
