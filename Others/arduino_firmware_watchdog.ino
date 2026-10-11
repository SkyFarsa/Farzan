// -------------------------
// ARDUINO FIRMWARE (debugged)
// For serial_odom_node.py
//
// Fixes applied vs original:
//   1. Encoder print bug fixed (stray "x" char, missing "l" print)
//   3. Added command watchdog (CMD_TIMEOUT_MS) that stops motors if no P command arrives
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

// Encoders
#define ENC_L_A 18
#define ENC_L_B 19
#define ENC_R_A 3
#define ENC_R_B 2

volatile long left_ticks = 0;
volatile long right_ticks = 0;
unsigned long last_send = 0;

// ---- Command watchdog ----
// If no valid "P,l,r" command arrives within CMD_TIMEOUT_MS, motors are stopped.
// Nav2 publishes cmd_vel continuously (fine). teleop_twist_keyboard only publishes
// on key presses: hold the key, or raise this value for manual driving.
#define CMD_TIMEOUT_MS 500
unsigned long last_cmd = 0;
bool motors_active = false;

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
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enPin, pwm);
  } else if (pwm < 0) {
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
        last_cmd = millis();
        motors_active = (pwmL != 0 || pwmR != 0);
      }
    } else if (s == "R") {
      // Reset tick counters (called by serial_odom_node's /reset_odom service)
      noInterrupts();
      left_ticks = 0;
      right_ticks = 0;
      interrupts();
    }
  }

  // ---- Watchdog: stop motors if commands stop arriving ----
  if (motors_active && (millis() - last_cmd > CMD_TIMEOUT_MS)) {
    setMotors(0, 0);
    motors_active = false;
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
