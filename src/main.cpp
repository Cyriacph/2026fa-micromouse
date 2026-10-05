#include <Arduino.h>

#include "config.h"
#include "wiring.h"
#include "motors.h"

// Pin assignments
static const int LEFT_FWD  = MTR_A_IN1;  // MX1508 IN1
static const int LEFT_REV  = MTR_A_IN2;  // MX1508 IN2
static const int RIGHT_FWD = MTR_B_IN1;  // MX1508 IN3
static const int RIGHT_REV = MTR_B_IN2;  // MX1508 IN4

// High-level controller for both motors
Motors motors;

// Encoders: yellow = tick signal, green = direction
static const int ENC_L_YELLOW = ENC_A_CH_A;
static const int ENC_L_GREEN  = ENC_A_CH_B;
static const int ENC_R_YELLOW = ENC_B_CH_A;
static const int ENC_R_GREEN  = ENC_B_CH_B;

// volatile because the ISRs modify them outside the main program flow
static volatile long encLeft = 0, encRight = 0;

// One tick = one RISING edge on yellow. Green = directoin: HIGH = forward, LOW = backward
void IRAM_ATTR isrLeft()  { encLeft  += (digitalRead(ENC_L_GREEN) ? 1 : -1); }
void IRAM_ATTR isrRight() { encRight += (digitalRead(ENC_R_GREEN) ? 1 : -1); }

// MX1508: fwd pin gets PWM to go forward, rev pin for backward, both LOW = coast
void setMotor(int fwdPin, int revPin, int speed) {
  int fwdCh = (fwdPin == LEFT_FWD) ? CH_L_FWD : CH_R_FWD;
  int revCh = (fwdPin == LEFT_FWD) ? CH_L_REV : CH_R_REV;
  if      (speed > 0) { ledcWrite(fwdCh, speed); ledcWrite(revCh, 0); }
  else if (speed < 0) { ledcWrite(fwdCh, 0);     ledcWrite(revCh, -speed); }
  else                { ledcWrite(fwdCh, 0);     ledcWrite(revCh, 0); }
}

// MX1508: full HIGH both sides = short brake, both LOW = coast
void brakeMotor(int fwdPin, int revPin) {
  int fwdCh = (fwdPin == LEFT_FWD) ? CH_L_FWD : CH_R_FWD;
  int revCh = (fwdPin == LEFT_FWD) ? CH_L_REV : CH_R_REV;
  ledcWrite(fwdCh, MAX_PWM);   // full HIGH both sides = short brake
  ledcWrite(revCh, MAX_PWM);
}

// Run a motor for 1.2s, brake, print how many ticks it did
void testRun(const char* name, int fwdPin, int revPin, int speed, volatile long& enc) {
  long before = enc;
  Serial.printf("[%s] speed=%d  ", name, speed);
  setMotor(fwdPin, revPin, speed);
  delay(1200);
  brakeMotor(fwdPin, revPin);
  setMotor(fwdPin, revPin, 0);
  delay(500);
  Serial.printf("encoder delta = %ld\n", (long)enc - before);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configure the encoder pins and attach interrupts to count ticks
  pinMode(ENC_L_YELLOW, INPUT); pinMode(ENC_L_GREEN,  INPUT);
  pinMode(ENC_R_YELLOW, INPUT); pinMode(ENC_R_GREEN,  INPUT);
  attachInterrupt(ENC_L_YELLOW, isrLeft,  RISING);
  attachInterrupt(ENC_R_YELLOW, isrRight, RISING);

  // Configure the motors with the correct pins and PWM settings
  MotorConfig motorConfig;
  motorConfig.left.in1 = LEFT_FWD;
  motorConfig.left.in2 = LEFT_REV;
  motorConfig.left.ch1 = CH_L_FWD;
  motorConfig.left.ch2 = CH_L_REV;
  motorConfig.right.in1 = RIGHT_FWD;
  motorConfig.right.in2 = RIGHT_REV;
  motorConfig.right.ch1 = CH_R_FWD;
  motorConfig.right.ch2 = CH_R_REV;
  motorConfig.pwm_freq = PWM_FREQUENCY;
  motorConfig.pwm_res = PWM_RESOLUTION;
  motorConfig.max_pwm = MAX_PWM;
  motors.begin(motorConfig);

  Serial.println("\n=== Motor bench test ===");
  delay(3000); // hands clear before the motors move

  // fwd should give positive deltas, rev negative ones
  testRun("LEFT fwd ", LEFT_FWD, LEFT_REV,  150, encLeft);
  testRun("LEFT rev ", LEFT_FWD, LEFT_REV, -150, encLeft);
  testRun("RIGHT fwd", RIGHT_FWD, RIGHT_REV, 150, encRight);
  testRun("RIGHT rev", RIGHT_FWD, RIGHT_REV, -150, encRight);

  // feel the difference: coast = slow stop, brake = instant stop
  Serial.println("\n--- Coast vs brake (left motor) ---");
  setMotor(LEFT_FWD, LEFT_REV, 200);
  delay(1500);
  setMotor(LEFT_FWD, LEFT_REV, 0);  // coast
  delay(1500);
  setMotor(LEFT_FWD, LEFT_REV, 200);
  delay(1500);
  brakeMotor(LEFT_FWD, LEFT_REV);    // brake
  delay(1000);

  Serial.println("\nSpin the shafts by hand, watch the counts:");
}

// Just prints encoder counts every 200 ms for the manual test
void loop() {
  static unsigned long last = 0;
  if (millis() - last >= 200) {
    last = millis();
    Serial.printf("encLeft=%ld  encRight=%ld\n", (long)encLeft, (long)encRight);
  }
}