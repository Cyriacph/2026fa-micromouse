#include <Arduino.h>

#include "config.h"
#include "wiring.h"
#include "motors.h"
#include "encoders.h"

// Pin assignments
static const int LEFT_FWD  = MTR_A_IN1;  // MX1508 IN1
static const int LEFT_REV  = MTR_A_IN2;  // MX1508 IN2
static const int RIGHT_FWD = MTR_B_IN1;  // MX1508 IN3
static const int RIGHT_REV = MTR_B_IN2;  // MX1508 IN4

// High-level controller for both motors
Motors motors;

// Read the encoder associated with the motor being tested.
long readEncoder(bool leftMotor) {
  return leftMotor ? getEncoderLeft() : getEncoderRight();
}

// Run one motor for 1.2 seconds, brake, and print its encoder change.
void testRun(const char* name, bool leftMotor, int speed) {
  const long before = readEncoder(leftMotor);
  Serial.printf("[%s] speed=%d  ", name, speed);
  if (leftMotor) {
    motors.setLeft(speed);
  } else {
    motors.setRight(speed);
  }
  delay(1200);
  motors.brake();
  motors.stop();
  delay(500);
  const long after = readEncoder(leftMotor);
  Serial.printf("encoder delta = %ld\n", after - before);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Let the encoder module configure its pins and interrupt handlers.
  initEncoders();

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
  testRun("LEFT fwd ", true,  150);
  testRun("LEFT rev ", true, -150);
  testRun("RIGHT fwd", false, 150);
  testRun("RIGHT rev", false, -150);

  // feel the difference: coast = slow stop, brake = instant stop
  Serial.println("\n--- Coast vs brake (left motor) ---");
  motors.setLeft(200);
  delay(1500);
  motors.stop();  // coast
  delay(1500);
  motors.setLeft(200);
  delay(1500);
  motors.brake();
  delay(1000);

  Serial.println("\nSpin the shafts by hand, watch the counts:");
}

// Just prints encoder counts every 200 ms for the manual test
void loop() {
  static unsigned long last = 0;
  if (millis() - last >= 200) {
    last = millis();
    Serial.printf("encLeft=%ld  encRight=%ld\n",
                  getEncoderLeft(), getEncoderRight());
  }
}