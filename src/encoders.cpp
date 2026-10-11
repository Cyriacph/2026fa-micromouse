#include "encoders.h"
#include "config.h"
#include "wiring.h"

// Volatile variables to hold the encoder counts, updated in ISRs
volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;

// The encoder connected to the motor currently labeled B is the physical
// left wheel. Its channel B signal determines the direction of each step.
void IRAM_ATTR updateEncoderLeft() {
  const int direction = digitalRead(ENC_B_CH_B) ? 1 : -1;
  encoderValueLeft += ENCODER_LEFT_DIRECTION * direction;
}

// The encoder connected to the motor currently labeled A is the physical
// right wheel.
void IRAM_ATTR updateEncoderRight() {
  const int direction = digitalRead(ENC_A_CH_B) ? 1 : -1;
  encoderValueRight += ENCODER_RIGHT_DIRECTION * direction;
}

void initEncoders() {
  // Each encoder has a pulse channel (A) and a direction channel (B).
  pinMode(ENC_A_CH_A, INPUT);
  pinMode(ENC_A_CH_B, INPUT);
  pinMode(ENC_B_CH_A, INPUT);
  pinMode(ENC_B_CH_B, INPUT);

  // Count only rising edges on channel A to keep the interrupt rate manageable.
  attachInterrupt(digitalPinToInterrupt(ENC_B_CH_A), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_CH_A), updateEncoderRight, RISING);
}

void resetEncoders() {
  // Do not allow an interrupt to change a counter halfway through the reset.
  noInterrupts();
  encoderValueLeft = 0;
  encoderValueRight = 0;
  interrupts();
}

long getEncoderLeft() {
  // Copy atomically because a pulse could arrive while the value is read.
  noInterrupts();
  const long value = encoderValueLeft;
  interrupts();
  return value;
}

long getEncoderRight() {
  // Copy atomically because a pulse could arrive while the value is read.
  noInterrupts();
  const long value = encoderValueRight;
  interrupts();
  return value;
}