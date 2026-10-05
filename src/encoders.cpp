#include "encoders.h"
#include "config.h"
#include "wiring.h"

// Volatile variables to hold the encoder counts, updated in ISRs
volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;

// A rising edge on channel A means the left wheel moved one encoder step
// Channel B tells us which direction that step represents
void IRAM_ATTR updateEncoderLeft() {
  const int direction = digitalRead(ENC_A_CH_B) ? 1 : -1;
  encoderValueLeft += ENCODER_LEFT_DIRECTION * direction;
}

// The right encoder uses the same scheme as the left encoder
void IRAM_ATTR updateEncoderRight() {
  const int direction = digitalRead(ENC_B_CH_B) ? 1 : -1;
  encoderValueRight += ENCODER_RIGHT_DIRECTION * direction;
}

void initEncoders() {
  // Each encoder has a pulse channel (A) and a direction channel (B).
  pinMode(ENC_A_CH_A, INPUT);
  pinMode(ENC_A_CH_B, INPUT);
  pinMode(ENC_B_CH_A, INPUT);
  pinMode(ENC_B_CH_B, INPUT);

  // Count only rising edges on channel A to keep the interrupt rate manageable.
  attachInterrupt(digitalPinToInterrupt(ENC_A_CH_A), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_CH_A), updateEncoderRight, RISING);
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
  noInterrupts();
  const long value = encoderValueRight;
  interrupts();
  return value;
}