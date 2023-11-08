#include <Arduino.h>

#define RCPinForward A3
#define RCPinTurn A0

// the zero point of the reciever.  
#define ZERO_POINT_SIGNAL 1510 

bool debug_controller = false; 

typedef struct {
  volatile unsigned long startTime;
  volatile unsigned long pulseWidth;
} PWMInput;

typedef struct {
  double forward_command; // -1 to 1 of the signal
  double turn_command;
} JoystickCommand; 

PWMInput inputForward;
PWMInput inputTurn;
JoystickCommand stick_command;

void setup() {
  pinMode(RCPinForward, INPUT);
  pinMode(RCPinTurn, INPUT);

  attachInterrupt(digitalPinToInterrupt(RCPinForward), handleInterruptForward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinTurn), handleInterruptTurn, CHANGE);

  Serial.begin(9600);
}


void loop() {
  noInterrupts();
  unsigned long pulseWidthForward = inputForward.pulseWidth;
  unsigned long pulseWidthTurn = inputTurn.pulseWidth;
  interrupts();
  // convert from raw reciever to -1 to 1, stores in stick_command struct. 
  reciever_to_command(pulseWidthForward,pulseWidthTurn); 

  Serial.print("Forward/Backward signal: ");
  Serial.print(stick_command.forward_command);
  Serial.print(" | Turn signal: ");
  Serial.println(stick_command.turn_command);

  delay(100); // Small delay to prevent flooding the serial output.
}


void handleInterruptForward() {
  unsigned long currentTime = micros();
  if (digitalRead(RCPinForward) == HIGH) {
    inputForward.startTime = currentTime;
  } else {
    inputForward.pulseWidth = currentTime - inputForward.startTime;
  }
}

void handleInterruptTurn() {
  unsigned long currentTime = micros();
  if (digitalRead(RCPinTurn) == HIGH) {
    inputTurn.startTime = currentTime;
  } else {
    inputTurn.pulseWidth = currentTime - inputTurn.startTime;
  }
}

void reciever_to_command(unsigned long pulseWidthForward, unsigned long pulseWidthTurn) {
  stick_command.forward_command = (double)((long)pulseWidthForward - ZERO_POINT_SIGNAL) / 495.0; 
  stick_command.turn_command = ((double)((long)pulseWidthTurn - ZERO_POINT_SIGNAL) / 500.0) + 0.02; 
}

