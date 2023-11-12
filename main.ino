#include <Arduino.h>
#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>

// rx vals
#define RCPinForward A3
#define RCPinTurn A0
#define RCPinWeapon A1
#define ZERO_POINT_SIGNAL 1510 
// motor vals
// signal to arm the motor, must be sent on startup. 
#define ARM_SIGNAL 1500
#define ARM_DURATION 5000 // milliseconds 
#define LEFT_MOTOR_PIN A7
#define RIGHT_MOTOR_PIN A9
#define WEAPON_PIN A8
#define MOTOR_MAX_MS 2000
#define MOTOR_MIN_MS 1000
const int motorPulseRange = MOTOR_MAX_MS - MOTOR_MIN_MS;
// servo handles PWM 
Servo left_esc; 
Servo right_esc; 
Servo weapon_esc; 

MPU6050 mpu(Wire);


const bool debug = true; 
bool weird_reciever_activity = false; 

// struct to hold gamepad raw values in pwm signal units
typedef struct {
  volatile unsigned long startTime;
  volatile unsigned long pulseWidth;
} PWMInput;

// noramalized gamepad units.
typedef struct {
  double forward_command; // -1 to 1 of the signal
  double turn_command;
} JoystickCommand; 

// the normalized drive signal that will be sent to the motors
typedef struct { 
  double left_command;  // -1 to 1 
  double right_command; // -1 to 1 
} DriveSignal; 

PWMInput inputForward;
PWMInput inputTurn;
PWMInput weaponSpeed; 
JoystickCommand stick_command;
DriveSignal drive_signal;

bool imu_ok = true; // true if we are ok to use the imu, else just handle appropriately to preserve driver control function

void setup() {
  pinMode(RCPinForward, INPUT);
  pinMode(RCPinTurn, INPUT);
  stick_command.forward_command = 0; 
  stick_command.turn_command = 0;

  attachInterrupt(digitalPinToInterrupt(RCPinForward), handleInterruptForward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinTurn), handleInterruptTurn, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinWeapon), handleInterruptWeaponSpeed, CHANGE);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  if (status != 0) {
    imu_ok = false; 
  }
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accel

  Serial.println("Done!\n");
  delay(500);

  initialize_esc(); 
  if (debug) {
    Serial.begin(9600);
  }
}


void loop() {
  update_imu_safe();

  noInterrupts();
  unsigned long pulseWidthForward = inputForward.pulseWidth;
  unsigned long pulseWidthTurn = inputTurn.pulseWidth;
  interrupts();
  if (pulseWidthForward < 1000) {
    weird_reciever_activity = true; 
  }
  if (!weird_reciever_activity) {
    reciever_to_command(pulseWidthForward,pulseWidthTurn);
    weapon_esc.writeMicroseconds(weaponSpeed.pulseWidth);
  } else {
    reciever_to_command(1500, 1500); 
    weapon_esc.writeMicroseconds(1000);
  }
  tank_steering_normalized();




  write_motor_signal(drive_signal.left_command, drive_signal.right_command);




  if (debug) {
    if (imu_ok) {
      Serial.print("X : ");
      Serial.print(mpu.getAngleX());
      Serial.print("\tY : ");
      Serial.print(mpu.getAngleY());
      Serial.print("\tZ : ");
      Serial.print(mpu.getAngleZ());
      Serial.print("  ");
    }
    Serial.print("pulse width forward: ");
    Serial.print(pulseWidthForward); 
    Serial.print(" pulse width turn: ");
    Serial.print(pulseWidthTurn);
    Serial.print(" pulse width weapon: ");
    Serial.print(weaponSpeed.pulseWidth);
    Serial.print("\t");
    Serial.print("Forward/Backward signal: ");
    Serial.print(stick_command.forward_command);
    Serial.print(" | Turn signal: ");
    Serial.print(stick_command.turn_command);
    Serial.print("\t weird reciever activity: ");
    Serial.println(weird_reciever_activity); 
    delay(100); // Small delay to prevent flooding the serial output.
  }
  weird_reciever_activity = false;

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

void handleInterruptWeaponSpeed() {
  unsigned long currentTime = micros();
  if (digitalRead(RCPinWeapon) == HIGH) {
    weaponSpeed.startTime = currentTime;
  } else {
    weaponSpeed.pulseWidth = currentTime - weaponSpeed.startTime; 
  }
}

void reciever_to_command(unsigned long pulseWidthForward, unsigned long pulseWidthTurn) {
  stick_command.forward_command = (double)((long)pulseWidthForward - ZERO_POINT_SIGNAL) / 495.0; 
  stick_command.turn_command = ((double)((long)pulseWidthTurn - ZERO_POINT_SIGNAL) / 500.0) + 0.02; 
}

void tank_steering_normalized() {
  // turning is negative so subtracting from the left and adding to the right will make it turn to the left and vice cersa 
  drive_signal.left_command = stick_command.forward_command + stick_command.turn_command; 
  drive_signal.right_command = stick_command.forward_command - stick_command.turn_command; 
}

// initialize the speed controllers and follow setup routine. 
void initialize_esc() {
  left_esc.attach(LEFT_MOTOR_PIN);
  right_esc.attach(RIGHT_MOTOR_PIN); 
  weapon_esc.attach(WEAPON_PIN);
  left_esc.writeMicroseconds(ARM_SIGNAL); 
  right_esc.writeMicroseconds(ARM_SIGNAL);
  delay(ARM_DURATION); 
  weapon_esc.writeMicroseconds(MOTOR_MAX_MS);
  delay(500);
  weapon_esc.write(MOTOR_MIN_MS);
  delay(250);
}

void write_motor_signal(double left_normalized, double right_normalized) {

  int left_pwm = mapNormalizedCommandToMicroseconds(left_normalized);
  int right_pwm = mapNormalizedCommandToMicroseconds(right_normalized);
  
  if (debug) {
    Serial.println("actual motor PWM: ");
    Serial.println(left_pwm);
    Serial.println(right_pwm);
  }
  left_esc.writeMicroseconds(left_pwm);
  right_esc.writeMicroseconds(right_pwm); 
}

// maps a -1 to 1 motor command into the appropriate range for our system. 
int mapNormalizedCommandToMicroseconds(double normalized_command) {

  // if the command is really far out the reciever disconnected thus we rshoudl zero it.

  if (normalized_command > 1.0) normalized_command = 1.0;
  if (normalized_command < -1.0) normalized_command = -1.0;

  // Midpoint for the motor's pulse width
  const double motorMidPoint = (MOTOR_MAX_MS + MOTOR_MIN_MS) / 2.0;

  // Now map the command to the motor's pulse width.
  double pulseWidth = motorMidPoint + (normalized_command * motorPulseRange / 2.0);

  return (int) pulseWidth;
}

double LPF(double value, double previous_value, double a) {
  return (a * previous_value) + (1-a) * value; 
}

void update_imu_safe() {
  if (imu_ok) {
    mpu.update();
  }
}
