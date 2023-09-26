#include <Arduino.h>
#include "PIDPositionControl.h"

void PositionMotor::initializeMotor(){
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  analogWrite(pwmPin, 0);

  encoder = new Encoder(encAPin, encBPin);
}

//Default constructor - uses default pin values
PositionMotor::PositionMotor(){
  initializeMotor();
}

//Pin constructor - define custom pin values
PositionMotor::PositionMotor(int newdirPin1, int newdirPin2, int newpwmPin, int newencAPin, int newencBPin){
  dirPin1 = newdirPin1;
  dirPin2 = newdirPin2;
  pwmPin = newpwmPin;
  encAPin = newencAPin;
  encBPin = newencBPin;

  initializeMotor();
}

PositionMotor::PositionMotor(int newdirPin1, int newdirPin2, int newpwmPin, int newencAPin, int newencBPin, double newkP, double newkD, double newkI){
  dirPin1 = newdirPin1;
  dirPin2 = newdirPin2;
  pwmPin = newpwmPin;
  encAPin = newencAPin;
  encBPin = newencBPin;

  kP = newkP;
  kD = newkD;
  kI = newkI;

  initializeMotor();
}

PositionMotor::PositionMotor(int newdirPin1, int newdirPin2, int newpwmPin, int newencAPin, int newencBPin, double newkP, double newkD, double newkI, int newcpt, double newratio = 1){
  dirPin1 = newdirPin1;
  dirPin2 = newdirPin2;
  pwmPin = newpwmPin;
  encAPin = newencAPin;
  encBPin = newencBPin;

  kP = newkP;
  kD = newkD;
  kI = newkI;

  cpt = newcpt;
  gearRatio = newratio;

  initializeMotor();
}

int PositionMotor::getPosition(bool update = false){
  if (update){
    return encoder->read();
  }
  else{
    return currentPosition;
  }
  
}

bool PositionMotor::setTargetCounts(int newtarget){
  if (targetPosition < minPosition){
    return false;
  }
  if (targetPosition > maxPosition){
    return true;
  }
  
  targetPosition = newtarget;
  return true;
}

void PositionMotor::setTargetDegrees(double newtarget){
  setTargetCounts((newtarget / (360/cpt)) * gearRatio);
}

void PositionMotor::setTargetRadians(double newtarget){
  setTargetCounts(((newtarget / (360/cpt)) * gearRatio) * 57.2958);
}

//Reset the encoder state to 0 (this does not change the motor target position)
void PositionMotor::resetPosition(){
  encoder->write(0);
}

//Reset the encoder state to any desired value (this does not change the motor target position)
void PositionMotor::resetPosition(long newPosition){
  encoder->write(newPosition);
}

//Perform an iteration of the PID control loop. Range 
int PositionMotor::controlLoop(int newrange) {
  long currentPosition = encoder->read(); // Use -> to access members of a pointer
  d_time = micros() - prevTime;
  prevTime = micros(); // Corrected variable name

  int prevPosition = currentPosition; // Declare prevPosition and dEdT
  double dEdT = 0;

  error = targetPosition - currentPosition;
  d_error = currentPosition - prevPosition;
  dEdT = static_cast<double>(d_error) / d_time; // Cast d_error to double
  i_error = i_error + error;

  int signal = static_cast<int>((kP * error) + (kD * dEdT) + (kI * i_error)); // Cast result to int

  // Limit signal output to pwm range
  range = newrange;

  if (signal > range) {
    signal = range;
  } else if (signal < (-range)) {
    signal = -range;
  }

  if (signal < 0) {
    digitalWrite(dirPin1, HIGH); // Corrected dir1 to dirPin1
    digitalWrite(dirPin2, LOW);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH); // Corrected dir2 to dirPin2
  }
  analogWrite(pwmPin, signal); // Corrected pwmout to pwmPin

  return signal;
}

PositionMotor::~PositionMotor(){
  delete encoder;
}
