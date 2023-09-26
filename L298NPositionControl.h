#ifndef L298NPOSITIONCONTROL_H
#define L298NPOSITIONCONTROL_H

#include <Arduino.h>
#include <Encoder.h>

class PositionMotor{

  private:

    Encoder *encoder;
    // Pin definitions
    int dirPin1 = 2;
    int dirPin2 = 3;
    int pwmPin = 5;
    int encAPin = 10;
    int encBPin = 11;

    //PID related variables
    double kP = 1;
    double kD = 0;
    double kI = 0;
    long d_time = 0;
    long prevTime = 0;
    int error = 0;
    int prevError = 0;
    int d_error = 0;
    int i_error = 0;


    //Motor and hardware system characteristics
    int cpt = 64;
    double gearRatio = 1;
    int range = 255;                //Most Arduino boards use 0-255 to set their pwm duty cycle (8-bit resolution). Some offer a 12- or 16-bit resolution. This parameter can optionally be changed when calling the loop function.
    long minPosition = -2147483647L;
    long maxPosition = 2147483646L;


    //State variables
    long currentPosition = 0;
    long targetPosition = 0;
    

  public:

  void initializeMotor();

  //Default constructor - uses default pin values
  PositionMotor();

  //Pin constructor - define custom pin values
  PositionMotor(int newdirPin1, int newdirPin2, int newpwmPin, int newencAPin, int newencBPin);

  //PID constructor - define custom pins and PID values
  PositionMotor(int newdirPin1, int newdirPin2, int newpwmPin, int newencAPin, int newencBPin, double newkP, double newkD, double newkI);

  //Full constructor - define custom pins, PID values and system characteristics
  PositionMotor(int newdirPin1, int newdirPin2, int newpwmPin, int newencAPin, int newencBPin, double newkP, double newkD, double newkI, int newcpt, double newratio = 1);

  //Read the current motor position (either from memory or from the encoder)
  int getPosition(bool update = false);

  //Set the new target position in encoder counts
  bool setTargetCounts(int newtarget);

  //Set the new target position in degrees (output shaft angle if using a gearbox)
  void setTargetDegrees(double newtarget);

  //Set the new target position in radians (output shaft angle if using a gearbox)
  void setTargetRadians(double newTarget);

  //Reset the encoder state to 0 (this does not change the motor target position)
  void resetPosition();
  
  //Reset the encoder state to any desired value (this does not change the motor target position)
  void resetPosition(long newPosition);

  //Perform an iteration of the PID control loop. Range 
  int controlLoop(int newrange = 255);

  //Destructor to clear encoder from memory
  ~PositionMotor();

};

#endif
