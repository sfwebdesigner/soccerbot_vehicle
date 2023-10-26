#ifndef MECANUM_WHEELS_ADAFRUIT_MS_H
#define MECANUM_WHEELS_ADAFRUIT_MS_H

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// wheels using adafruit motor shield and Mecanum Wheels
class MecanumWheels_AdafruitMS {
public:
    MecanumWheels_AdafruitMS(int _scaleMotorSpeed, int _minTranslateSpeed, int _minRotateSpeed, double _strafingCorrection = 1.0);
    void begin();
    void setSpeed(int speedFB, int speedLR, int speedRotate);
    void testMotors();

    protected:
    Adafruit_MotorShield afms;
    Adafruit_DCMotor *pFrontLeftMotor;
    Adafruit_DCMotor *pFrontRightMotor;
    Adafruit_DCMotor *pBackLeftMotor;
    Adafruit_DCMotor *pBackRightMotor;
    
    // custom setting
    int scaleMotorSpeed;
    int minTranslateSpeed;
    int minRotateSpeed;
    double strafingCorrection;

    int testMotorsStep;
};

#endif
