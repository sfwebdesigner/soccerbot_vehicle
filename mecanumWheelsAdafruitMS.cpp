#include "mecanumWheels_AdafruitMS.h"

//=======================================================  constructor
MecanumWheels_AdafruitMS::MecanumWheels_AdafruitMS(const int _scaleMotorSpeed, int _minTranslateSpeed, int _minRotateSpeed, double _strafingCorrection) : afms(Adafruit_MotorShield())
{

  // save custom settings
  scaleMotorSpeed = _scaleMotorSpeed;
  strafingCorrection = _strafingCorrection;
  minTranslateSpeed = _minTranslateSpeed;
  minRotateSpeed = _minRotateSpeed;

  // objects for each of the four motors
  // as viewed by someone in the drivers seat
  pFrontLeftMotor = afms.getMotor(4);
  pFrontRightMotor = afms.getMotor(3);
  pBackLeftMotor = afms.getMotor(1);
  pBackRightMotor = afms.getMotor(2);
}

//====================================================== begin
void MecanumWheels_AdafruitMS::begin()
{
  afms.begin(); // default frequent 1.6Khz,  begin(1000) would make it 1Khz
  setSpeed(0, 0, 0);
  testMotorsStep = 0;
}

//======================================================= setSpeed
// speed can be -100 to 100.  A value of 0 means stop
// actual speed is scaled by scaleMotorSpeed
//
// The position of the wheel are relative to driver.
// NS is the forward / backward direction
// and ES is the right / left direction
//
// speedNS and speedEW exspected to range from -100 to 100
// this speed is scaled to match motor shield duty cycle input of 0 to 255
// and futher scale by scaleMotorSpeed config value
//
//
void MecanumWheels_AdafruitMS::setSpeed(int speedNS, int speedEW, int speedRotate)
{
  // stop motors?
  if (!(abs(speedNS) > minTranslateSpeed || abs(speedEW) > minTranslateSpeed || abs(speedRotate) > minRotateSpeed))
  {
    // we need to be stopped
    pFrontLeftMotor->setSpeed(0);
    pBackLeftMotor->setSpeed(0);
    pFrontRightMotor->setSpeed(0);
    pBackRightMotor->setSpeed(0);
    return;
  }

  // if we get here at least one of the the speed values
  // is 1 to 100 inclusive
  //
  double y = speedNS;
  double x = speedEW;
  double rx = speedRotate;
  double frontLeftMotorPower, frontRightMotorPower, backLeftMotorPower, backRightMotorPower;

  // use algebra or trig
  if (true)
  {
    // mixing magic using simple algebra
    // mixing magic
    frontLeftMotorPower = (y + x + rx);
    backLeftMotorPower = (y - x + rx);
    frontRightMotorPower = (y - x - rx);
    backRightMotorPower = (y + x - rx);
  }
  else
  {
    // mixing magic using trigonometry
    // don't be afraid, we can do trig really fast on a teensy
    double r = sqrt(x * x + y * y);
    double robotAngle = atan(y / x) - (3.1415 / 4);
    frontLeftMotorPower = r * cos(robotAngle) + rx;
    frontRightMotorPower = r * sin(robotAngle) - rx;
    backLeftMotorPower = r * sin(robotAngle) + rx;
    backRightMotorPower = r * cos(robotAngle) - rx;
  }

  // determine max magnitude
  double max = 100;
  if (max < fabs(frontLeftMotorPower))
    max = fabs(frontLeftMotorPower);
  if (max < fabs(backLeftMotorPower))
    max = fabs(backLeftMotorPower);
  if (max < fabs(frontRightMotorPower))
    max = fabs(frontRightMotorPower);
  if (max < fabs(backRightMotorPower))
    max = fabs(backRightMotorPower);

  // normalize to 0 to 1
  frontLeftMotorPower /= max;
  backLeftMotorPower /= max;
  frontRightMotorPower /= max;
  backRightMotorPower /= max;

  int frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed;
  int frontLeftDirection, frontRightDirection, backLeftDirection, backRightDirection;

  // determine motor direction (1 or -1)
  frontLeftDirection = 1;
  if (frontLeftMotorPower < 0)
    frontLeftDirection = -1;
  frontRightDirection = 1;
  if (frontRightMotorPower < 0)
    frontRightDirection = -1;
  backLeftDirection = 1;
  if (backLeftMotorPower < 0)
    backLeftDirection = -1;
  backRightDirection = 1;
  if (backRightMotorPower < 0)
    backRightDirection = -1;

  // scale motor power to 0 to 255

  frontLeftSpeed = fabs(frontLeftMotorPower) * 255 * scaleMotorSpeed / 100;
  if (frontLeftSpeed > 255)
    frontLeftSpeed = 255;
  frontRightSpeed = fabs(frontRightMotorPower) * 255 * scaleMotorSpeed / 100;
  if (frontRightSpeed > 255)
    frontRightSpeed = 255;
  backLeftSpeed = fabs(backLeftMotorPower) * 255 * scaleMotorSpeed / 100;
  if (backLeftSpeed > 255)
    backLeftSpeed = 255;
  backRightSpeed = fabs(backRightMotorPower) * 255 * scaleMotorSpeed / 100;
  if (backRightSpeed > 255)
    backRightSpeed = 255;

  // update motors
  pFrontLeftMotor->setSpeed(frontLeftSpeed);
  pFrontLeftMotor->run(frontLeftDirection);
  pBackLeftMotor->setSpeed(backLeftSpeed);
  pBackLeftMotor->run(backLeftDirection);

  pFrontRightMotor->setSpeed(frontRightSpeed);
  pFrontRightMotor->run(frontRightDirection);
  pBackRightMotor->setSpeed(backRightSpeed);
  pBackRightMotor->run(backRightDirection);
}

//=============================================================== testMotors
void MecanumWheels_AdafruitMS::testMotors()
{

  switch (testMotorsStep)
  {

  // front left
  case 0:
    pFrontLeftMotor->setSpeed(50);
    pFrontLeftMotor->run(1);
    break;
  case 1:
    pFrontLeftMotor->setSpeed(0);
    break;
  case 2:
    pFrontLeftMotor->setSpeed(50);
    pFrontLeftMotor->run(-1);
    break;
  case 3:
    pFrontLeftMotor->setSpeed(0);
    break;

  // front right
  case 4:
    pFrontRightMotor->setSpeed(50);
    pFrontRightMotor->run(1);
    break;
  case 5:
    pFrontRightMotor->setSpeed(0);
    break;
  case 6:
    pFrontRightMotor->setSpeed(50);
    pFrontRightMotor->run(-1);
    break;
  case 7:
    pFrontRightMotor->setSpeed(0);
    break;

  // back left
  case 8:
    pBackLeftMotor->setSpeed(50);
    pBackLeftMotor->run(1);
    break;
  case 9:
    pBackLeftMotor->setSpeed(0);
    break;
  case 10:
    pBackLeftMotor->setSpeed(50);
    pBackLeftMotor->run(-1);
    break;
  case 11:
    pBackLeftMotor->setSpeed(0);
    break;

  // back right
  case 12:
    pBackRightMotor->setSpeed(50);
    pBackRightMotor->run(1);
    break;
  case 13:
    pBackRightMotor->setSpeed(0);
    pBackRightMotor->run(1);
    break;
  case 14:
    pBackRightMotor->setSpeed(50);
    pBackRightMotor->run(-1);
    break;
  case 15:
    pBackRightMotor->setSpeed(0);
    break;
  }

  if (++testMotorsStep >= 16)
    testMotorsStep = 0;

  delay(500);

}