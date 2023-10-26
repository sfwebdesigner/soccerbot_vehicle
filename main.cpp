//===============================================================
//
//  soccer bot vehicle using teensy 4.0
//


//===============================================================
//  Configuration settings
//

// deadzone for joysticks
const int MIN_RIGHT_JOYSTICK = 10;      // minimum joystick value for right joystick f/b or  l/r
const int MIN_LEFT_JOYSTICK = 10;       // minimum joystick value for left joystick f/b or l/r

// configure motor stuff
const int SCALE_MOTOR_SPEED = 33;       // scale motor speed by this percentage to match KV spec of specific motor
const double SCRAFING_CORRECTION = 1.1; // helps to perfect strafing motion


//===============================================================

#include <TeensyTimerTool.h>
using namespace TeensyTimerTool;

#include "RingBuf.h"
#include "nrfVehicle.h"
#include "nrfPacket.h"
#include "mecanumWheels_AdafruitMS.h"
#include "gimbal.h"
#include "dipsw.h"

#include <Wire.h>

#define VERSION "Hunter Vehicle 1.04"
// weird, actual baud rate appears to 57,600 or 1/2 of the value below  (teensy thing?)
#define SERIAL_BAUD_RATE 115200

// laser out and sensor in pins
const int pinLaser = 2;
const int pinSens4 = 20;
const int pinSens3 = 21;
const int pinSens2 = 22;
const int pinSens1 = 23;

// brushlees motor pwm lines 
const int yawPwmA = 24;
const int yawPwmB = 25;
const int yawPwmC = 26;
const int rollPwmA = 27;
const int rollPwmB = 28;
const int rollPwmC = 29;

const int oneUsecPin = 33;

// led strip pin
const int pinLedStrip = 16;

//================================================================ gimbal

const int gimbalPitchPin = 0;
const int gimbalRollPin = 1;

int gimbalPitchSignal = GIMBAL_PITCH_0;
Gimbal gimbalPitch(gimbalPitchPin);

//================================================================ Wheels
MecanumWheels_AdafruitMS wheels(SCALE_MOTOR_SPEED, MIN_LEFT_JOYSTICK, MIN_RIGHT_JOYSTICK);

//================================================================ Nrf
NrfVehicle nrf(10, 9, 4000000); // CE pin,  CSN pin, spi speed (4Mhz)

//================================================================ dipswitch
Dipsw dipsw;

//================================================================ 1 usec interrupt (yikes that's fast)


//================================================================ laserXmit
//
IntervalTimer laserXmitTimer;
laserXmit Xmit(pinLaser);
void laserXmitTick() {
  Xmit.tick();
}

//================================================================ laserRcvr
//
laserRcvr Rcvr1(pinSens1, "S1");
//laserRcvr Rcvr2(pinSens2, "S2");
//laserRcvr Rcvr3(pinSens3, "S3");
//laserRcvr Rcvr4(pinSens4, "S4");

void rcvr1ISR() {
  Rcvr1.edgeEvent();
}
void rcvr2ISR() {
  //Rcvr2.edgeEvent();
}
void rcvr3ISR() {
  //Rcvr3.edgeEvent();
}
void rcvr4ISR() {
  //Rcvr4.edgeEvent();
}

int nrf_ok;
int nrf_chan;

//============================================================= setup
//
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(pinLaser, OUTPUT);
  pinMode(pinSens1, INPUT);
  pinMode(pinSens2, INPUT);
  pinMode(pinSens3, INPUT);
  pinMode(pinSens4, INPUT);
  digitalWrite(pinLaser, LOW);

  pinMode(pinLedStrip, OUTPUT);
  digitalWrite(pinLedStrip, HIGH);

  // wheels
  Wire.begin();
  wheels.begin();

  // start NRF link
  // get channel number from dipswitch
  nrf_chan = dipsw.read() & 0x3f;
  nrf_ok = nrf.begin(nrf_chan);

  // init the pitch gimbal stuff;
  gimbalPitch.begin();
  gimbalPitch.set(GIMBAL_PITCH_0);
  gimbalPitchSignal = GIMBAL_PITCH_0;

  // laser xmit need to be ticked continuously
  laserXmitTimer.begin(laserXmitTick, PULSE_PACKET_PERIOD_USECS);

  // laser receivers need interrupts attached to sensor pin and interrupt on change
  attachInterrupt(digitalPinToInterrupt(pinSens1), rcvr1ISR, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(pinSens2), rcvr2ISR, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(pinSens3), rcvr3ISR, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(pinSens4), rcvr4ISR, CHANGE);
  interrupts();
}

long rcvMismatchCount = 0;
long rcvTotalCount = 0;
char format[128];

// timeout objects
MsecTimer otherTimeout(500);
MsecTimer debugTimeout(2000);

MsecTimer nrfTimeout(250);
MsecTimer pollTimeout(10);
MsecTimer slewTimeout(250);

// point to task we want to perform in loop
void (*task)() = &mainTask;

//============================================================= mainTask
void mainTask() {
  if (pollTimeout.timeout()) {
    digitalWrite(pinLedStrip, LOW);
    // =====  receive NRF data or timeout if no NRF data ====
    // read Nrf to get packets from controller
    NrfPacket packet;
    if (nrf.read(packet)) {

      // restart timeout
      nrfTimeout.startTimeout();

      // set wheel speeds
      //              FBspeed,       LRSpeed,      RotateSpeed
      wheels.setSpeed(packet.lJoyY, packet.lJoyX, packet.rJoyX);

      // adjust slew periodically
      if (slewTimeout.timeout()) {

        // do we need to adjust pitch signal
        if (abs(packet.rJoyY) > MIN_RIGHT_JOYSTICK) {
          // yes
          // packet rJoyY is used to determine direction and rate of slew
          if (packet.rJoyY > GIMBAL_FAST_SLEW_START) {
            gimbalPitchSignal -= GIMBAL_FAST_SLEW_STEP;
          }
          else if (packet.rJoyY > GIMBAL_SLOW_SLEW_START) {
            gimbalPitchSignal -= GIMBAL_SLOW_SLEW_STEP;
          }
          if (packet.rJoyY < -GIMBAL_FAST_SLEW_START) {
            gimbalPitchSignal += GIMBAL_FAST_SLEW_STEP;
          }
          else if (packet.rJoyY < -GIMBAL_SLOW_SLEW_START) {
            gimbalPitchSignal += GIMBAL_SLOW_SLEW_STEP;
          }
        }

        // update gimbal pitch signal
        if (gimbalPitchSignal > GIMBAL_PITCH_MAX) gimbalPitchSignal = GIMBAL_PITCH_MAX;
        if (gimbalPitchSignal < -GIMBAL_PITCH_MAX) gimbalPitchSignal = -GIMBAL_PITCH_MAX;
        gimbalPitch.set(gimbalPitchSignal);

      }
    }
    else {
      // check for packet timeout
      if (nrfTimeout.timeout()) {
        // timeout
        // indicate using ledStrip
        digitalWrite(pinLedStrip, HIGH);

        // not receiving packets so stop wheels
        // and rest gimbal
        wheels.setSpeed(0, 0, 0);

        // back to 0 pitch
        gimbalPitchSignal = GIMBAL_PITCH_0;
        gimbalPitch.set(gimbalPitchSignal);
      }
    }
  }

  if (otherTimeout.timeout()) {
    // send and receive laser data
    Xmit.buffer.lockedPush((unsigned char)0x11);
    Xmit.buffer.lockedPush((unsigned char)0x88);
    delay(60);
    Rcvr1.dumpBufferToSerial();
  }

  if (debugTimeout.timeout()) {
    Serial.println(VERSION);

    Serial.print("NRF channel: ");
    Serial.println(nrf_chan);

    Serial.println("--------------------------------------------");
  }
}


//============================================================= motorTest
void motorTest() {
  wheels.testMotors();
}

//============================================================= motorTest
void nrfTest() {
  if (nrf.begin(nrf_chan)) {
    digitalWrite(pinLedStrip, LOW);
  }
  else {
    digitalWrite(pinLedStrip, HIGH);
    delay (200);
    digitalWrite(pinLedStrip, LOW);
    delay (200);
  }

}

//============================================================= loop
void loop()
{

  // perform which task
  int sw = dipsw.read() & 0xC0;
  switch (sw) {
    case 0x00:
    default:
      task = &mainTask;
      break;
    case 0x80:
      task = &motorTest;
      break;
    case 0x40:
      task = &nrfTest;
      break;
  }

  // perform the current task (either mainTask or one of the self test tasks)
  task();
}
