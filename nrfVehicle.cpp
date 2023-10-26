#include "nrfVehicle.h"
#include <Arduino.h>

#include <RF24.h>

#define PALevel RF24_PA_LOW
#define XMIT_PIPE 0xF0F0F0F0F0LL
#define RCV_PIPE 0xF0F0F0F0F0LL

//==================================================================== constructor
NrfVehicle::NrfVehicle(int csPin, int csnPin, long spiSpeed) : radio(csPin, csnPin, spiSpeed)
{
}

//==================================================================== vehicle begin
bool NrfVehicle::begin(int _channelNum)
{
    // start up the radio
    if (!radio.begin())
        // return now if failed
        return false;

    // radio is good, so configure its settings

    // select the carrier frequency (channel)
    channelNum = _channelNum;
    radio.setChannel(channelNum);

    // set the transmitter power level
    radio.setPALevel(PALevel); // RF24_PA_MAX is default.

    // set receive pipe and go into receive mode
    radio.openReadingPipe(1, RCV_PIPE); // vehice receive pipe
    radio.startListening();             // put radio in receive Mode

    return true;
}

//==================================================================== read (receive used by vehicle)
bool NrfVehicle::read(NrfPacket &rcv)
{
    uint8_t pipe;
    // is there a payload? get the pipe number that recieved it
    if (radio.available(&pipe))
    {  
        uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
        radio.read(&rcv, bytes);                // fetch payload from FIFO

        // only receive on pipe 1
        if (pipe != 1) 
            return false;              

        return true;
        
        Serial.print(F("Received "));
        Serial.print(bytes); // print the size of the payload
        Serial.print(F(" bytes on pipe "));
        Serial.print(pipe); // print the pipe number
        Serial.print(F(": "));

        Serial.print(" lx: ");
        Serial.print(rcv.lJoyX);
        Serial.print(" ly: ");
        Serial.print(rcv.lJoyY);
        Serial.print(" ltrig: ");
        Serial.print(rcv.lTrig);
        Serial.print(" rx: ");
        Serial.print(rcv.rJoyX);
        Serial.print(" ry: ");
        Serial.print(rcv.rJoyY);
        Serial.print(" rtrig: ");
        Serial.print(rcv.rTrig);
        Serial.print(micros());
        Serial.println();

        return true;
    }
    return false;
}
