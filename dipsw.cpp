#include "dipsw.h"
#include <Arduino.h>

int DS_PINS[8] = {15,14,8,7,6,5,4,3};  // sw 8 is lsb and sw 1 is msb

//======================================================= constructor
Dipsw::Dipsw()
{
    for (int i=0; i < 8; i++) {
        pinMode(DS_PINS[i], INPUT_PULLUP);
    }

}

//====================================================== read
unsigned char Dipsw::read()
{
    unsigned char result = 0x00;
    for (int i=0; i < 8; i++) {
        result |= digitalRead(DS_PINS[i]) << i;
    }

    return ~result;
}
