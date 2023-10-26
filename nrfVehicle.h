#ifndef NRF_VEHICLE_H
#define NRF_VEHICLE_H

#include "nrfPacket.h"
#include <RF24.h>


class NrfVehicle
{
public:
    NrfVehicle(int csPin, int csnPin, long spiSpeed);
    bool begin(int _channelNum);
    bool read(NrfPacket &Rcv);

protected:
    RF24 radio;
    int  channelNum;
};


#endif
