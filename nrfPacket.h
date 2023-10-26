#ifndef NRF_PACKET_H
#define NRF_PACKET_H
#include <Arduino.h> 
class NrfPacket {
    public:
        int16_t lJoyX; // 2
        int16_t lJoyY; // 4
        int16_t lTrig;
        int16_t rJoyX;
        int16_t rJoyY;
        int16_t rTrig; // 12
        byte pad[20];
};

#endif
