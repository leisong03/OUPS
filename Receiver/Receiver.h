



#ifndef _CRICKET_H
#define _CRICKET_H

#include <stdio.h>
#include <avr/eeprom.h>

#define SYN1 0x0f
#define SYN2 0x0e

#define SYNC_WD_TIMER 200

//#define TEMP_USE 1

// The speed of sound used for distance calculation
#define SPEED_OF_SOUND 342
// The offset due to overhead
//#define TIMER_OFFSET 5218
#define TIMER_OFFSET 40//320
// Compensation that represent the time a bit takes to travel over the radio
#define COMPENSATION_VAL 48
// Multiplier as a given speed of sound (currently 342)
#define DISTANCE_MULT 29.0//30
// The maximum amount of that to wait for ultrasound arrival
// The should be the time to travel the maximum distance + the offset
// Currently this is 45000 us + the offset
#define MAX_ALLOWED_TIMER_VAL ((unsigned int)30000 + (unsigned int)35000 + (unsigned int) TIMER_OFFSET)
// The average beacons time (in ms)
#define AVG_BEACON_INTERVAL_TIME 1000
// This is the delta (((MAX_ALLOW_TIMER_VAL -
// TIMER_OFFSET) / 1000) * MAX_NUMBER_OF_BEACON % 2)
#define DELTA_BEACON_INTERVAL_TIME 664
// Minimum beacon interval time
// This is eval to AVG_BEACON_INTERVAL_TIME - DELTA_BEACON_INTERVAL_TIME / 2
#define MIN_BEACON_INTERVAL_TIME 668
// This is eval to AVG_BEACON_INTERVAL_TIME + DELTA_BEACON_INTERVAL_TIME / 2
#define MAX_BEACON_INTERVAL_TIME 1332
// Maximum ultrasound travel time
#define MAX_US_TRAVEL_TIME ((unsigned int)23000 + (unsigned int)20000 + (unsigned int)10000)
// The maximum number of beacon that can be present in the neighborhood
#define MAX_NUMBER_OF_BEACONS 22
// Desync delay in microseconds.
#define DESYNC_DELAY 500



uint8_t debug_out = 1;

#define UARTOutput(__x,__y,__args...) do { \
    static char __s[] PROGMEM = __y; \
    if (debug_out) \
        printf_P(__s, ## __args);       \
        } while (0)

#endif

#define TARGETMSG 0x0c

typedef struct TargetMsgType 
{
  uint8_t  TargetId;   		// the Id of tag
  uint16_t EmitIdx;         // emission index
}TargetMsgType;

typedef struct CanMsg{
    uint8_t TagID;
    int8_t Temp;
    uint16_t EmitIdx;
    uint16_t distance;
    int16_t td;
}CanMsg;

//int16_t fine[15]={0,0,-39,16,0,0,0,0};
