#ifndef MSD700_UWB_H
#define MSD700_UWB_H

#include <Arduino.h>
#include <stdint.h>

// UWB transceiver constant macro
#define DISTANCE_THRD_U     1.2     // follower threshold distance in m
#define DISTANCE_THRD_L     0.8     // follower threshold distance in m
#define DEVIATION_THRD_U    0.8     // follower threshold deviation in m to start rotating
#define DEVIATION_THRD_L    0.5     // follower threshold deviation in m to start following
#define DEGREE_THRD         15.0    // follower threshold heading in degree

#define UWB_FIR_ORDER   4

class MSD700_UWB{
private:
    // FIR Filter Coeff for FS = 10 Hz at FC = 1 Hz
    const float UWB_FIR_COEFF[UWB_FIR_ORDER + 1] = {0.0284064700150113, 0.237008213590703, 0.469170632788571, 0.237008213590703, 0.0284064700150113};

    HardwareSerial* _uwb_serial;

    // data processing variables for uwb data filtering
    uint8_t rotation_mode = 0x00;
    uint16_t uwb_counter = 0;
    float distance_buffer[UWB_FIR_ORDER + 1];
    float deviation_buffer[UWB_FIR_ORDER + 1];

    // UWB signal variable
    float distance = 0.0, deviation = 0.0, prev_deviation = 0.0;
    float polar_heading = 0.0, polar_distance = 0.0;

    uint32_t uwb_lost_timer = 0;

public:
    MSD700_UWB(HardwareSerial* uwb_serial);
    ~MSD700_UWB();

    void update_uwb_data();
    float get_dist();
    float get_deviation();
    float get_rho();
    float get_theta();
};

#endif