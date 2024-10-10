#include "MSD700_UWB.h"

MSD700_UWB::MSD700_UWB(HardwareSerial* uwb_serial) : _uwb_serial(uwb_serial) {
    _uwb_serial->begin(115200);

    for(uint16_t i = 0; i <= UWB_FIR_ORDER; i++){
        distance_buffer[i] = 0;
        deviation_buffer[i]  = 0;
    }
}
MSD700_UWB::~MSD700_UWB(){}

void MSD700_UWB::update_uwb_data(){
    if(_uwb_serial->available() > 0){
        this->uwb_lost_timer = millis();
        uint8_t temp = _uwb_serial->read();
        if(temp == 'X'){
            this->distance_buffer[uwb_counter] = _uwb_serial->parseFloat(); // filtered distance
            this->distance = 0.0;
            for(uint16_t i = 0; i <= UWB_FIR_ORDER; i++){
                this->distance += UWB_FIR_COEFF[i] * distance_buffer[((UWB_FIR_ORDER + 1) + this->uwb_counter - i) % (UWB_FIR_ORDER + 1)];
            }
            // distance = distance_buffer[uwb_counter];
            temp = _uwb_serial->read();
            if(temp == 'Y'){
                this->deviation_buffer[uwb_counter] = _uwb_serial->parseFloat();
                this->prev_deviation = this->deviation;
                this->deviation = 0.0;
                for(uint16_t i = 0; i <= UWB_FIR_ORDER; i++){
                    this->deviation += UWB_FIR_COEFF[i] * this->deviation_buffer[((UWB_FIR_ORDER + 1) + this->uwb_counter - i) % (UWB_FIR_ORDER + 1)];
                }
                // deviation = deviation_buffer[uwb_counter];
                this->polar_distance  = sqrt(pow(this->distance,2) + pow(this->deviation,2));
                this->polar_heading   = atan2(this->deviation, this->distance) * 57.295779513;
                this->uwb_counter = (this->uwb_counter + 1) % (UWB_FIR_ORDER + 1);
            }
        }
    }
}

float MSD700_UWB::get_dist(){
    return this->distance;
}
float MSD700_UWB::get_deviation(){
    return this->deviation;
}
float MSD700_UWB::get_rho(){
    return this->polar_distance;
}
float MSD700_UWB::get_theta(){
    return this->polar_heading;
}