#include <MSD700_UWB.h>

// Initialize the UWB Module
MSD700_UWB uwb1(&Serial2);

void setup() {
    Serial.begin(115200);
}

float dist, rho, theta;
void loop() {
    // Update data
    uwb1.update_uwb_data();

    // Get and Print all data;
    Serial.print("dist: ");
    Serial.println(uwb1.get_dist());
    Serial.print("rho: ");
    Serial.println(uwb1.get_rho());
    Serial.print("theta: ");
    Serial.println(uwb1.get_theta());
    delay(100);
}
