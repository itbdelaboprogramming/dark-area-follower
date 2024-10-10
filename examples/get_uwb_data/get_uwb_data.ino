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
    uwb1.get_data(&dist, &rho, &theta);
    Serial.print("dist: ");
    Serial.println(dist);
    Serial.print("rho: ");
    Serial.println(rho);
    Serial.print("theta: ");
    Serial.println(theta);
    delay(100);
}
