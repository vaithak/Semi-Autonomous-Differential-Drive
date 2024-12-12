#ifndef VIVE_H
#define VIVE_H

/*
 * Sample Vive code
 */
#include "vive510.h"

#define SIGNALPIN1 34 // pin receiving signal from Vive circuit - TODO Change to correct pin
#define SIGNALPIN2 7 // pin receiving signal from Vive circuit - TODO Change to correct pin

Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);

#define PRINT_VIVE_INTERVAL 2000
uint32_t last_vive_print_time = 0;

// Struct to store the Vive results
struct ViveResults {
    // Also store old values for filtering
    uint16_t oldx1;
    uint16_t oldy1;
    uint16_t oldx2;
    uint16_t oldy2;
    uint16_t x_current;
    uint16_t y_current;

    uint16_t x_filtered;
    uint16_t y_filtered;
    uint16_t status;
};

// Create a single instance of the Vive results for each Vive tracker
ViveResults vive1_results;
ViveResults vive2_results;

// Struct to store the combined Vive results
struct CombinedViveResults {
    // Combined position and orientation

    // Store previous position and orientation for filtering
    float old_position_x;
    float old_position_y;
    float old_orientation_theta;

    // Current position and orientation
    float position_x;
    float position_y;
    float orientation_theta; // theta orientation in degrees from the x-axis
};

// Create a single instance of the combined Vive results
CombinedViveResults combined_vive_results;

void vive_setup() {
    vive1.begin();
    vive2.begin();
    Serial.println("Vive trackers started");

    // Initialize the Vive results struct
    vive1_results.oldx1 = 0;       vive1_results.oldy1 = 0;
    vive1_results.oldx2 = 0;       vive1_results.oldy2 = 0;
    vive1_results.x_current = 0;   vive1_results.y_current = 0;
    vive1_results.x_filtered = 0;  vive1_results.y_filtered = 0;
    vive1_results.status = VIVE_RECEIVING;

    vive2_results.oldx1 = 0;       vive2_results.oldy1 = 0;
    vive2_results.oldx2 = 0;       vive2_results.oldy2 = 0;
    vive2_results.x_current = 0;   vive2_results.y_current = 0;
    vive2_results.x_filtered = 0;  vive2_results.y_filtered = 0;
    vive2_results.status = VIVE_RECEIVING;

    // Initialize the combined Vive results struct
    combined_vive_results.position_x = 0;
    combined_vive_results.position_y = 0;
    combined_vive_results.orientation_theta = 0;
}

uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t middle;
  if ((a <= b) && (a <= c))
    middle = (b <= c) ? b : c;  
  else if ((b <= a) && (b <= c))
    middle = (a <= c) ? a : c;
  else    middle = (a <= b) ? a : b;
  return middle;
}


// Function to update the Vive position based on the given vive object
// and update the vive results struct in place.
void UpdateVivePosition (Vive510& vive_obj, ViveResults& vive_results) {
    if (vive_obj.status() == VIVE_RECEIVING) {
        vive_results.oldx2 = vive_results.oldx1;
        vive_results.oldy2 = vive_results.oldy1;
        vive_results.oldx1 = vive_results.x_current;
        vive_results.oldy1 = vive_results.y_current;
        
        vive_results.x_current = vive_obj.xCoord();
        vive_results.y_current = vive_obj.yCoord();

        vive_results.x_filtered = med3filt(vive_results.x_current, vive_results.oldx1, vive_results.oldx2);
        vive_results.y_filtered = med3filt(vive_results.y_current, vive_results.oldy1, vive_results.oldy2);
        vive_results.status = VIVE_RECEIVING;

        if (vive_results.x_filtered > 8000 || vive_results.y_filtered > 8000 || vive_results.x_filtered < 1000 || vive_results.y_filtered < 1000) {
            vive_results.x_filtered = 0;
            vive_results.y_filtered = 0;
        }
    } else {
        // Sync if not receiving
        vive_obj.sync(5);
    }
}


// Function to update the combined coordinates and orientation based on the two global
// Vive results structs
void UpdateCombinedCoordinatesAndOrientation () {
    if (vive1_results.status == VIVE_RECEIVING && vive2_results.status == VIVE_RECEIVING) {
        float position_x = (vive1_results.x_filtered + vive2_results.x_filtered) / 2.0;
        float position_y = (vive1_results.y_filtered + vive2_results.y_filtered) / 2.0;

        float dx = vive1_results.x_filtered - vive2_results.x_filtered;
        float dy = vive1_results.y_filtered - vive2_results.y_filtered;

        float orientation_theta = atan2(dy, dx) * 180 / PI;

        // Update the combined Vive results struct
        combined_vive_results.old_position_x = combined_vive_results.position_x;
        combined_vive_results.old_position_y = combined_vive_results.position_y;
        combined_vive_results.old_orientation_theta = combined_vive_results.orientation_theta;

        combined_vive_results.position_x = position_x;
        combined_vive_results.position_y = position_y;
        combined_vive_results.orientation_theta = orientation_theta;
    }
}

// Function to update 
void ViveUpdate() {
    UpdateVivePosition(vive1, vive1_results);
    UpdateVivePosition(vive2, vive2_results);
    UpdateCombinedCoordinatesAndOrientation();

    // Print the combined results
    if (millis() - last_vive_print_time > PRINT_VIVE_INTERVAL) {
        Serial.print("Vive 1: ");
        Serial.print(vive1_results.x_filtered);
        Serial.print(", ");
        Serial.print(vive1_results.y_filtered);
        Serial.print(" | Vive 2: ");
        Serial.print(vive2_results.x_filtered);
        Serial.print(", ");
        Serial.print(vive2_results.y_filtered);
        Serial.print(" | Combined: ");
        Serial.print(combined_vive_results.position_x);
        Serial.print(", ");
        Serial.print(combined_vive_results.position_y);
        Serial.print(" | Orientation: ");
        Serial.println(combined_vive_results.orientation_theta);

        last_vive_print_time = millis();
    }
}

#endif