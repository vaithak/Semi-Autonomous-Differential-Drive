
#ifndef PLANNING_H
#define PLANNING_H

#include "wall_follow.h"
#include "vive.h"
#include "pid.h"
#include "reachlogic.h"

#define PLANNING_PRINT_INTERVAL 1000

// Create enum for defining the modes of operation
enum Mode {
    LEFT_WALL_FOLLOW,
    RIGHT_WALL_FOLLOW,
    REACH,  // Reach a particular point and orientation
    ATTACK  // Reach a particular point and orientation and attack the tower.
};

// Store important coordinates and orientation for the robot
struct RobotState {
    float x;
    float y;
    float theta;
};

// TODO: fill in the coordinates and orientation for the robot
bool teamRed = true; // Change to false for blue team
RobotState center_tower_blue = {0, 0, 0};
RobotState nexus_red = {0, 0, 0};
RobotState nexus_blue = {0, 0, 0};
RobotState ramp_red_side_entry_point = {0, 0, 0}; // till where vive can track - from here wall follow
RobotState ramp_blue_side_entry_point = {0, 0, 0}; // till where vive can track - from here wall follow
RobotState ramp_red_tower = {0, 0, 0}; // from here vive can track - from here wall follow
RobotState ramp_blue_tower = {0, 0, 0}; // from here vive can track - from here wall follow

// Class to handle the planning logic
class Planner {
    private:
        Mode mode;

        // Store the desired coordinates and orientation
        // for reach and attack modes
        RobotState desiredState;

        // Last time the planner printed debug information
        uint32_t last_planning_print_time = 0;

        PIDController orientationPID(
            Kp_steering, Ki_steering, Kd_steering, 
            10, // useless like me
            -MAX_STEERING_ANGLE_PERCENT, MAX_STEERING_ANGLE_PERCENT
        );

    public:
        Planner() {
            mode = LEFT_WALL_FOLLOW;
        }

        void setup() {
            // Initialize the ToF sensors
            Serial.println("Initializing ToF sensors...");
            initToFSensors();

            // Initialize the Vive trackers
            Serial.println("Initializing Vive trackers...");
            vive_setup();
        }

        void setMode(Mode mode) {
            this->mode = mode;
        }

        void setDesiredState(RobotState desiredState) {
            this->desiredState = desiredState;
        }

        void planLogic() {
            bool printDebug = false;
            if (millis() - last_planning_print_time > PLANNING_PRINT_INTERVAL) {
                printDebug = true;
                last_planning_print_time = millis();
            }
            switch (mode) {
                case LEFT_WALL_FOLLOW:
                    if (printDebug) {
                        Serial.println("Left wall follow mode");
                    }
                    wallFollowLogic(true);
                    break;
                case RIGHT_WALL_FOLLOW:
                    if (printDebug) {
                        Serial.println("Right wall follow mode");
                    }
                    wallFollowLogic(false);
                    break;
                case REACH:
                case ATTACK:
                    // First update the Vive trackers
                    ViveUpdate();
                    float currentX = combined_vive_results.position_x;
                    float currentY = combined_vive_results.position_y;
                    float currentTheta = combined_vive_results.orientation_theta;

                    // Convert vive's orientation to match the robot's orientation
                    float offset = 0; // TODO: Add the offset to match the robot's orientation
                    currentTheta = currentTheta + offset;
                    // Normalize the angle to be between -180 and 180
                    currentTheta = normalizeAngle(currentTheta);

                    if (mode == REACH) {
                        if (printDebug) {
                            Serial.println("Reach mode");
                        }
                        // Reach the desired state
                        reachLogic(
                            currentX, currentY, currentTheta,
                            desiredState.x, desiredState.y, desiredState.theta,
                            orientationPID
                        );
                    }
                    else if (mode == ATTACK) {
                        // Attack the tower
                        attackLogic(
                            currentX, currentY, currentTheta,
                            desiredState.x, desiredState.y, desiredState.theta,
                            orientationPID
                        );
                    }
            }
        }
};

#endif