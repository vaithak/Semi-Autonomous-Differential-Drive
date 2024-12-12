
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
        // for reach and attack modes as an array of waypoints.
        RobotState waypoints[10];

        // Offset for which waypoint we are executing for now
        int waypointOffset = 0;

        // Len of waypoints
        int lenWaypoints = 0;

        // Last time the planner printed debug information
        uint32_t last_planning_print_time = 0;

        PIDController orientationPID;

    public:
        // Constructor - initialize mode and orientation PID
        Planner() : mode(LEFT_WALL_FOLLOW), 
                    orientationPID(
                        Kp_steering, Ki_steering, Kd_steering, 
                        10, // useless like me
                        -MAX_STEERING_ANGLE_PERCENT, MAX_STEERING_ANGLE_PERCENT
                    ) {}

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

        void setWaypointsAndMode(int target_x, int target_y, const char* mode) {
            // Set waypointOffset to 0
            this->waypointOffset = 0;

            // Set the mode
            if (strcmp(mode, "leftWallFollow") == 0) {
                this->mode = LEFT_WALL_FOLLOW;
            } else if (strcmp(mode, "rightWallFollow") == 0) {
                this->mode = RIGHT_WALL_FOLLOW;
            } else if (strcmp(mode, "attackRampBlueTower") == 0) {
                // TODO: fix
                this->waypoints[0] = ramp_blue_tower;
                this->mode = ATTACK;
                this->lenWaypoints = 1;
            } else if (strcmp(mode, "attackRampRedTower") == 0) {
                // TODO: fix
                this->waypoints[0] = ramp_red_tower;
                this->mode = ATTACK;
                this->lenWaypoints = 1;
            } else if (strcmp(mode, "attackGroundNexusRight") == 0) {
                // TODO: fix
                this->waypoints[0] = nexus_red;
                this->mode = ATTACK;
                this->lenWaypoints = 1;
            } else if (strcmp(mode, "attackGroundNexusLeft") == 0) {
                // TODO: fix
                this->waypoints[0] = nexus_blue;
                this->mode = ATTACK;
                this->lenWaypoints = 1;
            } else if (strcmp(mode, "attackBlueGroundNexusCenter") == 0) {
                // TODO: fix
                this->waypoints[0] = nexus_blue;
                this->mode = ATTACK;
                this->lenWaypoints = 1;
            } else if (strcmp(mode, "attackRedGroundNexusCenter") == 0) {
                // TODO: fix
                this->waypoints[0] = nexus_red;
                this->mode = ATTACK;
                this->lenWaypoints = 1;
            } else if (strcmp(mode, "gridMode") == 0) {
                // TODO: fix
                this->waypoints[0] = RobotState(
                    target_x, target_y, 0
                );
            }
        }

        void planLogic() {
            bool printDebug = false;
            if (millis() - last_planning_print_time > PLANNING_PRINT_INTERVAL) {
                printDebug = true;
                last_planning_print_time = millis();
            }
            // First update the Vive trackers
            ViveUpdate();
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
                    // If offset is >= len(waypoints), nothing to do
                    if (waypointOffset >= lenWaypoints) {
                        return;
                    }

                    float currentX = combined_vive_results.position_x;
                    float currentY = combined_vive_results.position_y;
                    float currentTheta = combined_vive_results.orientation_theta;

                    // Convert vive's orientation to match the robot's orientation
                    float offset = 0; // TODO: Add the offset to match the robot's orientation
                    currentTheta = currentTheta + offset;
                    // Normalize the angle to be between -180 and 180
                    currentTheta = normalizeAngle(currentTheta);

                    RobotState desiredState = waypoints[waypointOffset];

                    if (mode == REACH) {
                        if (printDebug) {
                            Serial.println("Reach mode");
                        }
                        // Reach the desired state
                        if (reachLogic(
                            currentX, currentY, currentTheta,
                            desiredState.x, desiredState.y, desiredState.theta,
                            orientationPID
                        )) {
                            // Increment the offset
                            waypointOffset += 1;
                        }
                    }
                    else if (mode == ATTACK) {
                        // Attack the tower
                        if (attackLogic(
                            currentX, currentY, currentTheta,
                            desiredState.x, desiredState.y, desiredState.theta,
                            orientationPID
                        )) {
                            // Increment the offset
                            waypointOffset += 1;
                        }
                    }
            }
        }
};

#endif