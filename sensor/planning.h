
#ifndef PLANNING_H
#define PLANNING_H

#include "wall_follow.h"
#include "vive.h"

// Create enum for defining the modes of operation
enum Mode {
    WALL_FOLLOW,
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

    public:
        Planner() {
            mode = WALL_FOLLOW;
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
            switch (mode) {
                case WALL_FOLLOW:
                    wallFollowLogic();
                    break;
                case REACH:
                case ATTACK:
                    // First update the Vive trackers
                    ViveUpdate();

                    // TODO: Implement the logic for reaching the desired state

                    
                    // Now, assume that we have reached the desired state
                    // and attack if the mode is ATTACK
                    if (mode == ATTACK) {
                        // Attack the tower
                        // TODO: Implement the attack logic

                    }
            }
        }
};

#endif