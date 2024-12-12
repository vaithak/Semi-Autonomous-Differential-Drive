#ifndef REACHLOGIC_H
#define REACHLOGIC_H

#include "wall_follow.h"

#define REACHABLE_DISTANCE_THRESHOLD 25 // 25 mm
#define REACHABLE_ORIENTATION_THRESHOLD 5 // 5 degrees
#define ORIENTATION_ALIGNMENT_STEERING_PERCENT 2 // 2% of the maximum steering angle

// Function to check if the robot has reached the desired point
bool hasReachedPosition(float current_x, float current_y, float desired_x, float desired_y) {
    // Manhattan distance
    float distance = abs(current_x - desired_x) + abs(current_y - desired_y);
    return distance <= REACHABLE_DISTANCE_THRESHOLD;
}

// Function to check if the robot has reached the desired orientation
bool hasReachedOrientation(float current_theta, float desired_theta) {
    // Check if the orientation is within 5 degrees of the desired orientation
    return abs(current_theta - desired_theta) <= REACHABLE_ORIENTATION_THRESHOLD;
}

// Normalize angle to be between -180 and 180 degrees
float normalizeAngle(float angle) {
    while (angle > 180)  angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// Function to reach the desired orientation.
// Return true if the robot is still moving towards the orientation
// Return false if the robot has reached the orientation, and did not move in this iteration
bool reachOrientation(float current_theta, float desired_theta, PIDController orientationPID) {
    // Check if the robot has reached the desired orientation
    if (hasReachedOrientation(current_theta, desired_theta)) {
        return false;
    }

    // Use PID to align the orientation
    float diff_theta = desired_theta - current_theta;
    diff_theta = normalizeAngle(diff_theta);
    int steering_angle = (int)(orientationPID.compute(0, diff_theta));

    // Send the steering command to align the orientation
    if (steering_angle > 0) {
        sendSteeringCommand(steering_angle, "LEFT", 0);
    } else if (steering_angle < 0) {
        sendSteeringCommand(-steering_angle, "RIGHT", 0);
    } else {
        sendSteeringCommand(0, "FORWARD", 0);
    }
}

// Function to move the robot towards the desired point
// Return true if the robot is still moving towards the point
// Return false if the robot has reached the point, and did not move in this iteration
bool moveTowardsPoint(
    float current_x, 
    float current_y,
    float current_theta,
    float desired_x,
    float desired_y,
    PIDController orientationPID
) {
    if (hasReachedPosition(current_x, current_y, desired_x, desired_y)) {
        return false;
    }

    // Try front obstacle avoidance once
    if (frontObstacleAvoidance()) {
        return true; // moved because of obstacle avoidance
    }

    // Compute the angle to the desired point
    float dx = desired_x - current_x;
    float dy = desired_y - current_y;
    float desired_theta = atan2(dy, dx) * 180 / PI;

    // Align the bot towards the desired point.
    float diff_theta = desired_theta - current_theta;
    diff_theta = normalizeAngle(diff_theta);
    if (abs(diff_theta) > 3 * REACHABLE_ORIENTATION_THRESHOLD) {
        // If more than 15 degrees, then first align then move forward
        reachOrientation(current_theta, desired_theta, orientationPID);
        return true; // moved because of aligning orientation
    }

    // If less than 15 degrees, then move forward while aligning
    int speed = NORMAL_SPEED;
    int steering_angle = (int)(orientationPID.compute(0, diff_theta));
    if (steering_angle > 0)
        sendSteeringCommand(steering_angle, "LEFT", speed);
    else if (steering_angle < 0)
        sendSteeringCommand(-steering_angle, "RIGHT", speed);
    else
        sendSteeringCommand(0, "FORWARD", speed);
    return true; // moved because of moving forward
}


// Execute reachLogic,
// Return true if we have reached the destination.
bool reachLogic(
    float current_x,
    float current_y,
    float current_theta,

    float desired_x,
    float desired_y,
    float desired_theta,

    PIDController orientationPID
) {
    if (moveTowardsPoint(
        current_x, current_y, 
        desired_x, desired_y,
        current_theta,
        orientationPID)) {
        return false; // as we moved towards point so not finished
    }

    if (reachOrientation(current_theta, desired_theta, orientationPID)) {
        return false; // as we moved towards orientation so not finished
    }

    return true;
}

bool attackLogic(
    float current_x,
    float current_y,
    float current_theta,

    float desired_x,
    float desired_y,
    float desired_theta,

    PIDController orientationPID
) {
    // If not reached the desired position, move towards it
    if (!hasReachedPosition(current_x, current_y, desired_x, desired_y) ||
        !hasReachedOrientation(current_theta, desired_theta)) {
        // Call the reach logic
        return reachLogic(
            current_x, current_y, current_theta, 
            desired_x, desired_y, desired_theta,
            orientationPID);
    }

    return true;

    // If reached the desired position and orientation, attack the tower
    // TODO: Implement the attack logic here
}

#endif