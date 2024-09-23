#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>

constexpr int STANDARD_BUFFER_SIZE = 100;
constexpr double DEG2RAD (M_PI / 180.0);
constexpr double RAD2DEG (180.0 / M_PI);

enum Directions {
    CENTER = 0,
    LEFT,
    RIGHT,
    HARD_LEFT,
    HARD_RIGHT
};

enum States {

    // initial wall following states
    LOCATE_WALL = 0,
    ROTATE_IN_PLACE,
    TURN_TO_WALL,

    // standard driving states
    GET_TB3_DIRECTION,
    TB3_DRIVE_FORWARD,
    TB3_RIGHT_TURN,
    TB3_RIGHT_TURN_90_DEG,
    TB3_LEFT_TURN,
    TB3_LEFT_TURN_90_DEG,
    TB3_SLOW_FORWARD,

    // switching wall follow states
    DRIVE_TO_NEXT_WALL,

    // end states
    STOP,
    DANCE

};

enum CameraSettingsColors {
  RED_INDEX_ADJUSTMENT = 0,
  GREEN_INDEX_ADJUSTMENT,
  BLUE_INDEX_ADJUSTMENT,  
  AMOUNT_OF_COLOURS
};


// Constants for LIDAR ANGLE DETECTIONS
namespace LidarAngles {

    // number of angles
    constexpr int NUM_ANGLES = 5;

    // angle definitions
    constexpr int CENTER_ANGLE = 0;
    constexpr int LEFT_ANGLE = 30;
    constexpr int RIGHT_ANGLE = 330;
    constexpr int HARD_LEFT_ANGLE = 90;
    constexpr int HARD_RIGHT_ANGLE = 270;
}


namespace Distance
{   
    constexpr double NO_WALL_DIST = 0.57;
    constexpr double CHECK_FORWARD_DIST = 0.4;
    constexpr double CHECK_SIDE_DIST = 0.4;
    constexpr double ESCAPE_RANGE = 1 * DEG2RAD;
    constexpr double ESCAPE_RANGE_90 = 90 * DEG2RAD;
    constexpr double MAX_DISTANCE = 4.0;
    constexpr double CHECK_ANGLE_WRAP = 180.0 * DEG2RAD;

}

namespace MotorControl
{
    constexpr double LINEAR_VELOCITY = 0.3;
    constexpr double ANGULAR_VELOCITY = 0.2;
    constexpr double TIME_FOR_HALF_ROTATION = (M_PI/ANGULAR_VELOCITY);
}

namespace GoalTracking
{
    constexpr double GOAL_DETECT_LOWER_THRESHOLD = 35;
    constexpr double GOAL_FOUND = 55;
}

namespace ColorThresholds {
    constexpr double GREEN_CENTER_THESHOLD = 50;
}

#endif