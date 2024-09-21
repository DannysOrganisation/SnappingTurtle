#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>

constexpr int STANDARD_BUFFER_SIZE = 10;
constexpr double DEG2RAD (M_PI / 180.0);
constexpr double RAD2DEG (180.0 / M_PI);

enum Directions {
    CENTER = 0,
    LEFT,
    RIGHT,
    HARD_LEFT
};

enum States {
 LOCATE_WALL = 0,
 TURN_TO_WALL,
 GET_TB3_DIRECTION,
 TB3_DRIVE_FORWARD,
 TB3_RIGHT_TURN,
 TB3_RIGHT_TURN_90_DEG,
 TB3_LEFT_TURN,
 TB3_LEFT_TURN_90_DEG,
 TB3_SLOW_FORWARD
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
    constexpr double ESCAPE_RANGE = 2 * DEG2RAD;
    constexpr double ESCAPE_RANGE_90 = 90 * DEG2RAD;

}

namespace MotorControl
{
    constexpr double LINEAR_VELOCITY = 0.3;
    constexpr double ANGULAR_VELOCITY = 0.2;
}

enum CameraSettingsColors {
  RED_INDEX_ADJUSTMENT = 0,
  GREEN_INDEX_ADJUSTMENT,
  BLUE_INDEX_ADJUSTMENT,  
  AMOUNT_OF_COLOURS
};

#endif