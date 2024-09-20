#ifndef ENUM_H
#define ENUM_H


enum Directions {
    CENTER = 0,
    LEFT,
    RIGHT,
    HARD_LEFT,
    HARD_RIGHT
};

enum Views {
    CENTER_ANGLE = 0,
    LEFT_ANGLE = 30,
    RIGHT_ANGLE = 330,
    HARD_LEFT_ANGLE = 90,
    HARD_RIGHT_ANGLE = 270
}


enum States {
    LOCATE_WALL = 0,
    GET_TB3_DIRECTION,
    TB3_DRIVE_FORWARD,
    TB3_RIGHT_TURN,
    TB3_LEFT_TURN,
    TB3_LEFT_90,
    TB3_SLOW_FORWARD
};

#endif