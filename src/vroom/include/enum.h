#ifndef ENUM_H
#define ENUM_H


enum Directions {
    CENTER = 0,
    LEFT,
    RIGHT,
    HARD_LEFT
};

enum States {
 LOCATE_WALL = 0,
 GET_TB3_DIRECTION,
 TB3_DRIVE_FORWARD,
 TB3_RIGHT_TURN,
 TB3_RIGHT_90,
 TB3_LEFT_TURN,
 TB3_LEFT_90,
 TB3_SLOW_FORWARD
};



#endif