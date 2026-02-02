#pragma once
#include <string>

const float FRONT_DIST_OFFSET = 6.276;
const float LEFT_DIST_OFFSET = 4.716;
const float RIGHT_DIST_OFFSET = 5.084;
const float BACK_DIST_OFFSET = 5.014;

const float MM_TO_IN = 0.0393701;

enum WALL {
    NORTH,
    SOUTH,
    EAST,
    WEST,
};

extern float positionFromRaycast(float sensorReading, float sensorOffset, WALL wall);

extern float normalizeAngle(float angle);