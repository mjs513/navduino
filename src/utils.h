#pragma once
#include "Arduino.h"
#include "math.h"


#define deg2rad(angle) (angle * M_PI / 180.0)
#define rad2deg(angle) (angle * 180.0 / M_PI)
#define wrapToPi(e) (fmod(e + M_PI, 2 * M_PI) - M_PI)


const bool RADIANS = true;
const bool DEGREES = false;

const bool NED_TO_BODY = true;
const bool BODY_TO_NED = false;