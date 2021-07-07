#pragma once

#include "Arduino.h"

#define FLOAT_EPSILON (0.0000001f)
//#define DEBUG
class Helper {

public:
	Helper();
	static float map_f(float x, float in_min, float in_max, float out_min, float out_max);
	static bool float_eq(float f1, float f2);
};
