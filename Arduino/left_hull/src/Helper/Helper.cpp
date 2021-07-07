#include "Helper.h"

Helper::Helper(){

}

float Helper::map_f(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool Helper::float_eq(float f1, float f2){
  if(fabs(f1-f2) < FLOAT_EPSILON){
    return true;
  } else {
    return false;
  }

}