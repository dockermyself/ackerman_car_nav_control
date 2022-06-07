//
// Created by server on 2022-06-01.
//

#include "cmath.h"
#include "math.h"

int float_compare(float a, float b) {
    const float epsilon = 0.00001f;
    if (a - b > epsilon) {
        return 1;
    } else if (a - b < -epsilon) {
        return -1;
    }
    return 0;
}

int float_sign(float a){
    const float epsilon = 0.00001f;
    if(a > epsilon){
        return 1;
    }else if(a < -epsilon){
        return -1;
    }
    return 0;
}

int float_sign_equal(float a,float b){
    if(float_sign(a) == float_sign(b)){
        return 1;
    }
    return 0;
}

int float_abs_compare(float a, float b) {
    return float_compare(fabsf(a), fabsf(b));
}