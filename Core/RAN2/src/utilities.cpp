#include "../include/utilities.hpp"

float kit::deg2Rad(float degrees) {
    return degrees/180.f*(float)M_PI;
}

float kit::rad2Deg(float radians) {
    return radians/(float)M_PI*180.f;
}

float kit::seconds2Microseconds(float seconds) {
    return seconds*1000000.f;
}

float kit::microseconds2Seconds(float microseconds) {
    return microseconds/1000000.f;
}



