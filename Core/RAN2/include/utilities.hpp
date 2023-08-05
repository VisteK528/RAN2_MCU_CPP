#ifndef ROBOTARMNUMBER2CPP_UTILITIES_HPP
#define ROBOTARMNUMBER2CPP_UTILITIES_HPP

#include <cmath>

// Degrees <=> Radians
#define deg2Rad(x) (x/180.f*M_PI)
#define rad2Deg(x) (x*180.f/M_PI)

// Seconds <=> Milliseconds
#define seconds2Milliseconds(x) (x*1000.f)
#define milliseconds2Seconds(x) (x/1000.f)

// Seconds <=> Microseconds
#define seconds2Microseconds(x) (x*1000000.f)
#define microseconds2Seconds(x) (x/1000000.f)

// Millimeters <=> Inches
#define millimeters2Inches(x) (x*0.0393700787f)
#define inches2Millimeters(x) (x*25.4f)

#endif //ROBOTARMNUMBER2CPP_UTILITIES_HPP
