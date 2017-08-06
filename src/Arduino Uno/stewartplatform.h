#ifndef STEWARTPLATFORM_H
#define STEWARTPLATFORM_H

#include "math.h"

struct StewartPlatformGeometry
{
    int base_servo_pos [6][3];
    int platform_conn_pos [6][3];
    int oper_rod_len;
    int servo_arm_len;
    int servo_angle_from_x [6];
};

// Note that since this must compile on an Arduino, no external libraries
// should be used
class StewartPlatform
{
// All units in mm (to avoid floating-point arithmetic)
public:
    StewartPlatform(StewartPlatformGeometry& geom);
    ~StewartPlatform();

private:
    void rotate3DVector(float vec[3], const float pitch, const float roll, const float yaw);
    void calcLegLengths(const int dx, const int dy, const int dz,
                        const float pitch, const float roll, const float yaw);
    void calcServoAngles(float alpha[6]);

    // Constants from the geometry of the Stewart Platform
    int b_ [6][3];     // The postions of each servo on the base
    int p_ [6][3];     // The positions of each connection to the platform (in its local frame)
    int s_;            // Length of the operating rod
    int a_;            // Servo arm length
    float beta_ [6];   // Angle of the servo from the x-axis
    int h0_;           // The 'home' z-displacement of the platform
    float alpha0_ [6]; // 'Home' angles for each servo
    // Geometric variables
    int l_ [6];        // The individual effect leg length for each servo

    int platform_pos_vec_ [3];
};

#endif
