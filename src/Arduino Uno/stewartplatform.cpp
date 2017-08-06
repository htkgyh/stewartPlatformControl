#include "stewartplatform.h"

StewartPlatform::StewartPlatform(StewartPlatformGeometry& geom)
{
    // Store geometry as private member variables
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            b_[i][j] = geom.base_servo_pos[i][j];
            p_[i][j] = geom.platform_conn_pos[i][j];
        }
        beta_[i] = geom.servo_angle_from_x[i];
    }
    s_ = geom.oper_rod_len;
    a_ = geom.servo_arm_len;

    // Calulate the home position and angles
    // Due to symmetry this can be calculated for one servo only
    const int h0 = sqrt(s_*s_ + a_*a_ - (p_[0][0] - b_[0][0])*(p_[0][0] - b_[0][0]) - (p_[0][1] - b_[0][1])*(p_[0][1] - b_[0][1])) - p_[0][2];
    const int L = 2*a_*a_;
    const int M = 2*a_*( cos(beta_[0])*(p_[0][0] - b_[0][0]) + sin(beta_[0])*(p_[0][1] - b_[0][2]));
    const int N = 2*a_*(h0 + p_[0][2]);
    const int alpha0 = asin(L/sqrt(M*M + N*N)) - atan(M/N);
    for (int i = 0; i < 6; ++i)
    {
        alpha0_[i] = alpha0;
    }
    h0_ = h0;

    // Initialise the effective leg lengths
    calcLegLengths(0,0,h0,0,0,0);
}

void StewartPlatform::rotate3DVector(float vec[3], const float pitch, const float roll, const float yaw)
{
    float ret [3];
    ret[0] = vec[0]*cosf(yaw)*cosf(pitch) - vec[1]*( sinf(yaw)*cosf(roll) + cosf(yaw)*sinf(pitch)*sinf(roll) )
    + vec[2]*( sinf(yaw)*sinf(roll) + cosf(yaw)*sinf(pitch)*cosf(roll) );
    ret[1] = vec[0]*sinf(yaw)*cosf(pitch) + vec[1]*( cosf(yaw)*cosf(roll) + sinf(yaw)*sinf(pitch)*sinf(roll) )
    - vec[2]*( cosf(yaw)*sinf(roll) + sinf(yaw)*sinf(pitch)*cosf(roll) );
    ret[2] = -vec[0]*sinf(yaw) + vec[1]*cosf(pitch)*sinf(roll) + vec[2]*cosf(pitch)*cosf(roll);
    vec[0] = ret[0];
    vec[1] = ret[1];
    vec[2] = ret[2];
}

void StewartPlatform::calcLegLengths(const int dx, const int dy, const int dz,
                                     const float pitch, const float roll, const float yaw)
{
    // Follows the equation: l_i = T + R*p_i - b_i
    int T [3] = {dx, dy, dz};
    // Iterate over each servo
    for (int i = 0; i < 6; ++i)
    {
        float p_rot [3];
        p_rot[0] = p_[i][0];
        p_rot[1] = p_[i][1];
        p_rot[2] = p_[i][2];
        rotate3DVector(p_rot, pitch, roll, yaw);
        // Iterate over each vector component
        for (int j = 0; j < 3; ++j)
        {
            l_[i] = T[j] + p_rot[j] - b_[i][j];
        }
    }

}

void StewartPlatform::calcServoAngles(float alpha[6], const int dx, const int dy, const int dz,
                                      const float pitch, const float roll, const float yaw)
{
    // Update effective leg lengths
    calcLegLengths(dx, dy, dz, pitch, roll, yaw);
    // Calculate individual servo angles 
    for (int i = 0; i < 6; ++i)
    {
        const float L = l_[i]*l_[i] - s_*s_ + a_*a_;
        const float M = 2*a_*(p_[i][2] - b_[i][2]);
        const float N = 2*a_*( cosf(beta_[i])*(p_[i][0]-b_[i][0]) + sin(beta_[i])*(p_[i][1] - b_[i][1]) );
        alpha[i] = asinf(L / sqrt(M*M + N*N)) - atanf(N / M);
    }
}
