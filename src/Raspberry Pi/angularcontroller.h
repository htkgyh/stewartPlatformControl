#ifndef ANGULARCONTROLLER_H
#define ANGULARCONTROLLER_H

#include <thread>
#include <vector>

// Control system to provide angular setpoint for stewart platform controller
class AngularController
{
public:
    AngularController();
    ~AngularController();

    // Read-only access to desiredPlatformStateVector
    void getDesiredStateVector() const;
private:
    // Outputs the resultant vector into desiredPlatformStateVector
    void PIDControlLoop();

    // Store the last known angular position of the platform received from
    // the Arduino
    std::vector<double> platformStateVector;

    // Store the (mutable) output of the controller internally
    std::vector<double> desiredPlatformStateVector;

    // Track the measured position of the ball in the local (platform) frame
    std::vector<double> ballPos;
};

#endif
