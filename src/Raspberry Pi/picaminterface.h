#ifndef PICAMINTERFACE_H
#define PICAMINTERFACE_H

#include <vector>

class PiCamInterface
{
public:
    PiCamInterface();
    ~PiCamInterface();

    // Retrieve an image from the PiCam
    void readImage();
    // Locate the position of the ball
    void locateBall();
    // Update the platform state vector from Arduino data
    // Provides data to the AngularController
    // NB On its own thread?
    void getPlatformStateVector();
private:
    // Functions for locateBall()
    void processImage();        // Needs a more descriptive name
    void getBallPos();
    void posToCoords();
    void transformToLocalFrame();
};


#endif
