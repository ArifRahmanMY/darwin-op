// File:          Marathon.hpp
// Date:          20th of September 2011
// Description:   Simple Marathon player showing how to use the middleware between webots and
//                the darwin-op framework
// Author:        fabien.rohrer@cyberbotics.com

#ifndef MARATHON_HPP
#define MARATHON_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace managers {
  class DARwInOPMotionManager;
  class DARwInOPGaitManager;
  class DARwInOPVisionManager;
}

namespace webots {
  class Motor;
  class LED;
  class Camera;
  class Accelerometer;
  class Gyro;
  class Speaker;
  class Display;
};

class Marathon : public webots::Robot {
  public:
                                     Marathon();
    virtual                         ~Marathon();
    void                             run();
    
  private:
    int                              mTimeStep;
    
    void                             myStep();
    void                             wait(int ms);
    bool                             getBallCenter(double &x, double &y);
    
    webots::Motor                    *mMotors[NMOTORS];
    webots::LED                      *mEyeLED;
    webots::LED                      *mHeadLED;
    webots::LED                      *mBackLedRed;
    webots::LED                      *mBackLedGreen;
    webots::LED                      *mBackLedBlue;
    webots::Camera                   *mCamera;
    webots::Accelerometer            *mAccelerometer;
    webots::Gyro                     *mGyro;
    webots::Display                  *mOverlay;
    
    managers::DARwInOPMotionManager  *mMotionManager;
    managers::DARwInOPGaitManager    *mGaitManager;
    managers::DARwInOPVisionManager  *mVisionManager;
};

#endif
