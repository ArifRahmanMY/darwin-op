#include "Marathon_sim.hpp"
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>
#include <DARwInOPVisionManager.hpp>



#include <cassert>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace webots;
using namespace managers;
using namespace std;

static double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];
int xd,yd;
int pixels[160][120];

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
  "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
  "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
  "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
  "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};

Marathon::Marathon():
    Robot()
{
  mTimeStep = getBasicTimeStep();
  
  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  mHeadLED->set(0x00FF00);
  mBackLedRed = getLED("BackLedRed");
  mBackLedGreen = getLED("BackLedGreen");
  mBackLedBlue = getLED("BackLedBlue");
  mCamera = getCamera("Camera");
  mCamera->enable(2*mTimeStep);
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  mGyro = getGyro("Gyro");
  mGyro->enable(mTimeStep);
  mOverlay = getDisplay("Display");
  mOverlay->setColor(0xFFFFFF); 
  mOverlay->setAlpha(0.0);      // make the overlay layer transparent
  mOverlay->fillRectangle(0,0,160,120);
  mOverlay->setAlpha(1.0);      // make the overlay items opaque
  
  for (int i=0; i<NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    mMotors[i]->enablePosition(mTimeStep);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }
  
  mMotionManager = new DARwInOPMotionManager(this);
  mGaitManager = new DARwInOPGaitManager(this, "config.ini");
  mVisionManager = new DARwInOPVisionManager(mCamera->getWidth(), mCamera->getHeight(), 360, 20, 10, 15, 0, 30);
}

Marathon::~Marathon() {
}

void Marathon::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Marathon::wait(int ms) {
  double startTime = getTime();
  double s = (double) ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// Ball detection based on the ball color using the Vision Manager
// - return: indicates if the algorithm found the ball
// - args: return the position of the ball [-1.0, 1.0]

bool Marathon::getBallCenter(double &x, double &y) {
  static int width  = mCamera->getWidth();
  static int height = mCamera->getHeight();
  
  const unsigned char *im = mCamera->getImage();
  bool find = mVisionManager->getBallCenter(x, y, im);
  
  if(!find) {
    x = 0.0;
    y = 0.0;
    return false;
  } else {
    xd = x;
    yd = y;
    x = 2.0 * x / width  - 1.0;
    y = 2.0 * y / height - 1.0;

    // find and save detected regions
    for (int vx=0; vx<width; vx++) {
      for (int vy=0; vy<height; vy++) {
        if(mVisionManager->isDetected(vx,vy))
          pixels[vx][vy] = 1;
        else
          pixels[vx][vy] = 0;
      }
    }

    return true;
  }
  
}

// function containing the main feedback loop
void Marathon::run() {  
  cout << "---------------Demo of DARwIn-OP---------------" << endl;
  cout << "This demo illustrates all the possibilities available for the DARwIn-OP." << endl;
  cout << "This includes motion playback, walking algorithm and image processing." << endl;
	
  // First step to update sensors values
  myStep();
	
  // set eye led to green
  mEyeLED->set(0x00FF00);
  
  // play the hello motion
  mMotionManager->playPage(1); // init position
  mMotionManager->playPage(24); // hello
  mMotionManager->playPage(9); // walkready position
  wait(200);

  // play the motion preparing the robot to walk
  mGaitManager->start();
  mGaitManager->step(mTimeStep);
  
  // main loop
  double px = 0.0;
  double py = 0.0;
  int fup = 0;
  int fdown = 0;
  const double acc_tolerance = 80.0;
  const double acc_step = 20;
  
  while (true) {
    double x, y, neckPosition, headPosition;
    bool ballInFieldOfView = getBallCenter(x, y);
    const double *acc = mAccelerometer->getValues();
    
    // clear overlay screen
    mOverlay->setAlpha(0.0);      // make the overlay layer transparent
    mOverlay->fillRectangle(0,0,160,120);  // fill with transparent box
    mOverlay->setAlpha(1.0);      // make the overlay items opaque
    
    // count how many steps the accelerometer
    // says that the robot is down
    if (acc[1] < 512.0 - acc_tolerance)
      fup++;
    else
      fup = 0;
    
    if (acc[1] > 512.0 + acc_tolerance)
      fdown++;
    else
      fdown = 0;
    
    // the robot face is down
    if (fup > acc_step) {
      mMotionManager->playPage(1); // init position
      mMotionManager->playPage(10); // f_up
      mMotionManager->playPage(9); // walkready position
      fup = 0;
    }
    // the back face is down
    else if (fdown > acc_step) {
      mMotionManager->playPage(1); // init position
      mMotionManager->playPage(11); // b_up
      mMotionManager->playPage(9); // walkready position
      fdown = 0;
    }
    // if the ball is in the field of view,
    // go in the direction of the ball and kick it
    else if (ballInFieldOfView) {
      // set eye led to blue
      mEyeLED->set(0x0000FF);

      // compute the direction of the head
      // the head move at maximum by 0.015 [rad] at each time step
      x  = 0.015*x + px;
      y  = 0.015*y + py;
      px = x;
      py = y;
      neckPosition = clamp(-x, minMotorPositions[18], maxMotorPositions[18]);
      headPosition = clamp(-y, minMotorPositions[19], maxMotorPositions[19]);

      // go forwards and turn according to the head rotation
      if (y < 0.1) // ball far away, go quickly
        mGaitManager->setXAmplitude(1.0);
      else // ball close, go slowly
        mGaitManager->setXAmplitude(0.5);
      mGaitManager->setAAmplitude(neckPosition);
      mGaitManager->step(mTimeStep);
      
      // Move head
      mMotors[18]->setPosition(neckPosition);
      mMotors[19]->setPosition(headPosition);
      
      // if the ball is close enough
      // kick the ball with the right foot
      if (y > 0.35) {
        mGaitManager->stop();
        wait(500);
        // set eye led to green
        mEyeLED->set(0x00FF00);
        if (x<0.0)
          mMotionManager->playPage(13); // left kick
        else
          mMotionManager->playPage(12); // right kick
        mMotionManager->playPage(9); // walkready position
        mGaitManager->start();
        px = 0.0;
        py = 0.0;
      }
      // only highlight the detected region if we are only walking
      else{
        // go through each pixel,
        for(int xx=0; xx<160; xx++){
          for(int yy=0; yy<120; yy++){
            // draw a cross right at the middle of the screen
            if(xx==160/2 || yy==120/2){
              mOverlay->setColor(0x000000);
              mOverlay->drawPixel(xx, yy);
            }
            // draw a box at the ball center
            else if(xx > xd-5 && xx < xd+5 && yy > yd-5 && yy < yd+5){
              mOverlay->setColor(0x0000FF);
              mOverlay->drawPixel(xx, yy);
            }
            // highlight all detected regions
            else if(pixels[xx][yy]==1) {
              mOverlay->setColor(0x00FF00);
              mOverlay->drawPixel(xx, yy);
            }
            
          }
        }
      }
    }
    
    // the ball is not in the field of view,
    // search it by turning round and moving vertically the head 
    else {
      // set eye led to red
      mEyeLED->set(0xFF0000);

      // turn round
      mGaitManager->setXAmplitude(0.0);
      mGaitManager->setAAmplitude(-0.3);
      mGaitManager->step(mTimeStep);
      
      // move the head vertically
      headPosition = clamp(0.7*sin(2.0*getTime()), minMotorPositions[19], maxMotorPositions[19]);
      mMotors[19]->setPosition(headPosition);
    }
    
    // step
    myStep();
  }
}
