#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="grabber.h" />
///
/// <summary>
///   This module contains the functions to handle the grabber.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#pragma systemFile

#ifdef MOD_ID
  #undef MOD_ID
#endif
#define MOD_GRABBER                     TGenModId(1)
#define MOD_ID                          MOD_GRABBER

//
// Constants.
//
#define GRABBERMODE_STOPPED             0
#define GRABBERMODE_OPENING             1
#define GRABBERMODE_CLOSING             2

#define GRABBER_MOVE_THRESHOLD          2

//
// Type definitions.
//
typedef struct
{
  int  motorGrabber;
  int  powerCalibrate;
  int  powerMax;
  long posMin;
  long posMax;
  long timeStep;
  int  modeGrabber;
  bool fCalibrating;
  int  powerGrabber;
  long posGrabber;
  long timePrev;
} GRABBER;

/// <summary>
///   This function resets the grabber system.
/// </summary>
///
/// <param name="grabber">
///   Points to the GRABBER structure to be reset.
/// </param>
///
/// <returns> None. </returns>

void
GrabberReset(
  __out GRABBER &grabber
  )
{
  TFuncName("GrabberReset");
  TEnter(FUNC);

  grabber.modeGrabber = GRABBERMODE_STOPPED;
  grabber.fCalibrating = false;
  grabber.powerGrabber = 0;
  grabber.posGrabber = 0;
  grabber.timePrev = time1[T1];
  motor[grabber.motorGrabber] = 0;
  nMotorEncoder[grabber.motorGrabber] = 0;

  TExit(FUNC);
  return;
}   //GrabberReset

/// <summary>
///   This function starts the grabber calibration.
/// </summary>
///
/// <param name="grabber">
///   Points to the GRABBER structure.
/// </param>
///
/// <returns> None. </returns>

void
GrabberStartCal(
  __inout GRABBER &grabber
  )
{
  TFuncName("GrabberStartCal");
  TEnter(API);

  if (!grabber.fCalibrating)
  {
    grabber.fCalibrating = true;
    grabber.posGrabber = nMotorEncoder[grabber.motorGrabber];
    grabber.powerGrabber = -grabber.powerCalibrate;
    grabber.modeGrabber = GRABBERMODE_CLOSING;
  }

  TExit(API);
  return;
}   //GrabberStartCal

/// <summary>
///   This function initializes the grabber system.
/// </summary>
///
/// <param name="grabber">
///   Points to the GRABBER structure to be initialized.
/// </param>
/// <param name="motorGrabber">
///   Specifies the grabber motor.
/// </param>
/// <param name="powerCalibrate">
///   Specifies the power used for calibration.
/// </param>
/// <param name="powerMax">
///   Specifies the maximum power for the grabber.
/// </param>
/// <param name="posMin">
///   Specifies the minimum position of the grabber when fully closed.
/// </param>
/// <param name="posMax">
///   Specifies the maximum position of the grabber when fully opened.
/// </param>
/// <param name="timeStep">
///   Specifies the time step for the grabber task loop.
/// </param>
///
/// <returns> None. </returns>

void
GrabberInit(
  __out GRABBER &grabber,
  __in int motorGrabber,
  __in int powerCalibrate,
  __in int powerMax,
  __in long posMin,
  __in long posMax,
  __in long timeStep
  )
{
  TFuncName("GrabberInit");
  TEnter(INIT);

  grabber.motorGrabber = motorGrabber;
  grabber.powerCalibrate = powerCalibrate;
  grabber.powerMax = powerMax;
  grabber.posMin = posMin;
  grabber.posMax = posMax;
  grabber.timeStep = timeStep;
  GrabberReset(grabber);
//  GrabberStartCal(grabber);

  TExit(INIT);
  return;
}   //GrabberInit

/// <summary>
///   This function sets the grabber power to start the action.
/// </summary>
///
/// <param name="grabber">
///   Points to the GRABBER structure.
/// </param>
/// <param name="powerGrabber">
///   Specifies the grabber motor power within the specified range.
///   The power range is used to normalized a power value from joystick
///   range to the specified powerMax range.
/// </param>
///
/// <returns> None. </returns>

void
GrabberSetPower(
  __out GRABBER &grabber,
  __in int powerGrabber
  )
{
  TFuncName("GrabberSetPower");
  TEnterMsg(API, ("Power=%d", powerGrabber));

  if (!grabber.fCalibrating)
  {
    //
    // Don't do anything if we are still calibrating.
    //
    grabber.powerGrabber = NORMALIZE(powerGrabber,
                                     -100, 100,
                                     -grabber.powerMax,
                                     grabber.powerMax);
    grabber.modeGrabber = (powerGrabber == 0)? GRABBERMODE_STOPPED:
                          (powerGrabber > 0)? GRABBERMODE_OPENING:
                                              GRABBERMODE_CLOSING;
  }

  TExit(API);
  return;
}   //GrabberSetPower

/// <summary>
///   This function performs the grabber task according to the grabber mode.
/// </summary>
///
/// <param name="grabber">
///   Points to the GRABBER structure.
/// </param>
///
/// <returns> None. </returns>

void
GrabberTask(
  __inout GRABBER &grabber
  )
{
  TFuncName("GrabberTask");
  TEnter(HIFREQ);

  static bool fMoving = false;
  long timeCurr = time1[T1];

  if (timeCurr >= grabber.timePrev + grabber.timeStep)
  {
    int power = grabber.powerGrabber;
    long posCurr = nMotorEncoder[grabber.motorGrabber];
    bool fStuck = fMoving &&
                  (abs(posCurr - grabber.posGrabber) <
                   GRABBER_MOVE_THRESHOLD);

    TVerbose(("Mode=%d", grabber.modeGrabber));
    switch (grabber.modeGrabber)
    {
      case GRABBERMODE_OPENING:
        if (fStuck ||
            (!grabber.fCalibrating && (posCurr >= grabber.posMax)))
        {
          fMoving = false;
          grabber.powerGrabber = 0;
          motor[grabber.motorGrabber] = 0;
          grabber.modeGrabber = GRABBERMODE_STOPPED;
        }
        else
        {
          motor[grabber.motorGrabber] = power;
          fMoving = grabber.powerGrabber != 0;
        }
        break;

      case GRABBERMODE_CLOSING:
        if (fStuck ||
            (!grabber.fCalibrating && (posCurr <= grabber.posMin)))
        {
          fMoving = false;
          grabber.powerGrabber = 0;
          motor[grabber.motorGrabber] = 0;
          grabber.modeGrabber = GRABBERMODE_STOPPED;
          if (grabber.fCalibrating)
          {
            nMotorEncoder[grabber.motorGrabber] = 0;
            grabber.fCalibrating = false;
          }
        }
        else
        {
          motor[grabber.motorGrabber] = power;
          fMoving = grabber.powerGrabber != 0;
        }
        break;
    }

    grabber.posGrabber = posCurr;
    grabber.timePrev = timeCurr;
  }

  TExit(HIFREQ);
  return;
}   //GrabberTask
