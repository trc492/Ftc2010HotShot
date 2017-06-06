#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="lnfollow.h" />
///
/// <summary>
///   This module contains the library functions to follow the line.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _LNFOLLOW_H
#define _LNFOLLOW_H

#pragma systemFile

#ifdef MOD_ID
  #undef MOD_ID
#endif
#define MOD_ID                  MOD_LNFOLLOW

//
// Constants.
//
#ifndef MAX_LIGHT_SENSORS
  #define MAX_LIGHT_SENSORS     3
#endif
#ifndef MAX_LNFOLLOW_ACTIONS
  #define MAX_LNFOLLOW_ACTIONS  27    //(3^MAX_LIGHT_SENSORS)
#endif

#define SPEED_STOP              0
#define SPEED_VERYSLOW          10
#define SPEED_SLOW              20
#define SPEED_MEDSLOW           35
#define SPEED_MEDIUM            50
#define SPEED_MEDFAST           65
#define SPEED_FAST              80
#define SPEED_VERYFAST          90
#define SPEED_FULL              100

#define TURN_CENTER             0
#define TURN_SMALL              10
#define TURN_MEDIUM             20
#define TURN_LARGE              30
#define TURN_HARD               40

#define LNF_OVERSHOOT           0x0001
#define LNF_CALIBRATING         0x0100
#define LnFollowCalibrating(l)  (l.flagsLnFollow & LNF_CALIBRATING)

//
// Type definitions.
//
typedef struct
{
  int    powerDrive;
  int    powerTurn;
  int    powerDriveOverShoot;
  int    powerTurnOverShoot;
  string desc;
} ACTION;

typedef struct
{
  ACTION    Actions[MAX_LNFOLLOW_ACTIONS];
  SENSOR    LightSensors[MAX_LIGHT_SENSORS];
  int       numLightSensors;
#if 0
  int       overshootDriveStep;
  int       overshootTurnStep;
  int       overshootMinDrive;
  int       overshootMaxTurn;
  int       cntOverShoot;
#endif
  long      valueWeighted;
  long      valuePrev;
  int       flagsLnFollow;
  int       powerDrive;
  int       powerTurn;
} LNFOLLOW;

/// <summary>
///   This function initializes the line follower object.
/// </summary>
///
/// <param name="lnfollow">
///   Points to the LNFOLLOW structure to be initialized.
/// </param>
/// <param name="numLightSensors">
///   Specifies the number of light sensors used.
/// </param>
///
/// <returns> None. </returns>

void
LnFollowInit(
  __in LNFOLLOW &lnfollow,
  __in int numLightSensors
#if 0
  __in int overshootDriveStep,
  __in int overshootTurnStep,
  __in int overshootMinDrive,
  __in int overshootMaxTurn
#endif
  )
{
  TFuncName("LnFollowInit");
  TEnter(INIT);

  lnfollow.numLightSensors = numLightSensors;
#if 0
  lnfollow.overshootDriveStep = overshootDriveStep;
  lnfollow.overshootTurnStep = overshootTurnStep;
  lnfollow.overshootMinDrive = overshootMinDrive;
  lnfollow.overshootMaxTurn = overshootMaxTurn;
#endif
  lnfollow.valueWeighted = 0;
  lnfollow.valuePrev = 0;
  lnfollow.flagsLnFollow = 0;
  lnfollow.powerDrive = 0;
  lnfollow.powerTurn = 0;

  TExit(INIT);
  return;
}   //LnFollowInit

/// <summary>
///   This function calibrates the light sensors of the line follower.
/// </summary>
///
/// <param name="lnfollow">
///   Points to the LNFOLLOW structure.
/// </param>
/// <param name="fStart">
///   Specifies TRUE to start calibration, FALSE to stop.
/// </param>
///
/// <returns> None. </returns>

void
LnFollowCal(
  __inout LNFOLLOW &lnfollow,
  __in bool fStart
  )
{
  TFuncName("LnFollowCal");
  TEnterMsg(API, ("fStart=%d", (byte)fStart));

  if (fStart)
  {
    lnfollow.flagsLnFollow |= LNF_CALIBRATING;
  }
  else
  {
    lnfollow.flagsLnFollow &= ~LNF_CALIBRATING;
  }

  for (int i = 0; i < lnfollow.numLightSensors; i++)
  {
    SensorCal(lnfollow.LightSensors[i], fStart);
  }

  TExit(API);
  return;
}   //LnFollowCal

/// <summary>
///   This function processes the sensor data for line following.
/// </summary>
///
/// <param name="lnfollow">
///   Points to the LNFOLLOW structure.
/// </param>
///
/// <returns> None. </returns>

void
LnFollowTask(
  __inout LNFOLLOW &lnfollow
  )
{
  TFuncName("LnFollowTask");
  TEnter(HIFREQ);

  lnfollow.valueWeighted = 0;
  for (int i = 0; i < lnfollow.numLightSensors; i++)
  {
    SensorTask(lnfollow.LightSensors[i]);
    lnfollow.valueWeighted = lnfollow.valueWeighted*3 +
                             lnfollow.LightSensors[i].zoneSensor;
  }

  long valueCurr = lnfollow.valueWeighted;
  if (valueCurr != 0)
  {
    //
    // We still see the line.
    //
    lnfollow.flagsLnFollow &= ~LNF_OVERSHOOT;
    if ((lnfollow.Actions[valueCurr].powerDrive == 0) &&
        (lnfollow.Actions[valueCurr].powerTurn == 0))
    {
      // We got an invalid weighted value, just continue what we
      // did last time.
      //
      valueCurr = lnfollow.valuePrev;
    }
  }
  else
  {
    //
    // We don't see the line, assume overshoot. Just continue what we
    // did last time.
    //
    valueCurr = lnfollow.valuePrev;
    lnfollow.flagsLnFollow |= LNF_OVERSHOOT;
  }

  if (lnfollow.flagsLnFollow & LNF_OVERSHOOT)
  {
    lnfollow.powerDrive = lnfollow.Actions[valueCurr].powerDriveOverShoot;
    lnfollow.powerTurn = lnfollow.Actions[valueCurr].powerTurnOverShoot;
  }
  else
  {
    lnfollow.powerDrive = lnfollow.Actions[valueCurr].powerDrive;
    lnfollow.powerTurn = lnfollow.Actions[valueCurr].powerTurn;
  }
#if 0
  TInfo(("err=%d,flags=%d",
         lnfollow.ErrTable[valueCurr], (byte)lnfollow.flagsLnFollow));
  lnfollow.powerDrive = 100 - abs(lnfollow.ErrTable[valueCurr]*lnfollow.overshootDriveStep);
  lnfollow.powerTurn = lnfollow.ErrTable[valueCurr]*lnfollow.overshootTurnStep;
  if (lnfollow.flagsLnFollow & LNF_OVERSHOOT)
  {
    lnfollow.powerDrive -= lnfollow.overshootDriveStep;
    lnfollow.powerTurn += lnfollow.overshootTurnStep;
  }
  lnfollow.powerDrive = BOUND(lnfollow.powerDrive,
                              lnfollow.overshootMinDrive, 100);
  lnfollow.powerTurn = BOUND(lnfollow.powerTurn,
                             -lnfollow.overshootMaxTurn,
                             lnfollow.overshootMaxTurn);
#endif
#if 0
  lnfollow.powerDrive = lnfollow.Actions[valueCurr].powerDrive -
                        lnfollow.cntOverShoot*lnfollow.overshootDriveStep;
  lnfollow.powerDrive = BOUND(lnfollow.powerDrive,
                              lnfollow.overshootMinDrive, 100);
  lnfollow.powerTurn = lnfollow.Actions[valueCurr].powerTurn +
                       lnfollow.cntOverShoot*lnfollow.overshootTurnStep;
  lnfollow.powerTurn = BOUND(lnfollow.powerTurn,
                             -lnfollow.overshootMaxTurn,
                             lnfollow.overshootMaxTurn);
#endif

  lnfollow.valuePrev = valueCurr;

  TExit(HIFREQ);
  return;
}   //LnFollowTask

#endif  //ifndef _LNFOLLOW_H
