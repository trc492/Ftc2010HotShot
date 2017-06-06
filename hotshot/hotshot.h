#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="hotshot.h" />
///
/// <summary>
///   This module contains all the competition tasks code.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#define MAX_LIGHT_SENSORS       3
#define MAX_LNFOLLOW_ACTIONS    27

//#include "..\drivers\LEGOLS-driver.h"
#include "..\lib\common.h"
#include "..\lib\trace.h"
#include "..\lib\button.h"
#include "..\lib\display.h"
#include "..\lib\sensor.h"
#include "..\lib\drive.h"
#include "..\lib\sm.h"
#include "..\lib\lnfollow.h"
#include "shooter.h"

#pragma systemFile

#ifdef MOD_ID
  #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN

//
// Trace info.
//
#define TRACE_MODULES           (MOD_MAIN | MOD_SHOOTER)
#define TRACE_LEVEL             FUNC
#define MSG_LEVEL               INFO

//
// State machine event types.
//
#define EVTTYPE_BUTTON          (EVTTYPE_NONE + 1)
#define EVTTYPE_SENSOR          (EVTTYPE_NONE + 2)
#define EVTTYPE_DRIVE           (EVTTYPE_NONE + 3)
#define EVTTYPE_SHOOTER         (EVTTYPE_NONE + 4)

//
// Game info.
//
#define STARTPOS_BLUE_LEFT      0
#define STARTPOS_BLUE_RIGHT     1
#define STARTPOS_RED_LEFT       2
#define STARTPOS_RED_RIGHT      3

//
// Drive info.
//
#define GEAR_RATIO              (16.0/24.0)     //wheel:motor=16:24
#define WHEEL_CIRCUMFERENCE     12.450          //in inches
#define CLICKS_PER_REVOLUTION   1440.0          //in clicks
#define CLICKS_PER_DISTANCE     (CLICKS_PER_REVOLUTION*GEAR_RATIO/WHEEL_CIRCUMFERENCE)
#define WHEELBASE_DISTANCE      27.08           //in inches
#define CLICKS_PER_DEGREE       (CLICKS_PER_DISTANCE*PI*WHEELBASE_DISTANCE/360)
#define MAX_DRIVE_POWER         100
#define KP                      0.3
#define KI                      0.0
#define KD                      0.0

//
// Shooter info.
//
#define SHOOTER_POWER_STEP      10
#define SHOOTER_TIME_STEP       10
#define SHOOTER_HIGH_POWER      100
#define SHOOTER_LOW_POWER       20
#define FEEDER_POWER            100
#define ROLLER_POWER            100
#define ELEVATOR_POWER          100

//
// Sensor info.
//
#define THRESHOLD_LO_LEFTLIGHT    475
#define THRESHOLD_HI_LEFTLIGHT    565
#define THRESHOLD_LO_CENTERLIGHT  400
#define THRESHOLD_HI_CENTERLIGHT  493
#define THRESHOLD_LO_RIGHTLIGHT   478
#define THRESHOLD_HI_RIGHTLIGHT   592

//
// Macros to initialize line follower action table.
//
#define LFInitInvalid(o,i)      {o.Actions[i].powerDrive = 0; \
                                 o.Actions[i].powerTurn = 0; \
                                 o.Actions[i].powerDriveOverShoot = 0; \
                                 o.Actions[i].powerTurnOverShoot = 0; \
                                 o.Actions[i].desc = "NA";}
#define LFInitRight3(o,i)       {o.Actions[i].powerDrive = SPEED_MEDSLOW; \
                                 o.Actions[i].powerTurn = TURN_LARGE; \
                                 o.Actions[i].powerDriveOverShoot = SPEED_SLOW; \
                                 o.Actions[i].powerTurnOverShoot = TURN_HARD; \
                                 o.Actions[i].desc = "R3";}
#define LFInitRight2(o,i)       {o.Actions[i].powerDrive = SPEED_MEDFAST; \
                                 o.Actions[i].powerTurn = TURN_MEDIUM; \
                                 o.Actions[i].powerDriveOverShoot = SPEED_MEDIUM; \
                                 o.Actions[i].powerTurnOverShoot = TURN_LARGE; \
                                 o.Actions[i].desc = "R2";}
#define LFInitRight1(o,i)       {o.Actions[i].powerDrive = SPEED_MEDFAST; \
                                 o.Actions[i].powerTurn = TURN_SMALL; \
                                 o.Actions[i].powerDriveOverShoot = SPEED_MEDFAST; \
                                 o.Actions[i].powerTurnOverShoot = TURN_SMALL; \
                                 o.Actions[i].desc = "R1";}
#define LFInitCenter(o,i)       {o.Actions[i].powerDrive = SPEED_MEDFAST; \
                                 o.Actions[i].powerTurn = TURN_CENTER; \
                                 o.Actions[i].powerDriveOverShoot = SPEED_MEDFAST; \
                                 o.Actions[i].powerTurnOverShoot = TURN_CENTER; \
                                 o.Actions[i].desc = "C";}
#define LFInitLeft1(o,i)        {o.Actions[i].powerDrive = SPEED_MEDFAST; \
                                 o.Actions[i].powerTurn = -TURN_SMALL; \
                                 o.Actions[i].powerDriveOverShoot = SPEED_MEDFAST; \
                                 o.Actions[i].powerTurnOverShoot = -TURN_SMALL; \
                                 o.Actions[i].desc = "L1";}
#define LFInitLeft2(o,i)        {o.Actions[i].powerDrive = SPEED_MEDFAST; \
                                 o.Actions[i].powerTurn = -TURN_MEDIUM; \
                                 o.Actions[i].powerDriveOverShoot = SPEED_MEDIUM; \
                                 o.Actions[i].powerTurnOverShoot = -TURN_LARGE; \
                                 o.Actions[i].desc = "L2";}
#define LFInitLeft3(o,i)        {o.Actions[i].powerDrive = SPEED_MEDSLOW; \
                                 o.Actions[i].powerTurn = -TURN_LARGE; \
                                 o.Actions[i].powerDriveOverShoot = SPEED_SLOW; \
                                 o.Actions[i].powerTurnOverShoot = -TURN_HARD; \
                                 o.Actions[i].desc = "L3";}

//
// Global data.
//
bool      g_fCalDrive = false;
int       g_StartPos = STARTPOS_BLUE_LEFT;
BUTTON    g_Buttons1;
BUTTON    g_Buttons2;
DRIVE     g_Drive;
SHOOTER   g_Shooter;
SM        g_AutoSM;
LNFOLLOW  g_LnFollow;

/// <summary>
///   This function handles the button notification events.
/// </summary>
///
/// <param name="button">
///   Points to the BUTTON structure that generated the event.
/// </param>
///
/// <returns> None. </returns>

void
ButtonEvent(
  __in BUTTON &button
  )
{
  TFuncName("ButtonEvent");
  TEnterMsg(EVENT, ("Button=%x,On=%d", button.maskButton, (byte)button.fPressed));

  int dirPickup = 1;  //Specifies the direction of the pickup motors.

  if (button.idJoystick == 1)
  {
    switch (button.maskButton)
    {
#ifdef HYBRID_MODE
      case Logitech_Btn1:
        //
        // In hybrid mode, we use Btn1 to toggle between
        // autonomous and teleop modes.
        //
        if (button.fPressed)
        {
          if (IsSMDisabled(g_AutoSM))
          {
            //
            // Starting autonomous mode by starting the autonomous state machine.
            //
            SMStart(g_AutoSM);
          }
          else
          {
            //
            // Stopping autonomous mode by stopping the autonomous state machine.
            //
            SMStop(g_AutoSM);
          }
        }
        break;
#endif

      case Logitech_Btn2:
        if (IsSMDisabled(g_AutoSM) && button.fPressed)
        {
          //
          // We only do calibration in teleop mode.
          // When calibrating the light sensors for the Line Follower,
          // start calibration mode and drive the robot around to sample
          // the brightest white and the darkest black on the floor.
          // Then stop the calibration so that it can compute the low and
          // high thresholds for each light sensor.
          //
          if (LnFollowCalibrating(g_LnFollow))
          {
            //
            // We were in calibration mode, we are done.
            //
            LnFollowCal(g_LnFollow, false);
          }
          else
          {
            //
            // We weren't in calibration mode, start it.
            //
            LnFollowCal(g_LnFollow, true);
          }
        }
        break;

      case Logitech_Btn3:
        if (IsSMDisabled(g_AutoSM) && button.fPressed)
        {
          //
          // We only do calibration in teleop mode.
          // Do a PID distance calibration, drive forward 9 ft.
          //
          if (g_fCalDrive)
          {
            g_fCalDrive = false;
          }
          else
          {
            DrivePIDSetDistance(g_Drive, -96.0, 50);
            g_fCalDrive = true;
          }
        }
        break;

      case Logitech_Btn4:
        if (IsSMDisabled(g_AutoSM) && button.fPressed)
        {
          //
          // We only do calibration in teleop mode.
          // Do a PID angle calibration, turn clockwise 360 degrees.
          //
          if (g_fCalDrive)
          {
            g_fCalDrive = false;
          }
          else
          {
            DrivePIDSetAngle(g_Drive, 360.0, 50);
            g_fCalDrive = true;
          }
        }
        break;

      default:
        break;
    }
  }
  else if (button.idJoystick == 2)
  {
    //
    // Make sure this is from joystick 2.
    //
    switch (button.maskButton)
    {
      case Logitech_RB6:
        if (IsSMDisabled(g_AutoSM))
        {
          //
          // We allow shooter control only in teleop mode.
          //
          if (button.fPressed)
          {
            //
            // While this button is pressed, operate the shooter at high power.
            //
            ShooterSetShootPower(g_Shooter, SHOOTER_HIGH_POWER, FEEDER_POWER, 0);
          }
          else
          {
            ShooterSetShootPower(g_Shooter, 0, 0, 0);
          }
        }
        break;

      case Logitech_RB8:
        if (IsSMDisabled(g_AutoSM))
        {
          //
          // We allow shooter control only in teleop mode.
          //
          if (button.fPressed)
          {
            //
            // While this button is pressed, operate the shooter at low power.
            //
            ShooterSetShootPower(g_Shooter, SHOOTER_LOW_POWER, FEEDER_POWER, 0);
          }
          else
          {
            ShooterSetShootPower(g_Shooter, 0, 0, 0);
          }
        }
        break;

      case Logitech_LB7:
        dirPickup = -1;
        //
        // Let it fall through.
        //
      case Logitech_LB5:
        if (IsSMDisabled(g_AutoSM))
        {
          //
          // We allow shooter control only in teleop mode.
          //
          if (button.fPressed)
          {
            //
            // This button toggles the pickup subsystem ON and OFF in forward direction.
            //
            if ((g_Shooter.powerRoller == 0) &&
                (g_Shooter.powerElevator == 0))
            {
              ShooterSetPickupPower(g_Shooter,
                                    dirPickup*ROLLER_POWER,
                                    dirPickup*ELEVATOR_POWER,
                                    0);
            }
            else
            {
              ShooterSetPickupPower(g_Shooter, 0, 0, 0);
            }
          }
        }
        break;

      default:
        break;
    }
  }

  TExit(EVENT);
  return;
}   //ButtonEvent

/// <summary>
///   This function handles the sensor notification events.
/// </summary>
///
/// <param name="sensor">
///   Points to the SESNOR structure that generated the event.
/// </param>
///
/// <returns> None. </returns>

void
SensorEvent(
  __in SENSOR &sensor
  )
{
  TFuncName("SensorEvent");
  TEnterMsg(EVENT, ("Sensor=%d,Zone=%d", sensor.idSensor, sensor.zoneSensor));

  //
  // Currently unused but we must declare this function since the sensor
  // module will call it.
  //
#if 0
  switch (sensor.idSensor)
  {
    default:
      break;
  }
#endif

  TExit(EVENT);
  return;
}   //SensorEvent

/// <summary>
///   This function handles the drive notification events.
/// </summary>
///
/// <param name="drive">
///   Points to the DRIVE structure that generated the event.
/// </param>
///
/// <returns> None. </returns>

void
DriveEvent(
  __in DRIVE &drive
  )
{
  TFuncName("DriveEvent");
  TEnterMsg(EVENT, ("Mode=%d", drive.modeDrive));

  if (IsSMEnabled(g_AutoSM))
  {
    //
    // In Autonomous mode, forward drive event to the
    // autonomous state machine to unblock waiters waiting
    // for this event.
    //
    SMSetEvent(g_AutoSM, EVTTYPE_DRIVE, drive.modeDrive, 0, 0, 0);
  }
  else
  {
    switch (drive.modeDrive)
    {
      case DRIVEMODE_PID_DISTANCE:
      case DRIVEMODE_PID_ANGLE:
        if (g_fCalDrive)
        {
          g_fCalDrive = false;
        }
        break;
    }
  }

  TExit(EVENT);
  return;
}   //DriveEvent

/// <summary>
///   This function handles the shooter notification events.
/// </summary>
///
/// <param name="drive">
///   Points to the SHOOTER structure that generated the event.
/// </param>
///
/// <returns> None. </returns>

void
ShooterEvent(
  __in SHOOTER &shooter
  )
{
  TFuncName("ShooterEvent");
  TEnterMsg(EVENT, ("Evt=%d", shooter.event));

  if (IsSMEnabled(g_AutoSM))
  {
    //
    // In Autonomous mode, forward shooter event to the
    // autonomous state machine to unblock waiters waiting
    // for this event.
    //
    SMSetEvent(g_AutoSM, EVTTYPE_SHOOTER, shooter.event, 0, 0, 0);
  }

  TExit(EVENT);
  return;
}   //ShooterEvent

/// <summary>
///   This function implements the autonomous mode with a start position
///   on the left of the blue alliance.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure.
/// </param>
///
/// <returns> None. </returns>
///
/// <remarks>
///   Strategy:
/// </remarks>

void
AutoBlueLeft(
  __inout SM &sm
  )
{
  TFuncName("AutoBlueLeft");
  TEnter(HIFREQ);

  switch (sm.currState)
  {
    case SMSTATE_STARTED:
      //
      // Move backward 8 ft.
      //
      DrivePIDSetDistance(g_Drive, -96.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 1:
      //
      // Turn left 90-degree.
      //
      DrivePIDSetAngle(g_Drive, -90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 2:
      //
      // Move backward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, -24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 3:
      //
      // Move forward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, 24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 4:
      //
      // Turn right 90-degree.
      //
      DrivePIDSetAngle(g_Drive, 90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 5:
      //
      // Move forward 6 ft.
      //
      DrivePIDSetDistance(g_Drive, 72.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 6:
      //
      // Turn left 90-degree.
      //
      DrivePIDSetAngle(g_Drive, -90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 7:
      //
      // Move forward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, 24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 8:
      //
      // Shoot high goal for 2-second.
      //
      ShooterSetShootPower(g_Shooter, SHOOTER_HIGH_POWER, FEEDER_POWER, 2000);
      SMAddWaitEvent(sm, EVTTYPE_SHOOTER, SHOOTEREVT_SHOOTSTOPPED, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 9:
      //
      // Turn right 180-degree.
      //
      DrivePIDSetAngle(g_Drive, 180.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 10:
      //
      // Move backward 6 ft.
      //
      DrivePIDSetDistance(g_Drive, -72.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 11:
      //
      // We are done.
      //
      SMStop(sm);
      break;

    default:
      break;
  }

  TExit(HIFREQ);
  return;
}   //AutoBlueLeft

/// <summary>
///   This function implements the autonomous mode with a start position
///   on the right of the blue alliance.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure.
/// </param>
///
/// <returns> None. </returns>

void
AutoBlueRight(
  __inout SM &sm
  )
{
  TFuncName("AutoBlueRight");
  TEnter(HIFREQ);

  switch (sm.currState)
  {
    case SMSTATE_STARTED:
      //
      // Move backward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, -24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 1:
      //
      // Turn right 90-degree.
      //
      DrivePIDSetAngle(g_Drive, 90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 2:
      //
      // Move backward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, -24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 3:
      //
      // Move forward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, 24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 4:
      //
      // Turn right 180-degree.
      //
      DrivePIDSetAngle(g_Drive, 180.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 5:
      //
      // Move backward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, -24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 6:
      //
      // Shoot high goal for 2-second.
      //
      ShooterSetShootPower(g_Shooter, SHOOTER_HIGH_POWER, FEEDER_POWER, 2000);
      SMAddWaitEvent(sm, EVTTYPE_SHOOTER, SHOOTEREVT_SHOOTSTOPPED, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 7:
      //
      // Move backward 4 ft.
      //
      DrivePIDSetDistance(g_Drive, -48.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 8:
      //
      // Turn right 90-degree.
      //
      DrivePIDSetAngle(g_Drive, 90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 9:
      //
      // Move backward 6 ft.
      //
      DrivePIDSetDistance(g_Drive, -72.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 10:
      //
      // Turn left 90-degree.
      //
      DrivePIDSetAngle(g_Drive, -90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 11:
      //
      // Move backward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, -24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 12:
      //
      // We are done.
      //
      SMStop(sm);
      break;

    default:
      break;
  }

  TExit(HIFREQ);
  return;
}   //AutoBlueRight

/// <summary>
///   This function implements the autonomous mode with a start position
///   on the left of the red alliance.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure.
/// </param>
///
/// <returns> None. </returns>
///
/// <remarks>
///   Strategy:
/// </remarks>

void
AutoRedLeft(
  __inout SM &sm
  )
{
  TFuncName("AutoRedLeft");
  TEnter(HIFREQ);

  switch (sm.currState)
  {
    case SMSTATE_STARTED:
      //
      // Move backward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, -24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 1:
      //
      // Turn left 90-degree.
      //
      DrivePIDSetAngle(g_Drive, -90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 2:
      //
      // Move backward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, -24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 3:
      //
      // Move forward 4 ft.
      //
      DrivePIDSetDistance(g_Drive, 48.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 4:
      //
      // Shoot high goal for 2-second.
      //
      ShooterSetShootPower(g_Shooter, SHOOTER_HIGH_POWER, FEEDER_POWER, 2000);
      SMAddWaitEvent(sm, EVTTYPE_SHOOTER, SHOOTEREVT_SHOOTSTOPPED, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 5:
      //
      // Move forward 4 ft.
      //
      DrivePIDSetDistance(g_Drive, 48.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 6:
      //
      // Turn left 90-degree.
      //
      DrivePIDSetAngle(g_Drive, -90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 7:
      //
      // Move forward 6 ft.
      //
      DrivePIDSetDistance(g_Drive, 72.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 8:
      //
      // Turn left 90-degree.
      //
      DrivePIDSetAngle(g_Drive, -90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 9:
      //
      // Move backward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, -24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 10:
      //
      // We are done.
      //
      SMStop(sm);
      break;

    default:
      break;
  }

  TExit(HIFREQ);
  return;
}   //AutoRedLeft

/// <summary>
///   This function implements the autonomous mode with a start position
///   on the right of the red alliance.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure.
/// </param>
///
/// <returns> None. </returns>
///
/// <remarks>
///   Strategy:
/// </remarks>

void
AutoRedRight(
  __inout SM &sm
  )
{
  TFuncName("AutoRedRight");
  TEnter(HIFREQ);

  switch (sm.currState)
  {
    case SMSTATE_STARTED:
      //
      // Move backward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, -24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 1:
      //
      // Turn left 90-degree.
      //
      DrivePIDSetAngle(g_Drive, -90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 2:
      //
      // Move backward 4 ft.
      //
      DrivePIDSetDistance(g_Drive, -48.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 3:
      //
      // Shoot high goal for 2-second.
      //
      ShooterSetShootPower(g_Shooter, SHOOTER_HIGH_POWER, FEEDER_POWER, 2000);
      SMAddWaitEvent(sm, EVTTYPE_SHOOTER, SHOOTEREVT_SHOOTSTOPPED, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 4:
      //
      // Move backward 4 ft.
      //
      DrivePIDSetDistance(g_Drive, -48.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 5:
      //
      // Move forward 8 ft.
      //
      DrivePIDSetDistance(g_Drive, 96.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 6:
      //
      // Turn left 90-degree.
      //
      DrivePIDSetAngle(g_Drive, -90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 7:
      //
      // Move forward 6 ft.
      //
      DrivePIDSetDistance(g_Drive, 72.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 8:
      //
      // Turn left 90-degree.
      //
      DrivePIDSetAngle(g_Drive, -90.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_ANGLE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 9:
      //
      // Move backward 2 ft.
      //
      DrivePIDSetDistance(g_Drive, -24.0, 50);
      SMAddWaitEvent(sm, EVTTYPE_DRIVE, DRIVEMODE_PID_DISTANCE, -1);
      SMWaitEvents(sm, sm.currState + 1, SMF_CLEAR_EVENTS);
      break;

    case SMSTATE_STARTED + 10:
      //
      // We are done.
      //
      SMStop(sm);
      break;

    default:
      break;
  }

  TExit(HIFREQ);
  return;
}   //AutoRedRight

/// <summary>
///   This function implements the state machine for autonomous mode.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure.
/// </param>
///
/// <returns> None. </returns>

void
AutonomousSM(
  __inout SM &sm
  )
{
  TFuncName("AutonomousSM");
  TEnter(HIFREQ);

  if (IsSMReady(sm))
  {
    //
    // We only execute the autonomous state machine if it is not in wait mode.
    //
//    LnFollowTask(g_LnFollow);
    switch (g_StartPos)
    {
      case STARTPOS_BLUE_LEFT:
        AutoBlueLeft(sm);
        break;

      case STARTPOS_BLUE_RIGHT:
        AutoBlueRight(sm);
        break;

      case STARTPOS_RED_LEFT:
        AutoRedLeft(sm);
        break;

      case STARTPOS_RED_RIGHT:
        AutoRedRight(sm);
        break;

      default:
        TErr(("Invalid StartPos"));
        break;
    }
  }

  TExit(HIFREQ);
  return;
}   //AutonomousSM

/// <summary>
///   This function initializes the Line Follower action table.
/// </summary>
///
/// <returns> None. </returns>

void
InitLnFollowActions()
{
  TFuncName("InitLnFollowActions");
  TEnter(INIT);

  LFInitInvalid(g_LnFollow, 0);         //0,0,0
  LFInitRight3(g_LnFollow, 1);          //0,0,1
  LFInitRight2(g_LnFollow, 2);          //0,0,2
  LFInitCenter(g_LnFollow, 3);          //0,1,0
  LFInitRight1(g_LnFollow, 4);          //0,1,1
  LFInitRight2(g_LnFollow, 5);          //0,1,2
  LFInitCenter(g_LnFollow, 6);          //0,2,0
  LFInitCenter(g_LnFollow, 7);          //0,2,1
  LFInitRight1(g_LnFollow, 8);          //0,2,2
  LFInitLeft3(g_LnFollow, 9);           //1,0,0
  LFInitInvalid(g_LnFollow, 10);        //1,0,1
  LFInitRight2(g_LnFollow, 11);         //1,0,2
  LFInitLeft1(g_LnFollow, 12);          //1,1,0
  LFInitCenter(g_LnFollow, 13);         //1,1,1
  LFInitRight1(g_LnFollow, 14);         //1,1,2
  LFInitCenter(g_LnFollow, 15);         //1,2,0
  LFInitCenter(g_LnFollow, 16);         //1,2,1
  LFInitRight1(g_LnFollow, 17);         //1,2,2
  LFInitLeft2(g_LnFollow, 18);          //2,0,0
  LFInitLeft2(g_LnFollow, 19);          //2,0,1
  LFInitInvalid(g_LnFollow, 20);        //2,0,2
  LFInitLeft2(g_LnFollow, 21);          //2,1,0
  LFInitLeft1(g_LnFollow, 22);          //2,1,1
  LFInitInvalid(g_LnFollow, 23);        //2,1,2
  LFInitLeft1(g_LnFollow, 24);          //2,2,0
  LFInitLeft1(g_LnFollow, 25);          //2,2,1
  LFInitCenter(g_LnFollow, 26);         //2,2,2

  TExit(INIT);
  return;
}   //InitLnFollowActions

/// <summary>
///   This function initializes the robot and its subsystems.
/// </summary>
///
/// <returns> None. </returns>

void
RobotInit()
{
  TFuncName("RobotInit");
  TEnter(INIT);

#ifdef HTSMUX_STATUS
  //
  // Initialize the SMUX.
  //
  TInfo(("SMUXInit"));
  HTSMUXinit();
  //
  // Tell the SMUX to scan its ports for connected sensors.
  //
  TInfo(("ScanPort"));
  HTSMUXscanPorts(HTSmux);
  //
  // Initialize the three light sensors.
  //
  TInfo(("SetActive"));
  LSsetActive(msensor_S4_1);
  LSsetActive(msensor_S4_2);
  LSsetActive(msensor_S4_3);
#endif
  //
  // Initialize the Line Follower and light sesnosrs.
  //
  InitLnFollowActions();
  LnFollowInit(g_LnFollow, MAX_LIGHT_SENSORS);
  SensorInit(g_LnFollow.LightSensors[0],
#ifdef HTSMUX_STATUS
             msensor_S4_1,
#else
             lightLeft,
#endif
             THRESHOLD_LO_LEFTLIGHT,
             THRESHOLD_HI_LEFTLIGHT,
             SENSORF_INVERSE
#ifdef HTSMUX_STATUS
             | SENSORF_HTSMUX
#endif
             );
  SensorInit(g_LnFollow.LightSensors[1],
#ifdef HTSMUX_STATUS
             msensor_S4_2,
#else
             lightCenter,
#endif
             THRESHOLD_LO_CENTERLIGHT,
             THRESHOLD_HI_CENTERLIGHT,
             SENSORF_INVERSE
#ifdef HTSMUX_STATUS
             | SENSORF_HTSMUX
#endif
             );
  SensorInit(g_LnFollow.LightSensors[2],
#ifdef HTSMUX_STATUS
             msensor_S4_3,
#else
             lightRight,
#endif
             THRESHOLD_LO_RIGHTLIGHT,
             THRESHOLD_HI_RIGHTLIGHT,
             SENSORF_INVERSE
#ifdef HTSMUX_STATUS
             | SENSORF_HTSMUX
#endif
             );
  //
  // Initialize the button subsystem for joystick 1 & 2.
  //
  ButtonInit(g_Buttons1, 1, BTNF_ENABLE_EVENTS);
  ButtonInit(g_Buttons2, 2, BTNF_ENABLE_EVENTS);
  //
  // Intialize the Drive subsystem of the robot running base.
  //
  DriveInit(g_Drive,
            motorG,     //Left motor
            motorF,     //Right motor
            CLICKS_PER_DISTANCE,
            CLICKS_PER_DEGREE,
            KP, KI, KD,
            DRIVEF_ENABLE_EVENTS);
  //
  // Initialize the Shoot subsystem.
  //
  ShooterInit(g_Shooter,
              motorE,   //Upper shooter motor
              motorD,   //Lower shooter motor
              motorC,   //Feeder motor
              motorA,   //Roller motor
              motorB,   //Elevator motor
              SHOOTER_POWER_STEP,
              SHOOTER_TIME_STEP);
  //
  // Initialize the Autonomous state machine.
  //
  SMInit(g_AutoSM);

  TExit(INIT);
  return;
}   //RobotInit

/// <summary>
///   This function processes all the input tasks.
/// </summary>
///
/// <param name="fDoJoystick">
///   If true, process joystick input.
/// </param>
///
/// <returns> None. </returns>

void
InputTasks(
  __in bool fDoJoystick
  )
{
  TFuncName("InputTasks");
  TEnter(HIFREQ);

  if (fDoJoystick)
  {
    getJoystickSettings(joystick);
    ButtonTask(g_Buttons1);
    ButtonTask(g_Buttons2);
  }

  TExit(HIFREQ);
  return;
}   //InputTasks

/// <summary>
///   This function processes all the main tasks.
/// </summary>
///
/// <returns> None. </returns>

void
MainTasks()
{
  TFuncName("MainTasks");
  TEnter(HIFREQ);

  nxtDisplayTextLine(1, "Left=%d", nMotorEncoder[g_Drive.motorLeft]);
  nxtDisplayTextLine(2, "Right=%d", nMotorEncoder[g_Drive.motorRight]);
  if (IsSMEnabled(g_AutoSM))
  {
    //
    // Autonomous mode.
    //
    nxtDisplayTextLine(0, "Mode=Auto");
    nxtDisplayTextLine(1, "State=%d", g_AutoSM.currState);
    AutonomousSM(g_AutoSM);
  }
  else if (!g_fCalDrive)
  {
    int powerLeft  = NORMALIZE(DEADBAND_INPUT(joystick.joy1_y1),
                               -128, 127, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
    int powerRight = NORMALIZE(DEADBAND_INPUT(joystick.joy1_y2),
                               -128, 127, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
    //
    // TeleOp mode.
    //
    nxtDisplayTextLine(0, "Mode=TeleOp");
//    nxtDisplayTextLine(1, "Left=%d", powerLeft);
//    nxtDisplayTextLine(2, "Right=%d", powerRight);
    DriveTank(g_Drive, powerLeft, powerRight);
  }

  TExit(HIFREQ);
  return;
}   //MainTasks

/// <summary>
///   This function processes all the output tasks. Output tasks
///   are where all the actions are taking place. All other tasks
///   are just changing states of various objects. There is no
///   action taken until the output tasks.
/// </summary>
///
/// <returns> None. </returns>

void
OutputTasks()
{
  TFuncName("OutputTasks");
  TEnter(HIFREQ);

  //
  // The Drive task programs the drive motors and set the robot into
  // action.
  //
  DriveTask(g_Drive);
  //
  // The Shooter task programs the shooter motors according to their
  // states.
  //
  ShooterTask(g_Shooter);

  TExit(HIFREQ);
  return;
}   //OutputTasks
