#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="drive.h" />
///
/// <summary>
///   This module contains the library functions for the drive subsystem.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _DRIVE_H
#define _DRIVE_H

#pragma systemFile

#ifdef MOD_ID
  #undef MOD_ID
#endif
#define MOD_ID                  MOD_DRIVE

//
// Constants.
//
#define DRIVEMODE_STOPPED       0
#define DRIVEMODE_DRIVE         1
#define DRIVEMODE_PID_DISTANCE  2
#define DRIVEMODE_PID_ANGLE     3

#define DRIVEF_USER_MASK        0x00ff
#define DRIVEF_ENABLE_EVENTS    0x0001

//
// Macros.
//
#define IsRunningState(s)       ((s != runStateIdle) && (s != runStateHoldPosition))
#define IsRunning(d)            (IsRunningState(nMotorRunState[d.motorLeft]) || \
                                 IsRunningState(nMotorRunState[d.motorRight]))
#define NORMALIZE_POWER(n,m)    NORMALIZE(n, -100, 100, -(m), (m))

//
// Type definitions.
//
typedef struct
{
  int   motorLeft;
  int   motorRight;
  float clicksPerDistance;
  float clicksPerDegree;
  float Kp;
  float Ki;
  float Kd;
  int   flagsDrive;
  int   modeDrive;
  int   powerLeft;
  int   powerRight;
  int   clickTargetLeft;
  int   clickTargetRight;
  int   errLeftPrev;
  int   errRightPrev;
  int   errLeftIntegral;
  int   errRightIntegral;
} DRIVE;

//
// Import function prototypes.
//
void
DriveEvent(
  __in DRIVE &drive
  );

/// <summary>
///   This function stops the motors in the drive system.
/// </summary>
///
/// <param name="drive">
///   Points to the DRIVE structure.
/// </param>
///
/// <returns> None. </returns>

void
DriveStop(
  __out DRIVE &drive
  )
{
  TFuncName("DriveStop");
  TEnter(API);

  drive.modeDrive = DRIVEMODE_STOPPED;
  drive.powerLeft = 0;
  drive.powerRight = 0;
  //
  // Stop the motors.
  //
  motor[drive.motorLeft] = 0;
  motor[drive.motorRight] = 0;

  TExit(API);
  return;
}   //DriveStop

/// <summary>
///   This function resets the drive system.
/// </summary>
///
/// <param name="drive">
///   Points to the DRIVE structure to be reset.
/// </param>
///
/// <returns> None. </returns>

void
DriveReset(
  __out DRIVE &drive
  )
{
  TFuncName("DriveReset");
  TEnter(API);

  DriveStop(drive);
  //
  // Reset the encoders.
  //
  nMotorEncoder[drive.motorLeft] = 0;
  nMotorEncoder[drive.motorRight] = 0;
  drive.clickTargetLeft = 0;
  drive.clickTargetRight = 0;
  drive.errLeftPrev = 0;
  drive.errRightPrev = 0;
  drive.errLeftIntegral = 0;
  drive.errRightIntegral = 0;

  TExit(API);
  return;
}   //DriveReset

/// <summary>
///   This function initializes the drive system.
/// </summary>
///
/// <param name="drive">
///   Points to the DRIVE structure to be initialized.
/// </param>
/// <param name="motorLeft">
///   Specifies the left motor.
/// </param>
/// <param name="motorRight">
///   Specifies the right motor.
/// </param>
/// <param name="clicksPerDistance">
///   Specifies the number of encoder clicks per distance travelled of the
///   robot base.
/// </param>
/// <param name="clicksPerDegree">
///   Specifies the number of encoder clicks per degree turn of the robot base.
/// </param>
/// <param name="Kp">
///   Specifies the proportional constant for PID drive.
/// </param>
/// <param name="Ki">
///   Specifies the integral constant for PID drive.
/// </param>
/// <param name="Kd">
///   Specifies the differential constant for PID drive.
/// </param>
/// <param name="flagsDrive">
///   Specifies the drive flags.
/// </param>
///
/// <returns> None. </returns>

void
DriveInit(
  __out DRIVE &drive,
  __in int motorLeft,
  __in int motorRight,
  __in float clicksPerDistance,
  __in float clicksPerDegree,
  __in float Kp,
  __in float Ki,
  __in float Kd,
  __in int flagsDrive
  )
{
  TFuncName("DriveInit");
  TEnter(INIT);

  drive.motorLeft = motorLeft;
  drive.motorRight = motorRight;
  drive.clicksPerDistance = clicksPerDistance;
  drive.clicksPerDegree = clicksPerDegree;
  drive.Kp = Kp;
  drive.Ki = Ki;
  drive.Kd = Kd;
  drive.flagsDrive = flagsDrive & DRIVEF_USER_MASK;
  DriveReset(drive);

  TExit(INIT);
  return;
}   //DriveInit

/// <summary>
///   This function sets power of the motors for tank drive.
/// </summary>
///
/// <param name="drive">
///   Points to the DRIVE structure.
/// </param>
/// <param name="powerLeft">
///   Specifies the left motor power.
/// </param>
/// <param name="powerRight">
///   Specifies the right motor power.
/// </param>
///
/// <returns> None. </returns>

void
DriveTank(
  __out DRIVE &drive,
  __in int powerLeft,
  __in int powerRight
  )
{
  TFuncName("DriveTank");
  TEnterMsg(HIFREQ, ("Left=%d,Right=%d", powerLeft, powerRight));

  drive.powerLeft = BOUND(powerLeft, -100, 100);
  drive.powerRight = BOUND(powerRight, -100, 100);
  drive.modeDrive = DRIVEMODE_DRIVE;

  TExit(HIFREQ);
  return;
}   //DriveTank

/// <summary>
///   This function sets power of the motors for arcade drive.
/// </summary>
///
/// <param name="drive">
///   Points to the DRIVE structure.
/// </param>
/// <param name="powerDrive">
///   Specifies the drive power.
/// </param>
/// <param name="powerTurn">
///   Specifies the turn power.
/// </param>
///
/// <returns> None. </returns>

void
DriveArcade(
  __out DRIVE &drive,
  __in int powerDrive,
  __in int powerTurn
  )
{
  TFuncName("DriveArcade");
  TEnterMsg(HIFREQ, ("Drive=%d,Turn=%d", powerDrive, powerTurn));

  powerDrive = BOUND(powerDrive, -100, 100);
  powerTurn = BOUND(powerTurn, -100, 100);
  if (powerDrive + powerTurn > 100)
  {
    //
    // Forward right:
    //  left = drive + turn - (drive + turn - 100)
    //  right = drive - turn - (drive + turn - 100)
    //
    drive.powerLeft = 100;
    drive.powerRight = -2*powerTurn + 100;
  }
  else if (powerDrive - powerTurn > 100)
  {
    //
    // Forward left:
    //  left = drive + turn - (drive - turn - 100)
    //  right = drive - turn - (drive - turn - 100)
    //
    drive.powerLeft = 2*powerTurn + 100;
    drive.powerRight = 100;
  }
  else if (powerDrive + powerTurn < -100)
  {
    //
    // Backward left:
    //  left = drive + turn - (drive + turn + 100)
    //  right = drive - turn - (drive + turn + 100)
    //
    drive.powerLeft = -100;
    drive.powerRight = -2*powerTurn - 100;
  }
  else if (powerDrive - powerTurn < -100)
  {
    //
    // Backward right:
    //  left = drive + turn - (drive - turn + 100)
    //  right = drive - turn - (drive - turn + 100)
    //
    drive.powerLeft = 2*powerTurn -100;
    drive.powerRight = -100;
  }
  else
  {
    drive.powerLeft = powerDrive + powerTurn;
    drive.powerRight = powerDrive - powerTurn;
  }
  drive.modeDrive = DRIVEMODE_DRIVE;

  TExit(HIFREQ);
  return;
}   //DriveArcade

/// <summary>
///   This function sets PID_DISTANCE drive mode with the given drive distance
///   set point.
/// </summary>
///
/// <param name="drive">
///   Points to the DRIVE structure.
/// </param>
/// <param name="setptDistance">
///   Specifies the target distance to travel.
/// </param>
/// <param name="powerDrive">
///   Specifies the drive power.
/// </param>
///
/// <returns> None. </returns>

void
DrivePIDSetDistance(
  __out DRIVE &drive,
  __in float setptDistance,
  __in int powerDrive
  )
{
  TFuncName("DrivePIDSetDistance");
  TEnterMsg(API, ("D=%5.1f,Pwr=%d", setptDistance, powerDrive));

  int clicksTarget = (int)(setptDistance*drive.clicksPerDistance);

  powerDrive = BOUND(abs(powerDrive), 0, 100);
  drive.powerLeft = powerDrive;
  drive.powerRight = powerDrive;
  drive.clickTargetLeft = nMotorEncoder[drive.motorLeft] + clicksTarget;
  drive.clickTargetRight = nMotorEncoder[drive.motorRight] + clicksTarget;
  drive.errLeftPrev = clicksTarget;
  drive.errRightPrev = clicksTarget;
  drive.errLeftIntegral = 0;
  drive.errRightIntegral = 0;
  drive.modeDrive = DRIVEMODE_PID_DISTANCE;

  TExit(API);
  return;
}   //DrivePIDSetDistance

/// <summary>
///   This function sets PID_ANGLE drive mode with the given drive angle
///   set point.
/// </summary>
///
/// <param name="drive">
///   Points to the DRIVE structure.
/// </param>
/// <param name="setptAngle">
///   Specifies the target angle to turn.
/// </param>
/// <param name="powerTurn">
///   Specifies the turn power.
/// </param>
///
/// <returns> None. </returns>

void
DrivePIDSetAngle(
  __out DRIVE &drive,
  __in float setptAngle,
  __in int powerTurn
  )
{
  TFuncName("DrivePIDSetAngle");
  TEnterMsg(API, ("A=%5.1f,Pwr=%d", setptAngle, powerTurn));

  int clicksTarget = (int)(setptAngle*drive.clicksPerDegree);

  powerTurn = BOUND(abs(powerTurn), 0, 100);
  drive.powerLeft = powerTurn;
  drive.powerRight = powerTurn;
  drive.clickTargetLeft = nMotorEncoder[drive.motorLeft] + clicksTarget;
  drive.clickTargetRight = nMotorEncoder[drive.motorRight] - clicksTarget;
  drive.errLeftPrev = clicksTarget;
  drive.errRightPrev = -clicksTarget;
  drive.errLeftIntegral = 0;
  drive.errRightIntegral = 0;
  drive.modeDrive = DRIVEMODE_PID_ANGLE;

  TExit(API);
  return;
}   //DrivePIDSetAngle

/// <summary>
///   This function performs the driving task according to the drive state.
/// </summary>
///
/// <param name="drive">
///   Points to the DRIVE structure.
/// </param>
///
/// <returns> None. </returns>

void
DriveTask(
  __inout DRIVE &drive
  )
{
  TFuncName("DriveTask");
  TEnter(HIFREQ);

  int errLeft, errRight, errDiff;
  int powerLeft, powerRight;

  switch (drive.modeDrive)
  {
    case DRIVEMODE_DRIVE:
      motor[drive.motorLeft] = drive.powerLeft;
      motor[drive.motorRight] = drive.powerRight;
      break;

    case DRIVEMODE_PID_DISTANCE:
    case DRIVEMODE_PID_ANGLE:
      errLeft = drive.clickTargetLeft - nMotorEncoder[drive.motorLeft];
      errRight = drive.clickTargetRight - nMotorEncoder[drive.motorRight];
      //
      // If we are going straight, we should try making errLeft and errRight the same.
      // Therefore, errDiff should be zero. If not, we will apply errDiff as a differential
      // to the motor powers. But we don't want to have a wide swing, so we bound it to
      // a small differential range.
      //
      errDiff = (drive.modeDrive == DRIVEMODE_PID_DISTANCE)?
                BOUND((errLeft - errRight)/2, -10, 10): 0;
      //
      // Calculate PID control power.
      //
      drive.errLeftIntegral += errLeft;
      drive.errRightIntegral += errRight;
      powerLeft = BOUND(drive.Kp*errLeft +
                        drive.Ki*drive.errLeftIntegral +
                        drive.Kd*(errLeft - drive.errLeftPrev),
                        -drive.powerLeft, drive.powerLeft);
      powerRight = BOUND(drive.Kp*errRight +
                         drive.Ki*drive.errRightIntegral +
                         drive.Kd*(errRight - drive.errRightPrev),
                         -drive.powerRight, drive.powerRight);
      drive.errLeftPrev = errLeft;
      drive.errRightPrev = errRight;

      if (errDiff != 0)
      {
        //
        // Make sure the power is still within bound after applying
        // power differential. If out of bound, bring it back within bound.
        //
        if (powerLeft + errDiff > drive.powerLeft)
        {
          powerLeft = drive.powerLeft;
          powerRight = (powerRight - errDiff) -
                       (powerLeft + errDiff - drive.powerLeft);
        }
        else if (powerLeft + errDiff < -drive.powerLeft)
        {
          powerLeft = -drive.powerLeft;
          powerRight = (powerRight - errDiff) -
                       (powerLeft + errDiff + drive.powerLeft);
        }
        else if (powerRight - errDiff > drive.powerRight)
        {
          powerLeft = (powerLeft + errDiff) -
                      (powerRight - errDiff - drive.powerRight);
          powerRight = drive.powerRight;
        }
        else if (powerRight - errDiff < -drive.powerRight)
        {
          powerLeft = (powerLeft + errDiff) -
                      (powerRight - errDiff + drive.powerRight);
          powerRight = -drive.powerRight;
        }
      }

      if ((abs(powerLeft) > 1) && (abs(powerRight) > 1))
      {
        motor[drive.motorLeft] = powerLeft;
        motor[drive.motorRight] = powerRight;
      }
      else
      {
        motor[drive.motorLeft] = 0;
        motor[drive.motorRight] = 0;
        if (drive.flagsDrive & DRIVEF_ENABLE_EVENTS)
        {
          DriveEvent(drive);
        }
        //
        // Must not change modeDrive until after DriveEvent.
        //
        drive.modeDrive = DRIVEMODE_STOPPED;
      }
      break;
  }

  TExit(HIFREQ);
  return;
}   //DriveTask

#endif  //ifndef _DRIVE_H
