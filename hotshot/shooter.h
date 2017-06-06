#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="shooter.h" />
///
/// <summary>
///   This module contains the functions to handle the shooter.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifdef MOD_ID
  #undef MOD_ID
#endif
#define MOD_SHOOTER             TGenModId(1)
#define MOD_ID                  MOD_SHOOTER

//
// Constants.
//

#define SHOOTERMODE_IDLE        0
#define SHOOTERMODE_RAMPUP      1
#define SHOOTERMODE_RAMPDOWN    2

#define SHOOTEREVT_SHOOTSTOPPED  0
#define SHOOTEREVT_PICKUPSTOPPED 1

//
// Type definitions.
//
typedef struct
{
  int   motorUpper;
  int   motorLower;
  int   motorFeeder;
  int   motorRoller;
  int   motorElevator;
  int   powerStep;
  long  timeStep;
  int   modeShooter;
  int   powerShooterGoal;
  int   powerShooterCurr;
  int   powerFeeder;
  int   powerRoller;
  int   powerElevator;
  int   event;
  long  timeStopShooter;
  long  timeStopPickup;
  long  timePrev;
} SHOOTER;

//
// Import function prototypes.
//
void
ShooterEvent(
  __in SHOOTER &shooter
  );

/// <summary>
///   This function resets the shooter system.
/// </summary>
///
/// <param name="shooter">
///   Points to the SHOOTER structure to be reset.
/// </param>
///
/// <returns> None. </returns>

void
ShooterReset(
  __out SHOOTER &shooter
  )
{
  TFuncName("ShooterReset");
  TEnter(FUNC);

  shooter.modeShooter = SHOOTERMODE_IDLE;
  shooter.powerShooterGoal = 0;
  shooter.powerShooterCurr = 0;
  shooter.powerFeeder = 0;
  shooter.powerRoller = 0;
  shooter.powerElevator = 0;
  shooter.event = 0;
  shooter.timeStopShooter = 0;
  shooter.timeStopPickup = 0;
  shooter.timePrev = time1[T1];
  motor[shooter.motorUpper] = 0;
  motor[shooter.motorLower] = 0;
  motor[shooter.motorFeeder] = 0;
  motor[shooter.motorRoller] = 0;
  motor[shooter.motorElevator] = 0;

  TExit(FUNC);
  return;
}   //ShooterReset

/// <summary>
///   This function initializes the shooter system.
/// </summary>
///
/// <param name="shooter">
///   Points to the SHOOTER structure to be initialized.
/// </param>
/// <param name="motorUpper">
///   Specifies the upper shooter motor.
/// </param>
/// <param name="motorLower">
///   Specifies the lower shooter motor.
/// </param>
/// <param name="motorFeeder">
///   Specifies the feeder motor that feeds the balls to the shooter.
/// </param>
/// <param name="motorRoller">
///   Specifies the roller motor that picks up balls from the floor.
/// </param>
/// <param name="motorElevator">
///   Specifies the elevator motor that transports the balls up to the shooter.
/// </param>
/// <param name="powerStep">
///   Specifies the shooter power step for ramping.
/// </param>
/// <param name="timeStep">
///   Specifies the shooter time step for ramping.
/// </param>
///
/// <returns> None. </returns>

void
ShooterInit(
  __out SHOOTER &shooter,
  __in int motorUpper,
  __in int motorLower,
  __in int motorFeeder,
  __in int motorRoller,
  __in int motorElevator,
  __in int powerStep,
  __in long timeStep
  )
{
  TFuncName("ShooterInit");
  TEnter(INIT);

  shooter.motorUpper = motorUpper;
  shooter.motorLower = motorLower;
  shooter.motorFeeder = motorFeeder;
  shooter.motorRoller = motorRoller;
  shooter.motorElevator = motorElevator;
  shooter.powerStep = powerStep;
  shooter.timeStep = timeStep;
  ShooterReset(shooter);

  TExit(INIT);
  return;
}   //ShooterInit

/// <summary>
///   This function sets shooter to the given shooter and feeder power values.
/// </summary>
///
/// <param name="shooter">
///   Points to the SHOOTER structure.
/// </param>
/// <param name="powerShooter">
///   Specifies the power of the shooter motors.
/// </param>
/// <param name="powerFeeder">
///   Specifies the power of the feeder motor.
/// </param>
/// <param name="timeDuration">
///   Specifies the how long the shooter should be ON.
/// </param>
///
/// <returns> None. </returns>

void
ShooterSetShootPower(
  __out SHOOTER &shooter,
  __in int powerShooter,
  __in int powerFeeder,
  __in long timeDuration
  )
{
  TFuncName("ShooterSetShooter");
  TEnterMsg(API, ("Shoot=%d,Feed=%d", powerShooter, powerFeeder));

  shooter.powerShooterGoal = BOUND(powerShooter, 0, 100);
  shooter.powerFeeder = BOUND(powerFeeder, 0, 100);
  shooter.modeShooter = (powerShooter > shooter.powerShooterCurr)?
                        SHOOTERMODE_RAMPUP: SHOOTERMODE_RAMPDOWN;
  shooter.timeStopShooter = (timeDuration > 0)? time1[T1] + timeDuration: 0;

  TExit(API);
  return;
}   //ShooterSetShootPower

/// <summary>
///   This function sets motor power values for the ball pickup system.
/// </summary>
///
/// <param name="shooter">
///   Points to the SHOOTER structure.
/// </param>
/// <param name="powerRoller">
///   Specifies the power of the roller motor.
/// </param>
/// <param name="powerElevator">
///   Specifies the power of the elevator motor.
/// </param>
/// <param name="timeDuration">
///   Specifies the how long the pickup should be ON.
/// </param>
///
/// <returns> None. </returns>

void
ShooterSetPickupPower(
  __out SHOOTER &shooter,
  __in int powerRoller,
  __in int powerElevator,
  __in long timeDuration
  )
{
  TFuncName("ShooterSetPickup");
  TEnterMsg(API, ("Roller=%d,Elev=%d", powerRoller, powerElevator));

  shooter.powerRoller = BOUND(powerRoller, -100, 100);
  shooter.powerElevator = BOUND(powerElevator, -100, 100);
  shooter.timeStopPickup = (timeDuration > 0)? time1[T1] + timeDuration: 0;

  TExit(API);
  return;
}   //ShooterSetPickupPower

/// <summary>
///   This function performs the shooter task according to the shooter mode.
/// </summary>
///
/// <param name="shooter">
///   Points to the SHOOTER structure.
/// </param>
///
/// <returns> None. </returns>

void
ShooterTask(
  __inout SHOOTER &shooter
  )
{
  TFuncName("ShooterTask");
  TEnter(HIFREQ);

  long timeCurr = time1[T1];

  if (timeCurr >= shooter.timePrev + shooter.timeStep)
  {
    int powerShooter;

    if ((shooter.timeStopShooter != 0) &&
        (timeCurr >= shooter.timeStopShooter))
    {
      shooter.powerShooterGoal = 0;
      shooter.powerFeeder = 0;
      shooter.modeShooter = SHOOTERMODE_RAMPDOWN;
      shooter.timeStopShooter = 0;
      shooter.event = SHOOTEREVT_SHOOTSTOPPED;
      ShooterEvent(shooter);
    }

    if ((shooter.timeStopPickup != 0) &&
        (timeCurr >= shooter.timeStopPickup))
    {
      shooter.powerRoller = 0;
      shooter.powerElevator = 0;
      shooter.timeStopPickup = 0;
      shooter.event = SHOOTEREVT_PICKUPSTOPPED;
      ShooterEvent(shooter);
    }

    switch (shooter.modeShooter)
    {
      case SHOOTERMODE_RAMPDOWN:
        if (shooter.powerShooterCurr > shooter.powerShooterGoal)
        {
          shooter.powerShooterCurr -= shooter.powerStep;
        }
        break;

      case SHOOTERMODE_RAMPUP:
        if (shooter.powerShooterCurr < shooter.powerShooterGoal)
        {
          shooter.powerShooterCurr += shooter.powerStep;
        }
        break;
    }
    powerShooter = BOUND(shooter.powerShooterCurr, 0, 100);
    motor[shooter.motorUpper] = powerShooter;
    motor[shooter.motorLower] = powerShooter;
    motor[shooter.motorFeeder] = shooter.powerFeeder;
    motor[shooter.motorRoller] = shooter.powerRoller;
    motor[shooter.motorElevator] = shooter.powerElevator;
    shooter.timePrev = timeCurr;
  }

  TExit(HIFREQ);
  return;
}   //ShooterTask
