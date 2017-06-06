#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="sensor.h" />
///
/// <summary>
///   This module contains the library functions to handle various sensors.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SENSOR_H
#define _SENSOR_H

#pragma systemFile

#ifdef MOD_ID
  #undef MOD_ID
#endif
#define MOD_ID                  MOD_SENSOR

//
// Constants.
//
#define SENSORZONE_LO           0
#define SENSORZONE_MID          1
#define SENSORZONE_HI           2

#define SENSORF_USER_MASK       0x00ff
#define SENSORF_ENABLE_EVENTS   0x0001
#define SENSORF_INVERSE         0x0002
#ifdef HTSMUX_STATUS
  #define SENSORF_HTSMUX        0x0080
#endif
#define SENSORF_CALIBRATING     0x0100

#define SensorCalibrating(s)    (s.flagsSensor & SENSORF_CALIBRATING)

//
// Type definitions.
//
typedef struct
{
  int idSensor;
  int valueThresholdLo;
  int valueThresholdHi;
  int flagsSensor;
  int valueSensor;
  int zoneSensor;
  int rawMin;
  int rawMax;
} SENSOR;

//
// Import function prototypes.
//
void
SensorEvent(
  __in SENSOR &sensor
  );

/// <summary>
///   This function initializes the sensor system.
/// </summary>
///
/// <param name="sensor">
///   Points to the SENSOR structure to be initialized.
/// </param>
/// <param name="idSensor">
///   Specifies the sensor ID.
/// </param>
/// <param name="valueThresholdLo">
///   Specifies the low threshold value.
/// </param>
/// <param name="valueThresholdHi">
///   Specifies the high threshold value.
/// </param>
/// <param name="flagsSensor">
///   Specifies the sensor flags.
/// </param>
///
/// <returns> None. </returns>

void
SensorInit(
  __out SENSOR &sensor,
  __in int idSensor,
  __in int valueThresholdLo,
  __in int valueThresholdHi,
  __in int flagsSensor
  )
{
  TFuncName("SensorInit");
  TEnter(INIT);

  sensor.idSensor = idSensor;
  sensor.valueThresholdLo = valueThresholdLo;
  sensor.valueThresholdHi = valueThresholdHi;
  sensor.flagsSensor = flagsSensor & SENSORF_USER_MASK;
  sensor.valueSensor = 0;
  sensor.zoneSensor = SENSORZONE_LO;

  TExit(INIT);
  return;
}   //SensorInit

/// <summary>
///   This function starts or stops the sensor calibration process.
/// </summary>
///
/// <param name="sensor">
///   Points to the SENSOR structure.
/// </param>
/// <param name="fStart">
///   Specifies whether to start or stop the calibration.
/// </param>
///
/// <returns> None. </returns>

void
SensorCal(
  __inout SENSOR &sensor,
  __in bool fStart
  )
{
  TFuncName("SensorCal");
  TEnterMsg(API, ("fStart=%d", (byte)fStart));

  if (fStart)
  {
    sensor.flagsSensor |= SENSORF_CALIBRATING;
    sensor.rawMin = 1023;
    sensor.rawMax = 0;
  }
  else
  {
    int zoneRange = (sensor.rawMax - sensor.rawMin)/3;

    sensor.flagsSensor &= ~SENSORF_CALIBRATING;
    sensor.valueThresholdLo = sensor.rawMin + zoneRange;
    sensor.valueThresholdHi = sensor.rawMax - zoneRange;
    TInfo(("ThLo=%d,ThHi=%d",
           sensor.valueThresholdLo, sensor.valueThresholdHi));
  }

  TExit(API);
  return;
}   //SensorCal

/// <summary>
///   This function processes the sensor reading and sends a trigger event
///   if necessary.
/// </summary>
///
/// <param name="sensor">
///   Points to the SENSOR structure.
/// </param>
///
/// <returns> None. </returns>

void
SensorTask(
  __inout SENSOR &sensor
  )
{
  TFuncName("SensorTask");
  TEnter(HIFREQ);

#ifdef HTSMUX_STATUS
  if (sensor.flagsSensor & SENSORF_HTSMUX)
  {
    sensor.valueSensor = 1023 - HTSMUXreadAnalogue((tMUXSensor)sensor.idSensor);
  }
  else
  {
    sensor.valueSensor = SensorRaw[sensor.idSensor];
  }
#else
  sensor.valueSensor = SensorRaw[sensor.idSensor];
#endif
  if (sensor.flagsSensor & SENSORF_CALIBRATING)
  {
    //
    // We are in calibration mode.
    //
    if (sensor.valueSensor < sensor.rawMin)
    {
      sensor.rawMin = sensor.valueSensor;
    }
    else if (sensor.valueSensor > sensor.rawMax)
    {
      sensor.rawMax = sensor.valueSensor;
    }
  }
  else
  {
    int zone;

    if (sensor.valueSensor <= sensor.valueThresholdLo)
    {
      zone = (sensor.flagsSensor & SENSORF_INVERSE)?
             SENSORZONE_HI: SENSORZONE_LO;
    }
    else if (sensor.valueSensor <= sensor.valueThresholdHi)
    {
      zone = SENSORZONE_MID;
    }
    else
    {
      zone = (sensor.flagsSensor & SENSORF_INVERSE)?
             SENSORZONE_LO: SENSORZONE_HI;
    }

    if (zone != sensor.zoneSensor)
    {
      //
      // We have crossed to another zone, let's send a sensor event.
      //
      sensor.zoneSensor = zone;
      if (sensor.flagsSensor & SENSORF_ENABLE_EVENTS)
      {
        SensorEvent(sensor);
      }
    }
  }

  TExit(HIFREQ);
  return;
}   //SensorTask

#endif  //ifndef _SENSOR_H
