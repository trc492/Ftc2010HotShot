#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="display.h" />
///
/// <summary>
///   This module contains the library functions for the NXT LCD display.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _DISPLAY_H
#define _DISPLAY_H

task DisplayTask()
{
  while (true)
  {
    if (externalBatteryAvg < 0)
    {
      //
      // External battery is off or not connected.
      //
      nxtDisplayTextLine(6, "Ext Batt:OFF");
    }
    else
    {
      nxtDisplayTextLine(6, "Ext Batt:%4.1f V",
                         (float)externalBatteryAvg/1000.0);
    }
    nxtDisplayTextLine(7, "NXT Batt:%4.1f V", (float)nAvgBatteryLevel/1000.0);
    wait1Msec(200);
  }
}   //DisplayTask

/// <summary>
///   This function initializes the NXT LCD display.
/// </summary>
///
/// <returns> None. </returns>

void
DisplayInit()
{
  StopTask(displayDiagnostics);
  eraseDisplay();
  StartTask(DisplayTask);
  return;
}   //DisplayInit

#endif  //ifndef _DISPLAY_H
