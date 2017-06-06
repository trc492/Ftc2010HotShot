#pragma config(Sensor, S4,     HTSMUX,              sensorLowSpeed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 * $Id: LEGOLS-SMUX-test1.c 20 2009-12-08 22:59:13Z xander $
 */

/**
 * LEGOLS-driver.h provides an API for the Lego Light Sensor.  This program
 * demonstrates how to use that API through a SMUX.
 *
 * Changelog:
 * - 0.1: Initial release
 * - 0.2: More comments
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 2.00 AND HIGHER.
 * Xander Soldaat (mightor_at_gmail.com)
 * 25 November 2009
 * version 0.2
 */

#include "..\drivers\LEGOLS-driver.h"

task main() {
  int raw1 = 0;
  int raw2 = 0;
  int raw3 = 0;
  bool active = true;

  // Before using the SMUX, you need to initialise the driver
  HTSMUXinit();

  // Tell the SMUX to scan its ports for connected sensors
  HTSMUXscanPorts(HTSMUX);

  // The sensor is connected to the first port
  // of the SMUX which is connected to the NXT port S1.
  // To access that sensor, we must use msensor_S1_1.  If the sensor
  // were connected to 3rd port of the SMUX connected to the NXT port S4,
  // we would use msensor_S4_3

  LSsetActive(msensor_S4_1);
  LSsetActive(msensor_S4_2);
  LSsetActive(msensor_S4_3);

  nNxtButtonTask  = -2;

  nxtDisplayCenteredTextLine(0, "Lego");
  nxtDisplayCenteredBigTextLine(1, "LIGHT");
  nxtDisplayCenteredTextLine(3, "SMUX Test");
  nxtDisplayCenteredTextLine(5, "Connect SMUX to");
  nxtDisplayCenteredTextLine(6, "S1 and sensor to");
  nxtDisplayCenteredTextLine(7, "SMUX Port 1");
  wait1Msec(2000);

  nxtDisplayClearTextLine(7);
  nxtDisplayTextLine(5, "Press [enter]");
  nxtDisplayTextLine(6, "to toggle light");
  wait1Msec(2000);

  while (true) {
    // The enter button has been pressed, switch
    // to the other mode
    if (nNxtButtonPressed == kEnterButton) {
      active = !active;
      if (!active)
      {
        LSsetInactive(msensor_S4_1);
        LSsetInactive(msensor_S4_2);
        LSsetInactive(msensor_S4_3);
      }
      else
      {
        LSsetActive(msensor_S4_1);
        LSsetActive(msensor_S4_2);
        LSsetActive(msensor_S4_3);
      }

      // wait 500ms to debounce the switch
      wait1Msec(500);
    }

    nxtDisplayClearTextLine(5);
    nxtDisplayClearTextLine(6);
    raw1 = LSvalRaw(msensor_S4_1);
    raw2 = LSvalRaw(msensor_S4_2);
    raw3 = LSvalRaw(msensor_S4_3);
    nxtDisplayTextLine(4, "Raw1: %4d", raw1);
    nxtDisplayTextLine(5, "Raw2: %4d", raw2);
    nxtDisplayTextLine(6, "Raw3: %4d", raw3);
    wait1Msec(50);
  }
}

/*
 * $Id: LEGOLS-SMUX-test1.c 20 2009-12-08 22:59:13Z xander $
 */
