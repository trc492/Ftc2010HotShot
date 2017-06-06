/*
 * $Id: HTDIR-driver.h 20 2009-12-08 22:59:13Z xander $
 */

#ifndef __HTDIR_H__
#define __HTDIR_H__
/** \file HTDIR-driver.h
 * \brief HiTechnic IR Seeker V2 driver
 *
 * HTDIR-driver.h provides an API for the HiTechnic IR Seeker V2.
 *
 * Changelog:
 * - 0.1: Initial release
 * - 0.2: Added SMUX functions
 * - 0.3: All functions using tIntArray are now pass by reference.<br>
 *        HTDIR_SMUXData removed
 * - 0.4: Removed all calls to ubyteToInt()<br>
 *        Replaced all functions that used SPORT/MPORT macros
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where its due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 2.00 AND HIGHER.
 * \author Xander Soldaat (mightor_at_gmail.com)
 * \date 29 November 2009
 * \version 0.4
 * \example HTDIR-test1.c
 * \example HTDIR-SMUX-test1.c
 */

#pragma systemFile

#ifndef __COMMON_H__
#include "common.h"
#endif

#define HTDIR_I2C_ADDR    0x10      /*!< IR Seeker I2C device address */
#define HTDIR_DSP_MODE    0x41      /*!< AC DSP mode - 0 = 1200Hz, 1 = 600Hz */
#define HTDIR_OFFSET      0x42      /*!< Offset for data registers */
#define HTDIR_DC_DIR      0x00      /*!< DC Direction data */
#define HTDIR_DC_SSTR1    0x01      /*!< DC Sensor 0 signal strength above average */
#define HTDIR_DC_SSTR2    0x02      /*!< DC Sensor 1 signal strength above average */
#define HTDIR_DC_SSTR3    0x03      /*!< DC Sensor 2 signal strength above average */
#define HTDIR_DC_SSTR4    0x04      /*!< DC Sensor 3 signal strength above average */
#define HTDIR_DC_SSTR5    0x05      /*!< DC Sensor 4 signal strength above average */
#define HTDIR_DC_SAVG     0x06      /*!< DC sensor signal strength average */
#define HTDIR_AC_DIR      0x07      /*!< DC Direction data */
#define HTDIR_AC_SSTR1    0x08      /*!< DC Sensor 0 signal strength above average */
#define HTDIR_AC_SSTR2    0x09      /*!< DC Sensor 1 signal strength above average */
#define HTDIR_AC_SSTR3    0x0A      /*!< DC Sensor 2 signal strength above average */
#define HTDIR_AC_SSTR4    0x0B      /*!< DC Sensor 3 signal strength above average */
#define HTDIR_AC_SSTR5    0x0C      /*!< DC Sensor 4 signal strength above average */


/*!< AC DSP modes */
typedef enum {
  DSP_1200 = 0,
  DSP_600 = 1
} tHTDIRDSPMode;

// ---------------------------- DC Signal processing -----------------------------
int HTDIRreadDCDir(tSensors link);
int HTDIRreadDCDir(tMUXSensor muxsensor);
int HTDIRreadDCStrength(tSensors link, byte sensorNr);
int HTDIRreadDCStrength(tMUXSensor muxsensor, byte sensorNr);
bool HTDIRreadAllDCStrength(tSensors link, int &dcS1, int &dcS2, int &dcS3, int &dcS4, int &dcS5);
bool HTDIRreadAllDCStrength(tMUXSensor muxsensor, int &dcS1, int &dcS2, int &dcS3, int &dcS4, int &dcS5);
int HTDIRreadDCAverage(tSensors link);
int HTDIRreadDCAverage(tMUXSensor muxsensor);
// ---------------------------- AC Signal processing -----------------------------
bool HTDIRsetDSPMode(tSensors link, tHTDIRDSPMode mode);
int HTDIRreadACDir(tSensors link);
int HTDIRreadACDir(tMUXSensor muxsensor);
int HTDIRreadACStrength(tSensors link, byte sensorNr);
int HTDIRreadACStrength(tMUXSensor muxsensor, byte sensorNr);
bool HTDIRreadAllACStrength(tSensors link, int &acS1, int &acS2, int &acS3, int &acS4, int &acS5);
bool HTDIRreadAllACStrength(tMUXSensor muxsensor, int &acS1, int &acS2, int &acS3, int &acS4, int &acS5);

tByteArray HTDIR_I2CRequest;    /*!< Array to hold I2C command data */
tByteArray HTDIR_I2CReply;      /*!< Array to hold I2C reply data */

// ---------------------------- DC Signal processing -----------------------------

/**
 * Read the value of the DC Direction data register and return it.
 * @param link the HTDIR port number
 * @return value of 0-9, the direction index of the detected IR signal or -1 if an error occurred.
 */
int HTDIRreadDCDir(tSensors link) {
  memset(HTDIR_I2CRequest, 0, sizeof(tByteArray));

  HTDIR_I2CRequest.arr[0] = 2;              // Message size
  HTDIR_I2CRequest.arr[1] = HTDIR_I2C_ADDR; // I2C Address
  HTDIR_I2CRequest.arr[2] = HTDIR_OFFSET + HTDIR_DC_DIR;  // Start direction register

  if (!writeI2C(link, HTDIR_I2CRequest, 1))
    return -1;

  if (!readI2C(link, HTDIR_I2CReply, 1))
    return -1;

  return HTDIR_I2CReply.arr[0];
}


/**
 * Read the value of the DC Direction data register and return it.
 * @param muxsensor the SMUX sensor port number
 * @return value of 0-9, the direction index of the detected IR signal or -1 if an error occurred.
 */
int HTDIRreadDCDir(tMUXSensor muxsensor) {
	memset(HTDIR_I2CReply, 0, sizeof(tByteArray));

  if (HTSMUXreadSensorType(muxsensor) != HTSMUXIRSeekerNew)
    return -1;

  if (!HTSMUXreadPort(muxsensor, HTDIR_I2CReply, 1, HTDIR_DC_DIR)) {
    return -1;
  }

  return HTDIR_I2CReply.arr[0];
}


/**
 * Read the value of the specified internal DC sensor above average, numbered 0-5 and return it.
 * @param link the HTDIR port number
 * @param sensorNr the internal sensor to read
 * @return the signal strength value of the specified sensor or -1 if an error occurred.
 */
int HTDIRreadDCStrength(tSensors link, byte sensorNr) {
  memset(HTDIR_I2CRequest, 0, sizeof(tByteArray));

  HTDIR_I2CRequest.arr[0] = 2;                      // Message size
  HTDIR_I2CRequest.arr[1] = HTDIR_I2C_ADDR;         // I2C Address
  HTDIR_I2CRequest.arr[2] = HTDIR_OFFSET + HTDIR_DC_SSTR1 + sensorNr; // Address of Sensor <sensorNr> signal strength

  if (!writeI2C(link, HTDIR_I2CRequest, 1))
    return -1;

  if (!readI2C(link, HTDIR_I2CReply, 1))
    return -1;

  return HTDIR_I2CReply.arr[0];
}


/**
 * Read the value of the specified internal DC sensor above average, numbered 0-5 and return it.
 * @param muxsensor the SMUX sensor port number
 * @param sensorNr the internal sensor to read
 * @return the signal strength value of the specified sensor or -1 if an error occurred.
 */
int HTDIRreadDCStrength(tMUXSensor muxsensor, byte sensorNr) {
  memset(HTDIR_I2CReply, 0, sizeof(tByteArray));

  if (HTSMUXreadSensorType(muxsensor) != HTSMUXIRSeekerNew)
    return -1;

  if (!HTSMUXreadPort(muxsensor, HTDIR_I2CReply, 1, HTDIR_DC_SSTR1 + sensorNr)) {
    return -1;
  }

  return HTDIR_I2CReply.arr[0];
}


/**
 * Read the value of the all of the internal DC sensors above average.
 * @param link the HTDIR port number
 * @param dcS1 data from internal sensor nr 1
 * @param dcS2 data from internal sensor nr 2
 * @param dcS3 data from internal sensor nr 3
 * @param dcS4 data from internal sensor nr 4
 * @param dcS5 data from internal sensor nr 5
 * @return true if no error occured, false if it did
 */
bool HTDIRreadAllDCStrength(tSensors link, int &dcS1, int &dcS2, int &dcS3, int &dcS4, int &dcS5) {
  memset(HTDIR_I2CRequest, 0, sizeof(tByteArray));

  HTDIR_I2CRequest.arr[0] = 2;                      // Message size
  HTDIR_I2CRequest.arr[1] = HTDIR_I2C_ADDR;         // I2C Address
  HTDIR_I2CRequest.arr[2] = HTDIR_OFFSET + HTDIR_DC_SSTR1;         // Sensor 0 signal strength

  if (!writeI2C(link, HTDIR_I2CRequest, 5))
    return false;

  if (!readI2C(link, HTDIR_I2CReply, 5))
    return false;

  dcS1 = HTDIR_I2CReply.arr[0];
  dcS2 = HTDIR_I2CReply.arr[1];
  dcS3 = HTDIR_I2CReply.arr[2];
  dcS4 = HTDIR_I2CReply.arr[3];
  dcS5 = HTDIR_I2CReply.arr[4];

  return true;
}


/**
 * Read the value of the all of the internal DC sensors above average.
 * @param muxsensor the SMUX sensor port number
 * @param dcS1 data from internal sensor nr 1
 * @param dcS2 data from internal sensor nr 2
 * @param dcS3 data from internal sensor nr 3
 * @param dcS4 data from internal sensor nr 4
 * @param dcS5 data from internal sensor nr 5
 * @return true if no error occured, false if it did
 */
bool HTDIRreadAllDCStrength(tMUXSensor muxsensor, int &dcS1, int &dcS2, int &dcS3, int &dcS4, int &dcS5) {
  memset(HTDIR_I2CReply, 0, sizeof(tByteArray));

  if (HTSMUXreadSensorType(muxsensor) != HTSMUXIRSeekerNew)
    return false;

  if (!HTSMUXreadPort(muxsensor, HTDIR_I2CReply, 5, HTDIR_DC_SSTR1)) {
    return false;
  }

  dcS1 = HTDIR_I2CReply.arr[0];
  dcS2 = HTDIR_I2CReply.arr[1];
  dcS3 = HTDIR_I2CReply.arr[2];
  dcS4 = HTDIR_I2CReply.arr[3];
  dcS5 = HTDIR_I2CReply.arr[4];

  return true;
}


/**
 * Read the value of the average data register and return it.
 * @param link the HTDIR port number
 * @return value of 0-9, the direction index of the detected IR signal or -1 if an error occurred.
 */
int HTDIRreadDCAverage(tSensors link) {
  memset(HTDIR_I2CRequest, 0, sizeof(tByteArray));

  HTDIR_I2CRequest.arr[0] = 2;              // Message size
  HTDIR_I2CRequest.arr[1] = HTDIR_I2C_ADDR; // I2C Address
  HTDIR_I2CRequest.arr[2] = HTDIR_OFFSET + HTDIR_DC_SAVG;  // DC sensor signal strength average

  if (!writeI2C(link, HTDIR_I2CRequest, 1))
    return -1;

  if (!readI2C(link, HTDIR_I2CReply, 1))
    return -1;

  return HTDIR_I2CReply.arr[0];
}


/**
 * Read the value of the average data register and return it.
 * @param muxsensor the SMUX sensor port number
 * @return value of 0-9, the direction index of the detected IR signal or -1 if an error occurred.
 */
int HTDIRreadDCAverage(tMUXSensor muxsensor) {
  memset(HTDIR_I2CReply, 0, sizeof(tByteArray));

  if (HTSMUXreadSensorType(muxsensor) != HTSMUXIRSeekerNew)
    return -1;

  if (!HTSMUXreadPort(muxsensor, HTDIR_I2CReply, 1, HTDIR_DC_SAVG)) {
    return -1;
  }

  return HTDIR_I2CReply.arr[0];
}


// ---------------------------- AC Signal processing -----------------------------

/**
 * Set the DSP mode of the AC carrier wave detector.
 *
 * Mode is one of:
 * -DSP_1200
 * -DSP_600
 * @param link the HTDIR port number
 * @param mode the frequency that should be detected
 * @return true if no error occured, false if it did
 */
bool HTDIRsetDSPMode(tSensors link, tHTDIRDSPMode mode) {
  memset(HTDIR_I2CRequest, 0, sizeof(tByteArray));

  HTDIR_I2CRequest.arr[0] = 3;              // Message size
  HTDIR_I2CRequest.arr[1] = HTDIR_I2C_ADDR; // I2C Address
  HTDIR_I2CRequest.arr[2] = HTDIR_DSP_MODE; // Start direction register
  HTDIR_I2CRequest.arr[3] = mode;

  return writeI2C(link, HTDIR_I2CRequest, 0);
}

/**
 * Read the value of the AC Direction data register and return it.
 * @param link the HTDIR port number
 * @return value of 0-9, the direction index of the detected IR signal or -1 if an error occurred.
 */
int HTDIRreadACDir(tSensors link) {
  memset(HTDIR_I2CRequest, 0, sizeof(tByteArray));

  HTDIR_I2CRequest.arr[0] = 2;              // Message size
  HTDIR_I2CRequest.arr[1] = HTDIR_I2C_ADDR; // I2C Address
  HTDIR_I2CRequest.arr[2] = HTDIR_OFFSET + HTDIR_AC_DIR;      // Start direction register

  if (!writeI2C(link, HTDIR_I2CRequest, 1))
    return -1;

  if (!readI2C(link, HTDIR_I2CReply, 1))
    return -1;

  return HTDIR_I2CReply.arr[0];
}


/**
 * Read the value of the AC Direction data register and return it.
 * @param muxsensor the SMUX sensor port number
 * @return value of 0-9, the direction index of the detected IR signal or -1 if an error occurred.
 */
int HTDIRreadACDir(tMUXSensor muxsensor) {
  memset(HTDIR_I2CReply, 0, sizeof(tByteArray));

  if (HTSMUXreadSensorType(muxsensor) != HTSMUXIRSeekerNew)
    return -1;

  if (!HTSMUXreadPort(muxsensor, HTDIR_I2CReply, 1, HTDIR_AC_DIR)) {
    return -1;
  }

  return HTDIR_I2CReply.arr[0];
}


/**
 * Read the value of the specified internal AC sensor above average, numbered 0-5 and return it.
 * @param link the HTDIR port number
 * @param sensorNr the internal sensor to read
 * @return the signal strength value of the specified sensor or -1 if an error occurred.
 */
int HTDIRreadACStrength(tSensors link, byte sensorNr) {
  memset(HTDIR_I2CRequest, 0, sizeof(tByteArray));

  HTDIR_I2CRequest.arr[0] = 2;                      // Message size
  HTDIR_I2CRequest.arr[1] = HTDIR_I2C_ADDR;         // I2C Address
  HTDIR_I2CRequest.arr[2] = HTDIR_OFFSET + HTDIR_AC_SSTR1 + sensorNr; // Address of Sensor <sensorNr> signal strength

  if (!writeI2C(link, HTDIR_I2CRequest, 1))
    return -1;

  if (!readI2C(link, HTDIR_I2CReply, 1))
    return -1;

  return HTDIR_I2CReply.arr[0];
}


/**
 * Read the value of the specified internal AC sensor above average, numbered 0-5 and return it.
 * @param muxsensor the SMUX sensor port number
 * @param sensorNr the internal sensor to read
 * @return the signal strength value of the specified sensor or -1 if an error occurred.
 */
int HTDIRreadACStrength(tMUXSensor muxsensor, byte sensorNr) {
  memset(HTDIR_I2CReply, 0, sizeof(tByteArray));

  if (HTSMUXreadSensorType(muxsensor) != HTSMUXIRSeekerNew)
    return -1;

  if (!HTSMUXreadPort(muxsensor, HTDIR_I2CReply, 1, HTDIR_AC_SSTR1 + sensorNr)) {
    return -1;
  }

  return HTDIR_I2CReply.arr[0];
}


/**
 * Read the value of the all of the internal AC sensors and copy into specified buffer.
 * @param link the HTDIR port number
 * @param acS1 data from internal sensor nr 1
 * @param acS2 data from internal sensor nr 2
 * @param acS3 data from internal sensor nr 3
 * @param acS4 data from internal sensor nr 4
 * @param acS5 data from internal sensor nr 5
 * @return true if no error occured, false if it did
 */
bool HTDIRreadAllACStrength(tSensors link, int &acS1, int &acS2, int &acS3, int &acS4, int &acS5) {
  memset(HTDIR_I2CRequest, 0, sizeof(tByteArray));

  HTDIR_I2CRequest.arr[0] = 2;                      // Message size
  HTDIR_I2CRequest.arr[1] = HTDIR_I2C_ADDR;         // I2C Address
  HTDIR_I2CRequest.arr[2] = HTDIR_OFFSET + HTDIR_AC_SSTR1;         // Sensor 0 signal strength

  if (!writeI2C(link, HTDIR_I2CRequest, 5))
    return false;

  if (!readI2C(link, HTDIR_I2CReply, 5))
    return false;

  acS1 = HTDIR_I2CReply.arr[0];
  acS2 = HTDIR_I2CReply.arr[1];
  acS3 = HTDIR_I2CReply.arr[2];
  acS4 = HTDIR_I2CReply.arr[3];
  acS5 = HTDIR_I2CReply.arr[4];

  return true;
}


/**
 * Read the value of the all of the internal AC sensors and copy into specified buffer.
 * @param muxsensor the SMUX sensor port number
 * @param acS1 data from internal sensor nr 1
 * @param acS2 data from internal sensor nr 2
 * @param acS3 data from internal sensor nr 3
 * @param acS4 data from internal sensor nr 4
 * @param acS5 data from internal sensor nr 5
 * @return true if no error occured, false if it did
 */
bool HTDIRreadAllACStrength(tMUXSensor muxsensor, int &acS1, int &acS2, int &acS3, int &acS4, int &acS5) {
  memset(HTDIR_I2CReply, 0, sizeof(tByteArray));

  if (HTSMUXreadSensorType(muxsensor) != HTSMUXIRSeekerNew)
    return false;

  if (!HTSMUXreadPort(muxsensor, HTDIR_I2CReply, 5, HTDIR_AC_SSTR1)) {
    return false;
  }

  acS1 = HTDIR_I2CReply.arr[0];
  acS2 = HTDIR_I2CReply.arr[1];
  acS3 = HTDIR_I2CReply.arr[2];
  acS4 = HTDIR_I2CReply.arr[3];
  acS5 = HTDIR_I2CReply.arr[4];

  return true;
}

#endif // __HTDIR_H__

/*
 * $Id: HTDIR-driver.h 20 2009-12-08 22:59:13Z xander $
 */
