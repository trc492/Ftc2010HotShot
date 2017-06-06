#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="button.h" />
///
/// <summary>
///   This module contains the library functions for the joystick buttons.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _BUTTON_H
#define _BUTTON_H

#include "JoystickDriver.c"

#pragma systemFile

#ifdef MOD_ID
  #undef MOD_ID
#endif
#define MOD_ID                  MOD_BUTTON

//
// Constants.
//
#ifndef NUM_BTNS
  #define NUM_BTNS              12
#endif
#define BTNF_USER_MASK          0x00ff
#define BTNF_ENABLE_EVENTS      0x0001

//
// Macros.
//
#define Btn(n)                  (1 << (n))

#define Logitech_Btn1           Btn(0)
#define Logitech_Btn2           Btn(1)
#define Logitech_Btn3           Btn(2)
#define Logitech_Btn4           Btn(3)
#define Logitech_LB5            Btn(4)
#define Logitech_RB6            Btn(5)
#define Logitech_LB7            Btn(6)
#define Logitech_RB8            Btn(7)
#define Logitech_Btn9           Btn(8)
#define Logitech_Btn10          Btn(9)
#define Logitech_LStick         Btn(10)
#define Logitech_RStick         Btn(11)

#define Xbox_A                  Btn(0)
#define Xbox_B                  Btn(1)
#define Xbox_X                  Btn(2)
#define Xbox_Y                  Btn(3)
#define Xbox_LB                 Btn(4)
#define Xbox_RB                 Btn(5)
#define Xbox_Back               Btn(6)
#define Xbox_Start              Btn(7)
#define Xbox_LStick             Btn(8)
#define Xbox_RStick             Btn(9)

//
// Type definitions.
//
typedef struct
{
  int  idJoystick;
  int  flagsButton;
  int  prevButtons;
  int  maskButton;
  bool fPressed;
} BUTTON;

//
// Import function prototypes.
//
void
ButtonEvent(
  __in BUTTON &button
  );

/// <summary>
///   This function initializes the joystick button system.
/// </summary>
///
/// <param name="button">
///   Points to the BUTTON structure to be initialized.
/// </param>
/// <param name="idJoystick">
///   Specifies the joystick ID.
/// </param>
/// <param name="flagsButton">
///   Specifies the button flags.
/// </param>
///
/// <returns> None. </returns>

void
ButtonInit(
  __out BUTTON &button,
  __in int idJoystick,
  __in int flagsButton
  )
{
  TFuncName("ButtonInit");
  TEnter(INIT);

  button.idJoystick = idJoystick;
  button.flagsButton = flagsButton & BTNF_USER_MASK;
  button.prevButtons = 0;
  button.maskButton = 0;
  button.fPressed = false;

  TExit(INIT);
  return;
}   //ButtonInit

/// <summary>
///   This function processes the changed buttons and sends button event
///   notifications.
/// </summary>
///
/// <param name="button">
///   Points to the BUTTON structure.
/// </param>
///
/// <returns> None. </returns>

void
ButtonTask(
  __inout BUTTON &button
  )
{
  int currButtons = (button.idJoystick == 1)? joystick.joy1_Buttons:
                                              joystick.joy2_Buttons;

  TFuncName("ButtonTask");
  TEnterMsg(HIFREQ, ("Prev=%x,Curr=%x", button.prevButtons, currButtons));

  if (button.flagsButton & BTNF_ENABLE_EVENTS)
  {
    int changedButtons = button.prevButtons^currButtons;

    while (changedButtons != 0)
    {
      //
      // maskButton contains the least significant set bit.
      //
      button.maskButton = changedButtons & ~(changedButtons^-changedButtons);
      if ((currButtons & button.maskButton) != 0)
      {
        //
        // Button is pressed.
        //
        button.fPressed = true;
        ButtonEvent(button);
      }
      else
      {
        //
        // Button is released.
        //
        button.fPressed = false;
        ButtonEvent(button);
      }
      changedButtons &= ~button.maskButton;
    }
  }
  button.prevButtons = currButtons;

  TExit(HIFREQ);
  return;
}   //ButtonTask

#endif  //ifndef _BUTTON_H
