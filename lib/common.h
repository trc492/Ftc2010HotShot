#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="common.h" />
///
/// <summary>
///   This module contains common definitions that can be used anywhere.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _COMMON_H
#define _COMMON_H

//
// Macros.
//
#define __in
#define __out
#define __inout

#define ARRAYSIZE(a)            (sizeof(a)/sizeof(a[0]))

//
// The BOUND macro limits the value (n) within the bounds between the given
// low (l) and high (h).
//
#define BOUND(n,l,h)            (((n) < (l))? (l): ((n) > (h))? (h): (n))

//
// The NORMALIZE macro transforms a value (n) in the range between (sl) and
// (sh) to the range between (tl) and (th).
//
#define NORMALIZE(n,sl,sh,tl,th) (int)(((long)(n) - (sl))*((th) - (tl))/((sh) - (sl)) + (tl))

//
// Joystick input macros.
//
#ifndef DEADBAND_INPUT_THRESHOLD
  #define DEADBAND_INPUT_THRESHOLD 20
#endif

//
// This macro ignores input value (n) that is within the DEADBAND_THRESHOLD.
// This is necessary because analog joysticks do not always centered at zero.
// So if the joystick is at the rest position, we will consider it zero even
// though the value is non-zero but within DEADBAND_THRESHOLD.
//
#define DEADBAND(n,t)           ((abs(n) > (t))? (n): 0)
#define DEADBAND_INPUT(n)       DEADBAND(n, DEADBAND_INPUT_THRESHOLD)
//
// This macro limits the input value (n) to the range between -128 and 127.
// This is useful when calculations on the input value may bring the result
// outside of the valid range. This macro will make sure the result is within
// bounds.
//
#define BOUND_INPUT(n)          BOUND(n, -128, 127)

#endif  //ifndef _COMMON_H
