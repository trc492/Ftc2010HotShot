#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="sm.h" />
///
/// <summary>
///   This module contains the library functions to handle state machines.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SM_H
#define _SM_H

#pragma systemFile

#ifdef MOD_ID
  #undef MOD_ID
#endif
#define MOD_ID                  MOD_SM

//
// Constants.
//
#define SMF_READY               0x8000
#define SMF_MASK                0x00ff
#define SMF_WAIT_ALL            0x0001
#define SMF_CLEAR_EVENTS        0x0002

#define SMSTATE_DISABLED        0
#define SMSTATE_STARTED         1

#define EVTTYPE_NONE            0

#ifndef MAX_WAIT_EVENTS
#define MAX_WAIT_EVENTS         4
#endif

//
// Macros.
//
#define IsSMDisabled(s)         (s.currState == SMSTATE_DISABLED)
#define IsSMEnabled(s)          (s.currState != SMSTATE_DISABLED)
#define IsSMReady(s)            (s.flagsSM & SMF_READY)

//
// Type definitions.
//
typedef struct
{
  int  evtType;
  int  evtID;
  int  evtData;
  int  evtParam1;
  int  evtParam2;
  bool fSignaled;
} WAIT_EVT;

typedef struct
{
  int  currState;
  int  nextState;
  int  flagsSM;
  bool fClearEvts;
  int  nWaitEvents;
  WAIT_EVT WaitEvents[MAX_WAIT_EVENTS];
} SM;

/// <summary>
///   This function clears all wait events in the state machine.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure.
/// </param>
///
/// <returns> None. </returns>

void
SMClearAllEvents(
  __out SM &sm
  )
{
  TFuncName("SMClearAllEvents");
  TEnter(FUNC);

  for (int i = 0; i < MAX_WAIT_EVENTS; ++i)
  {
    sm.WaitEvents[i].evtType = EVTTYPE_NONE;
    sm.WaitEvents[i].evtID = 0;
    sm.WaitEvents[i].evtData = 0;
    sm.WaitEvents[i].evtParam1 = 0;
    sm.WaitEvents[i].evtParam2 = 0;
    sm.WaitEvents[i].fSignaled = false;
  }
  sm.nWaitEvents = 0;

  TExit(FUNC);
  return;
}   //SMClearAllEvents

/// <summary>
///   This function initializes the state machine.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure to be initialized.
/// </param>
///
/// <returns> None. </returns>

void
SMInit(
  __out SM &sm
  )
{
  TFuncName("SMInit");
  TEnter(INIT);

  sm.currState = SMSTATE_DISABLED;
  sm.nextState = SMSTATE_DISABLED;
  sm.flagsSM = 0;
  sm.fClearEvts = false;
  SMClearAllEvents(sm);

  TExit(INIT);
  return;
}   //SMInit

/// <summary>
///   This function starts the state machine.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure to be initialized.
/// </param>
///
/// <returns> None. </returns>

void
SMStart(
  __out SM &sm
  )
{
  TFuncName("SMStart");
  TEnter(API);

  if (sm.currState == SMSTATE_DISABLED)
  {
    sm.currState = SMSTATE_STARTED;
    sm.nextState = SMSTATE_STARTED;
    sm.flagsSM = SMF_READY;
  }

  TExit(API);
  return;
}   //SMStart

/// <summary>
///   This function stops the state machine.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure to be initialized.
/// </param>
///
/// <returns> None. </returns>

void
SMStop(
  __out SM &sm
  )
{
  TFuncName("SMStop");
  TEnter(API);

  SMInit(sm);

  TExit(API);
  return;
}   //SMStop

/// <summary>
///   This function adds a wait event to the state machine.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure.
/// </param>
/// <param name="evtType">
///   Specifies the event type to wait for.
/// </param>
/// <param name="evtID">
///   Specifies the event ID to wait for.
/// </param>
/// <param name="evtData">
///   Specifies the event data to wait for.
/// </param>
///
/// <returns> Success: Return true. </returns>
/// <returns> Failure: Return false. </returns>

bool
SMAddWaitEvent(
  __inout SM &sm,
  __in int evtType,
  __in int evtID,
  __in int evtData
  )
{
  TFuncName("SMAddWaitEvent");
  TEnterMsg(API, ("Type=%x,ID=%x", evtType, evtID));

  bool fAdded = false;

  if (sm.nWaitEvents < MAX_WAIT_EVENTS)
  {
    sm.WaitEvents[sm.nWaitEvents].evtType = evtType;
    sm.WaitEvents[sm.nWaitEvents].evtID = evtID;
    sm.WaitEvents[sm.nWaitEvents].evtData = evtData;
    sm.WaitEvents[sm.nWaitEvents].evtParam1 = 0;
    sm.WaitEvents[sm.nWaitEvents].evtParam2 = 0;
    sm.WaitEvents[sm.nWaitEvents].fSignaled = false;
    sm.nWaitEvents++;
    fAdded = true;
  }
  TInfo(("nEvts=%d", sm.nWaitEvents));

  TExitMsg(API, ("fOK=%d", (byte)fAdded));
  return fAdded;
}   //SMAddWaitEvent

/// <summary>
///   This function sets the wait event mode and the next state to advance
///   to when the wait is fulfilled.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure.
/// </param>
/// <param name="nextState">
///   Specifies the next state to advance to.
/// </param>
/// <param name="flags">
///   Specifies the SMF flags.
/// </param>
///
/// <returns> None. </returns>

void
SMWaitEvents(
  __inout SM &sm,
  __in int nextState,
  __in int flags
  )
{
  TFuncName("SMWaitEvents");
  TEnterMsg(API, ("Next=%d,flags=%x", nextState, flags));

  if (sm.nWaitEvents > 0)
  {
    sm.nextState = nextState;
    sm.flagsSM |= flags & SMF_MASK;
    sm.flagsSM &= ~SMF_READY;
    if (sm.flagsSM & SMF_CLEAR_EVENTS)
    {
      sm.fClearEvts = true;
      sm.flagsSM &= ~SMF_CLEAR_EVENTS;
    }
    else
    {
      sm.fClearEvts = false;
    }
  }

  TExit(API);
  return;
}   //SMWaitEvents

/// <summary>
///   This function is called when an event has occurred. It will determine
///   if the state machine is waiting for the event and will advance the
///   state machine to the next state if necessary.
/// </summary>
///
/// <param name="sm">
///   Points to the SM structure.
/// </param>
/// <param name="evtType">
///   Specifies the event type.
/// </param>
/// <param name="evtID">
///   Specifies the event ID.
/// </param>
/// <param name="evtData">
///   Specifies the event data.
/// </param>
/// <param name="evtParam1">
///   Specifies the event parameter 1.
/// </param>
/// <param name="evtParam2">
///   Specifies the event parameter 2.
/// </param>
///
/// <returns> None. </returns>

void
SMSetEvent(
  __inout SM &sm,
  __in int evtType,
  __in int evtID,
  __in int evtData,
  __in int evtParam1,
  __in int evtParam2
  )
{
  TFuncName("SMSetEvent");
  TEnterMsg(EVENT, ("Type=%x,ID=%x", evtType, evtID));

  for (int i = 0; i < sm.nWaitEvents; ++i)
  {
    if (!sm.WaitEvents[i].fSignaled &&
        (sm.WaitEvents[i].evtType == evtType) &&
        ((sm.WaitEvents[i].evtID == -1) || (sm.WaitEvents[i].evtID == evtID)) &&
        ((sm.WaitEvents[i].evtData == -1) || (sm.WaitEvents[i].evtData == evtData)))
    {
      //
      // If the all event attributes matched or we don't care some of them,
      // we mark the event signaled.
      //
      TInfo(("Type=%x,ID=%x",
            sm.WaitEvents[i].evtType, sm.WaitEvents[i].evtID));
      sm.WaitEvents[i].fSignaled = true;
      sm.WaitEvents[i].evtID = evtID;
      sm.WaitEvents[i].evtData = evtData;
      sm.WaitEvents[i].evtParam1 = evtParam1;
      sm.WaitEvents[i].evtParam2 = evtParam2;

      bool fAdvanceState = true;
      if (sm.flagsSM & SMF_WAIT_ALL)
      {
        for (int j = 0; j < sm.nWaitEvents; ++j)
        {
          if (!sm.WaitEvents[j].fSignaled)
          {
            fAdvanceState = false;
            break;
          }
        }
      }

      if (fAdvanceState)
      {
        TInfo(("AdvanceState"));
        if (sm.fClearEvts)
        {
          sm.fClearEvts = false;
          SMClearAllEvents(sm);
        }
        sm.currState = sm.nextState;
        sm.flagsSM |= SMF_READY;
      }
      break;
    }
  }

  TExit(EVENT);
  return;
}   //SMSetEvent

#endif  //ifndef _SM_H
