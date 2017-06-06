#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="trace.h" />
///
/// <summary>
///   This module contains the tracing functions and definitions.
/// </summary>
///
/// <remarks>
///   Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _TRACE_H
#define _TRACE_H

#pragma systemFile

//
// Module ID.
//
#define MOD_DRIVE               0x0100
#define MOD_BUTTON              0x0200
#define MOD_SENSOR              0x0400
#define MOD_SM                  0x0800
#define MOD_LNFOLLOW            0x1000
#define MOD_LIB                 (MOD_DRIVE | MOD_BUTTON | MOD_SENSOR | MOD_SM |\
                                 MOD_LNFOLLOW)
#define MOD_MAIN                0x0001
#define TGenModId(n)            ((MOD_MAIN << (n)) && 0xff)

#define INIT                    0
#define API                     1
#define EVENT                   2
#define FUNC                    3
#define UTIL                    4
#define HIFREQ                  5

#define FATAL                   0
#define ERR                     1
#define WARN                    2
#define INFO                    3
#define VERBOSE                 4

//
// Trace macros.
//
#ifdef _DEBUG
  #define TModEnterMsg(m,l,p)   if (((g_TraceModules & (m)) != 0) && \
                                    ((l) <= g_TraceLevel)) \
                                { \
                                  TracePrefix(_strFuncName, true, false); \
                                  debugPrintLine p; \
                                }
  #define TModEnter(m,l)        if (((g_TraceModules & (m)) != 0) && \
                                    ((l) <= g_TraceLevel)) \
                                { \
                                  TracePrefix(_strFuncName, true, true); \
                                }
  #define TModExitMsg(m,l,p)    if (((g_TraceModules & (m)) != 0) && \
                                    ((l) <= g_TraceLevel)) \
                                { \
                                  TracePrefix(_strFuncName, false, false); \
                                  debugPrintLine p; \
                                }
  #define TModExit(m,l)         if (((g_TraceModules & (m)) != 0) && \
                                    ((l) <= g_TraceLevel)) \
                                { \
                                  TracePrefix(_strFuncName, false, true); \
                                }
  #define TModMsg(m,e,p)        if (((g_TraceModules & (m)) != 0) && \
                                    ((e) <= g_MsgLevel)) \
                                { \
                                  MsgPrefix(_strFuncName, e); \
                                  debugPrintLine p; \
                                }
  #define TraceInit(m,l,e)      { \
                                  g_TraceModules = (m); \
                                  g_TraceLevel = (l); \
                                  g_MsgLevel = (e); \
                                }
  #define TFuncName(s)          string _strFuncName = s
  #define TEnterMsg(l,p)        TModEnterMsg(MOD_ID, l, p)
  #define TEnter(l)             TModEnter(MOD_ID, l)
  #define TExitMsg(l,p)         TModExitMsg(MOD_ID, l, p)
  #define TExit(l)              TModExit(MOD_ID, l)
  #define TMsg(e,p)             TModMsg(MOD_ID, e, p)
  #define TFatal(p)             TModMsg(MOD_ID, FATAL, p)
  #define TErr(p)               TModMsg(MOD_ID, ERR, p)
  #define TWarn(p)              TModMsg(MOD_ID, WARN, p)
  #define TInfo(p)              TModMsg(MOD_ID, INFO, p)
  #define TVerbose(p)           TModMsg(MOD_ID, VERBOSE, p)
#else
  #define TModEnterMsg(c,f,p)
  #define TModEnter(c,f)
  #define TModExitMsg(c,f,p)
  #define TModExit(c,f)
  #define TModMsg(m,e,f,p)
  #define TInitTrace(m,l,e)
  #define TFuncName(s)
  #define TEnterMsg(l,p)
  #define TEnter(l)
  #define TExitMsg(l,p)
  #define TExit(l)
  #define TMsg(e,p)
  #define TFatal(p)
  #define TErr(p)
  #define TWarn(p)
  #define TInfo(p)
  #define TVerbose(p)
#endif  //ifdef _DEBUG

#ifdef _DEBUG

int g_TraceModules = 0;
int g_TraceLevel = 0;
int g_MsgLevel = 0;
int g_IndentLevel = 0;

/// <summary>
///   This function prints the trace prefix string to the debug stream.
/// </summary>
///
/// <param name="strFunc">
///   Specifies the function name.
/// </param>
/// <param name="fEnter">
///   Specifies whether this is a TEnter or TExit.
/// </param>
/// <param name="fNewLine">
///   Specifies whether it should print a newline.
/// </param>
///
/// <returns> None. </returns>

void
TracePrefix(
  __in string strFunc,
  __in bool fEnter,
  __in bool fNewLine
  )
{
  if (fEnter)
  {
    g_IndentLevel++;
  }

  for (int i = 0; i < g_IndentLevel; ++i)
  {
    writeDebugStream("| ");
  }

  writeDebugStream(strFunc);

  if (fEnter)
  {
    writeDebugStream(": ");
  }
  else
  {
    writeDebugStream("! ");
  }

  if (fNewLine)
  {
    writeDebugStream("\n\r");
  }

  if (!fEnter)
  {
    g_IndentLevel--;
  }

  return;
}   //TracePrefix

/// <summary>
///   This function prints the message prefix string to the debug stream.
/// </summary>
///
/// <param name="strFunc">
///   Specifies the function name.
/// </param>
/// <param name="msgLevel">
///   Specifies the message level.
/// </param>
///
/// <returns> None. </returns>

void
MsgPrefix(
  __in string strFunc,
  __in int msgLevel
  )
{
  writeDebugStream(strFunc);
  switch (msgLevel)
  {
    case FATAL:
      writeDebugStream("_Fatal:");
      break;

    case ERR:
      writeDebugStream("_Err:");
      break;

    case WARN:
      writeDebugStream("_Warn:");
      break;

    case INFO:
      writeDebugStream("_Info:");
      break;

    case VERBOSE:
      writeDebugStream("_Verbose:");
      break;

    default:
      writeDebugStream("_Unk:");
      break;
  }

  return;
}   //MsgPrefix

#endif  //ifdef _DEBUG

#endif  //ifndef _TRACE_H
