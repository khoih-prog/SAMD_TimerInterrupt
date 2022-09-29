/****************************************************************************************************************************
  SAMD_ISR_Timer.hpp
  For SAMD boards
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/SAMD_TimerInterrupt
  Licensed under MIT license

  Now even you use all these new 16 ISR-based timers,with their maximum interval practically unlimited (limited only by
  unsigned long miliseconds), you just consume only one SAMD timer and avoid conflicting with other cores' tasks.
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.

  Based on SimpleTimer - A timer library for Arduino.
  Author: mromani@ottotecnica.com
  Copyright (c) 2010 OTTOTECNICA Italy

  Based on BlynkTimer.h
  Author: Volodymyr Shymanskyy

  Version: 1.10.1

  Version  Modified By   Date      Comments
  -------  -----------  ---------- -----------
  1.0.0    K Hoang      30/10/2020 Initial coding
  1.0.1    K Hoang      06/11/2020 Add complicated example ISR_16_Timers_Array using all 16 independent ISR Timers.
  1.1.1    K.Hoang      06/12/2020 Add Change_Interval example. Bump up version to sync with other TimerInterrupt Libraries
  1.2.0    K.Hoang      08/01/2021 Add better debug feature. Optimize code and examples to reduce RAM usage
  1.3.0    K.Hoang      02/04/2021 Add support to Sparkfun SAMD21 and SAMD51 boards
  1.3.1    K.Hoang      09/05/2021 Fix compile error to some SAMD21-based boards
  1.4.0    K.Hoang      02/06/2021 Fix SAMD21 rare bug caused by not fully init Prescaler
  1.5.0    K.Hoang      08/10/2021 Improve frequency precision by using float instead of ulong
  1.6.0    K.Hoang      20/01/2022 Fix `multiple-definitions` linker error. Add support to many more boards
  1.7.0    K.Hoang      25/04/2022 Optimize code for setInterval() of SAMD21 TC3
  1.8.0    K.Hoang      07/05/2022 Scrap the buggy code in v1.7.0 for TC3
  1.9.0    K.Hoang      08/05/2022 Add TC4, TC5, TCC1 and TCC2 Timers to SAMD21
  1.10.0   K.Hoang      29/09/2022 Avoid conflict with Servo library. Modify all examples. Prevent overflow of TCx
  1.10.1   K.Hoang      30/09/2022 Using float instead of ulong for interval. Prevent overflow of SAMD51 TCx
*****************************************************************************************************************************/

#pragma once

#ifndef ISR_TIMER_GENERIC_HPP
#define ISR_TIMER_GENERIC_HPP

#if !( defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRWIFI1010) \
      || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310) \
      || defined(ARDUINO_SAMD_MKRGSM1400) || defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) \
      || defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(__SAMD51__) || defined(__SAMD51J20A__) \
      || defined(__SAMD51J19A__) || defined(__SAMD51G19A__) || defined(__SAMD51P19A__)  \
      || defined(__SAMD21E15A__) || defined(__SAMD21E16A__) || defined(__SAMD21E17A__) || defined(__SAMD21E18A__) \
      || defined(__SAMD21G15A__) || defined(__SAMD21G16A__) || defined(__SAMD21G17A__) || defined(__SAMD21G18A__) \
      || defined(__SAMD21J15A__) || defined(__SAMD21J16A__) || defined(__SAMD21J17A__) || defined(__SAMD21J18A__) )
  #error This code is designed to run on SAMD21/SAMD51 platform! Please check your Tools->Board setting.
#endif

#ifndef SAMD_TIMER_INTERRUPT_VERSION
  #define SAMD_TIMER_INTERRUPT_VERSION            "SAMDTimerInterrupt v1.10.0"
  
  #define SAMD_TIMER_INTERRUPT_VERSION_MAJOR      1
  #define SAMD_TIMER_INTERRUPT_VERSION_MINOR      10
  #define SAMD_TIMER_INTERRUPT_VERSION_PATCH      0

  #define SAMD_TIMER_INTERRUPT_VERSION_INT        1010000
#endif

#include "TimerInterrupt_Generic_Debug.h"

#include <stddef.h>

#include <inttypes.h>

#if defined(ARDUINO)
  #if ARDUINO >= 100
    #include <Arduino.h>
  #else
    #include <WProgram.h>
  #endif
#endif

#define SAMD_ISR_Timer SAMD_ISRTimer

typedef void (*timerCallback)();
typedef void (*timerCallback_p)(void *);

class SAMD_ISR_Timer 
{

  public:
    // maximum number of timers
#define MAX_NUMBER_TIMERS         16
#define TIMER_RUN_FOREVER         0
#define TIMER_RUN_ONCE            1

    // constructor
    SAMD_ISR_Timer();

    void init();

    // this function must be called inside loop()
    void run();

    // Timer will call function 'f' every 'd' milliseconds forever
    // returns the timer number (numTimer) on success or
    // -1 on failure (f == NULL) or no free timers
    int setInterval(const unsigned long& d, timerCallback f);

    // Timer will call function 'f' with parameter 'p' every 'd' milliseconds forever
    // returns the timer number (numTimer) on success or
    // -1 on failure (f == NULL) or no free timers
    int setInterval(const unsigned long& d, timerCallback_p f, void* p);

    // Timer will call function 'f' after 'd' milliseconds one time
    // returns the timer number (numTimer) on success or
    // -1 on failure (f == NULL) or no free timers
    int setTimeout(const unsigned long& d, timerCallback f);

    // Timer will call function 'f' with parameter 'p' after 'd' milliseconds one time
    // returns the timer number (numTimer) on success or
    // -1 on failure (f == NULL) or no free timers
    int setTimeout(const unsigned long& d, timerCallback_p f, void* p);

    // Timer will call function 'f' every 'd' milliseconds 'n' times
    // returns the timer number (numTimer) on success or
    // -1 on failure (f == NULL) or no free timers
    int setTimer(const unsigned long& d, timerCallback f, const unsigned& n);

    // Timer will call function 'f' with parameter 'p' every 'd' milliseconds 'n' times
    // returns the timer number (numTimer) on success or
    // -1 on failure (f == NULL) or no free timers
    int setTimer(const unsigned long& d, timerCallback_p f, void* p, const unsigned& n);

    // updates interval of the specified timer
    bool changeInterval(const unsigned& numTimer, const unsigned long& d);

    // destroy the specified timer
    void deleteTimer(const unsigned& numTimer);

    // restart the specified timer
    void restartTimer(const unsigned& numTimer);

    // returns true if the specified timer is enabled
    bool isEnabled(const unsigned& numTimer);

    // enables the specified timer
    void enable(const unsigned& numTimer);

    // disables the specified timer
    void disable(const unsigned& numTimer);

    // enables all timers
    void enableAll();

    // disables all timers
    void disableAll();

    // enables the specified timer if it's currently disabled, and vice-versa
    void toggle(const unsigned& numTimer);

    // returns the number of used timers
    unsigned getNumTimers();

    // returns the number of available timers
    unsigned getNumAvailableTimers() 
    {
      return MAX_NUMBER_TIMERS - numTimers;
    };

  private:
    // deferred call constants
#define TIMER_DEFCALL_DONTRUN   0       // don't call the callback function
#define TIMER_DEFCALL_RUNONLY   1       // call the callback function but don't delete the timer
#define TIMER_DEFCALL_RUNANDDEL 2       // call the callback function and delete the timer

    // low level function to initialize and enable a new timer
    // returns the timer number (numTimer) on success or
    // -1 on failure (f == NULL) or no free timers
    int setupTimer(const unsigned long& d, void* f, void* p, bool h, const unsigned& n);

    // find the first available slot
    int findFirstFreeSlot();

    typedef struct 
    {
      unsigned long prev_millis;        // value returned by the millis() function in the previous run() call
      void*         callback;           // pointer to the callback function
      void*         param;              // function parameter
      bool          hasParam;           // true if callback takes a parameter
      unsigned long delay;              // delay value
      unsigned      maxNumRuns;         // number of runs to be executed
      unsigned      numRuns;            // number of executed runs
      bool          enabled;            // true if enabled
      unsigned      toBeCalled;         // deferred function call (sort of) - N.B.: only used in run()
    } timer_t;

    volatile timer_t timer[MAX_NUMBER_TIMERS];

    // actual number of timers in use (-1 means uninitialized)
    volatile int numTimers;
};

#endif    // ISR_TIMER_GENERIC_HPP
