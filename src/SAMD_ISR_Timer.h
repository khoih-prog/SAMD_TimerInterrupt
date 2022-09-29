/****************************************************************************************************************************
  SAMD_ISR_Timer.h
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

#ifndef ISR_TIMER_GENERIC_H
#define ISR_TIMER_GENERIC_H

#include "SAMD_ISR_Timer.hpp"
#include "SAMD_ISR_Timer-Impl.h"

#endif    // ISR_TIMER_GENERIC_H
