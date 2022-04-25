/****************************************************************************************************************************
  SAMDTimerInterrupt.hpp
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
  
  Version: 1.7.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      30/10/2020 Initial coding
  1.0.1   K Hoang      06/11/2020 Add complicated example ISR_16_Timers_Array using all 16 independent ISR Timers.
  1.1.1   K.Hoang      06/12/2020 Add Change_Interval example. Bump up version to sync with other TimerInterrupt Libraries
  1.2.0   K.Hoang      08/01/2021 Add better debug feature. Optimize code and examples to reduce RAM usage
  1.3.0   K.Hoang      02/04/2021 Add support to Sparkfun SAMD21 and SAMD51 boards
  1.3.1   K.Hoang      09/05/2021 Fix compile error to some SAMD21-based boards
  1.4.0   K.Hoang      02/06/2021 Fix SAMD21 rare bug caused by not fully init Prescaler
  1.5.0   K.Hoang      08/10/2021 Improve frequency precision by using float instead of ulong
  1.6.0   K.Hoang      20/01/2022 Fix `multiple-definitions` linker error. Add support to many more boards
  1.7.0   K.Hoang      25/04/2022 Optimize code for setInterval() of SAMD21 TC3
*****************************************************************************************************************************/
/*
  SAMD21
  
  The Timer/Counter for Control Applications (TCC) module provides a set of timing and counting related functionality, such as the
  generation of periodic waveforms, the capturing of a periodic waveform's frequency/duty cycle, software timekeeping for periodic
  operations, waveform extension control, fault detection etc.
  The counter size of the TCC modules can be 16- or 24-bit depending on the TCC instance
  
  1) Nano-33-IoT SAMD21G18A
  .arduino15/packages/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS/Device/ATMEL/samd21/include/samd21g18a.h
   #define TC3  ((Tc *)0x42002C00UL)
  
*/
#pragma once

#ifndef SAMD_TIMERINTERRUPT_HPP
#define SAMD_TIMERINTERRUPT_HPP

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

#if ( defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) \
   || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310) || defined(ARDUINO_SAMD_MKRGSM1400) \
   || defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) || defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) \
   || defined(__SAMD21E15A__) || defined(__SAMD21E16A__) || defined(__SAMD21E17A__) || defined(__SAMD21E18A__) \
   || defined(__SAMD21G15A__) || defined(__SAMD21G16A__) || defined(__SAMD21G17A__) || defined(__SAMD21G18A__) \
   || defined(__SAMD21J15A__) || defined(__SAMD21J16A__) || defined(__SAMD21J17A__) || defined(__SAMD21J18A__) || defined(__SAMD21__) )
     
  #define TIMER_INTERRUPT_USING_SAMD21      true
   
  #if !defined(BOARD_NAME)
  
    #if defined(ARDUINO_QWIIC_MICRO)
      #define BOARD_NAME    "Sparkfun SAMD21_QWIIC_MICRO"
      
      #if (_TIMERINTERRUPT_LOGLEVEL_  > 2)
        #warning BOARD_NAME == Sparkfun SAMD21_QWIIC_MICRO
      #endif
    #elif defined(__SAMD21E15A__)
      #define BOARD_NAME    "__SAMD21E15A__"
    #elif defined(__SAMD21E16A__)
      #define BOARD_NAME    "__SAMD21E16A__"
    #elif defined(__SAMD21E17A__)
      #define BOARD_NAME    "__SAMD21E17A__"
    #elif defined(__SAMD21E18A__)
      #define BOARD_NAME    "__SAMD21E18A__"
    #elif defined(__SAMD21G15A__)
      #define BOARD_NAME    "__SAMD21G15A__"
    #elif defined(__SAMD21G16A__)
      #define BOARD_NAME    "__SAMD21G16A__"
    #elif defined(__SAMD21G17A__)
      #define BOARD_NAME    "__SAMD21G17A__"
    #elif defined(__SAMD21G18A__)
      #define BOARD_NAME    "__SAMD21G18A__"
    #elif defined(__SAMD21J15A__)
      #define BOARD_NAME    "__SAMD21J15A__"
    #elif defined(__SAMD21J16A__)
      #define BOARD_NAME    "__SAMD21J16A__"
    #elif defined(__SAMD21J17A__)
      #define BOARD_NAME    "__SAMD21J17A__"
    #elif defined(__SAMD21J18A__)
      #define BOARD_NAME    "__SAMD21J18A__" 
    #else
      #define BOARD_NAME    "Unknown SAMD21"   
    #endif
    
  #endif
  
  #if (_TIMERINTERRUPT_LOGLEVEL_  > 2)
    #warning Using SAMD21 Hardware Timer
  #endif
  
#elif ( defined(__SAMD51__) || defined(__SAMD51J20A__) || defined(__SAMD51J19A__) || defined(__SAMD51G19A__) || defined(__SAMD51P19A__) )
  #define TIMER_INTERRUPT_USING_SAMD51      true
   
  #if !defined(BOARD_NAME)
  
    #if defined(ARDUINO_SAMD51_THING_PLUS)
      #define BOARD_NAME    "Sparkfun SAMD51_THING_PLUS"
      
      #if (_TIMERINTERRUPT_LOGLEVEL_  > 2)
        #warning BOARD_NAME == Sparkfun SAMD51_THING_PLUS
      #endif
      
    #elif defined(ARDUINO_SAMD51_MICROMOD)
      #define BOARD_NAME    "Sparkfun SAMD51_MICROMOD"
      
      #if (_TIMERINTERRUPT_LOGLEVEL_  > 2)
        #warning BOARD_NAME == Sparkfun SAMD51_MICROMOD
      #endif
    #elif defined(__SAMD51J20A__)
      #define BOARD_NAME    "__SAMD51J20A__"
    #elif defined(__SAMD51J19A__)
      #define BOARD_NAME    "__SAMD51J19A__"
    #elif defined(__SAMD51G19A__)
      #define BOARD_NAME    "__SAMD51G19A__"
    #elif defined(__SAMD51P19A__)
      #define BOARD_NAME    "__SAMD51P19A__"      
    #else
      #define BOARD_NAME    "Unknown SAMD51"    
    #endif
    
  #endif
  
  #if (_TIMERINTERRUPT_LOGLEVEL_  > 2)
    #warning Using SAMD51 Hardware Timer
  #endif
#else
  #error Unknown board  
#endif

// Too many boards sharing the same ARDUINO_SAMD_ZERO but very different, such as SAMD21 SparkFun RedBoard Turbo
// Have to exclude some from the list
#if ( defined(ARDUINO_SAMD_ZERO) && ! ( defined(ADAFRUIT_FEATHER_M0) || defined(ARDUINO_SAMD_FEATHER_M0) || defined(ADAFRUIT_METRO_M0_EXPRESS) || \
      defined(ARDUINO_SAMD_HALLOWING_M0) || defined(ADAFRUIT_BLM_BADGE) ) )
  // Specific for SAMD21 SparkFun RedBoard Turbo
  #if !defined(Serial)
    #define Serial    SerialUSB
  #endif
#endif

#include "Arduino.h"

#ifndef SAMD_TIMER_INTERRUPT_VERSION
  #define SAMD_TIMER_INTERRUPT_VERSION            "SAMDTimerInterrupt v1.7.0"
  
  #define SAMD_TIMER_INTERRUPT_VERSION_MAJOR      1
  #define SAMD_TIMER_INTERRUPT_VERSION_MINOR      7
  #define SAMD_TIMER_INTERRUPT_VERSION_PATCH      0

  #define SAMD_TIMER_INTERRUPT_VERSION_INT        1007000
#endif

#include "TimerInterrupt_Generic_Debug.h"

#define TIMER_HZ      48000000L

////////////////////////////////////////////////////

////////////////////////////////////////////////////

#if (TIMER_INTERRUPT_USING_SAMD51)

typedef enum
{
  TIMER_TC3 = 0,
  MAX_TIMER
} SAMDTimerNumber;

class SAMDTimerInterrupt;

typedef SAMDTimerInterrupt SAMDTimer;

typedef void (*timerCallback)  ();

#define SAMD_TC3        ((TcCount16*) _SAMDTimer)

//////////////////////////////////////////

static inline void TC3_wait_for_sync()
{
  while (TC3->COUNT16.SYNCBUSY.reg != 0);
}

/////////////////////////////////////////////////////////////

class SAMDTimerInterrupt
{
  private:
    SAMDTimerNumber _timerNumber;

    // point to timer struct, (TcCount16*) TC3 for SAMD51
    void*           _SAMDTimer = NULL;

    timerCallback   _callback;        // pointer to the callback function

    // _prescaler = (1 << new_prescaler)
    uint8_t         new_prescaler;

    uint16_t        _compareValue;

  public:

    SAMDTimerInterrupt(const SAMDTimerNumber& timerNumber)
    {
      _timerNumber = timerNumber;

      if (_timerNumber == TIMER_TC3)
      {
        _SAMDTimer = (TcCount16*) TC3;
      }

      _callback = NULL;
    };

    //////////////////////////////////////////

    ~SAMDTimerInterrupt()
    {
    }

    //////////////////////////////////////////

    bool setFrequency(const float& frequency, timerCallback callback);
    bool _setPeriod(const float& _period, timerCallback callback);

    //////////////////////////////////////////

    // interval (in microseconds) and duration (in milliseconds). Duration = 0 or not specified => run indefinitely
    // No params and duration now. To be addes in the future by adding similar functions here or to SAMD-hal-timer.c
    bool setInterval(const unsigned long& interval, timerCallback callback)
    {
      return _setPeriod(interval, callback);
    }

    //////////////////////////////////////////

    bool attachInterrupt(const float& frequency, timerCallback callback)
    {
      return setFrequency(frequency, callback);
    }

    //////////////////////////////////////////

    // interval (in microseconds) and duration (in milliseconds). Duration = 0 or not specified => run indefinitely
    // No params and duration now. To be addes in the future by adding similar functions here or to SAMD-hal-timer.c
    bool attachInterruptInterval(const unsigned long& interval, timerCallback callback)
    {
      return _setPeriod(interval, callback);
    }

    //////////////////////////////////////////

    void detachInterrupt()
    {
      // Disable Interrupt
      if (_timerNumber == TIMER_TC3)
      {
        NVIC_DisableIRQ(TC3_IRQn);
      }
    }

    //////////////////////////////////////////

    void disableTimer()
    {
      // Disable Timer
      if (_timerNumber == TIMER_TC3)
      {
        // Disable TC3
        TC3->COUNT16.CTRLA.bit.ENABLE = 0;
      }
    }

    //////////////////////////////////////////

    // Duration (in milliseconds). Duration = 0 or not specified => run indefinitely
    void reattachInterrupt()
    {
      // Disable Interrupt
      if (_timerNumber == TIMER_TC3)
      {
        NVIC_EnableIRQ(TC3_IRQn);
      }
    }

    //////////////////////////////////////////

    // Duration (in milliseconds). Duration = 0 or not specified => run indefinitely
    void enableTimer()
    {
      // Enable Timer
      if (_timerNumber == TIMER_TC3)
      {
        // Enable TC3
        SAMD_TC3->CTRLA.reg |= TC_CTRLA_ENABLE;
      }
    }

    //////////////////////////////////////////

    // Just stop clock source, clear the count
    void stopTimer()
    {
      // TODO, clear the count
      disableTimer();
    }

    //////////////////////////////////////////

    // Just reconnect clock source, start current count from 0
    void restartTimer()
    {
      // TODO, clear the count
      enableTimer();
    }

    /////////////////////////////////////////////////////////////////////////

  private:

    void setPeriod_TIMER_TC3(const float& period)
    {
      uint32_t TC_CTRLA_PRESCALER_DIVN = 1;

      TC3->COUNT16.CTRLA.reg &= ~( TC_CTRLA_ENABLE |  TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_PRESCALER_DIV64 | \
                                   TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_PRESCALER_DIV4 | TC_CTRLA_PRESCALER_DIV2 | TC_CTRLA_PRESCALER_DIV1);
      TC3_wait_for_sync();

      uint8_t old_prescaler = new_prescaler;

      new_prescaler = 0;

      if (period > 300000)
      {
        TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV1024;
        new_prescaler = 10;
      }
      else if (80000 < period && period <= 300000)
      {
        TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV256;
        new_prescaler = 8;
      }
      else if (20000 < period && period <= 80000)
      {
        TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV64;
        new_prescaler = 6;
      }
      else if (10000 < period && period <= 20000)
      {
        TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV16;
        new_prescaler = 4;
      }
      else if (5000 < period && period <= 10000)
      {
        TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV8;
        new_prescaler = 3;
      }
      else if (2500 < period && period <= 5000)
      {
        TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV4;
        new_prescaler = 2;
      }
      else if (1000 < period && period <= 2500)
      {
        TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV2;
        new_prescaler = 1;
      }
      else
      {
        TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV1;
        new_prescaler = 0;
      }

      TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIVN;
      TC3_wait_for_sync();

      _compareValue = (uint16_t)(TIMER_HZ / ((1 << new_prescaler) / (period / 1000000.0))) - 1;

      uint16_t counter = TC3->COUNT16.CTRLA.reg;

      // scale up (or down) counter to match the new prescaler
      // if new prescaler is lower, then scale up counter (but not higher than _compareValue)
      // if new prescaler is higher, then scale down the counter (but not lower than 0)
      if (new_prescaler < old_prescaler)
      {
        counter <<= (old_prescaler - new_prescaler);

        if (counter > _compareValue)
          counter = _compareValue;
      }
      else // new_prescaler > old_prescaler
      {
        counter >>= (new_prescaler - old_prescaler);
      }

      TC3->COUNT16.COUNT.reg = counter;
      TC3_wait_for_sync();

      TC3->COUNT16.CC[0].reg = _compareValue;
      TC3_wait_for_sync();

      TC3->COUNT16.CTRLA.bit.ENABLE = 1;
      TC3_wait_for_sync();

      TISR_LOGDEBUG3(F("SAMD51 TC3 period ="), period, F(", _prescaler ="), 1 << new_prescaler);
      TISR_LOGDEBUG1(F("_compareValue ="), _compareValue);
    }
}; // class SAMDTimerInterrupt


////////////////////////////////////////////////////////////////////

#elif (TIMER_INTERRUPT_USING_SAMD21)

typedef enum
{
  TIMER_TC3 = 0,
  TIMER_TCC = 1,
  MAX_TIMER
} SAMDTimerNumber;

class SAMDTimerInterrupt;

typedef SAMDTimerInterrupt SAMDTimer;

typedef void (*timerCallback)  ();

#define SAMD_TC3        ((TcCount16*) _SAMDTimer)
#define SAMD_TCC        ((Tcc*) _SAMDTimer)

/////////////////////////////////////////////////////////

class SAMDTimerInterrupt
{
  private:
    SAMDTimerNumber _timerNumber;

    // point to timer struct, (TcCount16*) TC3 or (Tcc*) TCC0 for SAMD21
    void*           _SAMDTimer = NULL;

    timerCallback   _callback;        // pointer to the callback function

    uint16_t        _compareValue;

    // _prescaler = (1 << new_prescaler)
    uint8_t         new_prescaler;

    bool initialized;

  public:

    SAMDTimerInterrupt(const SAMDTimerNumber& timerNumber) : initialized(false)
    {
      _timerNumber = timerNumber;

      if (_timerNumber == TIMER_TC3)
      {
        _SAMDTimer = (TcCount16*) TC3;
      }
      else if (_timerNumber == TIMER_TCC)
      {
        _SAMDTimer = (Tcc*) TCC0;
      }

      _callback = NULL;
    };

    //////////////////////////////////////////

    ~SAMDTimerInterrupt()
    {
    }

    //////////////////////////////////////////

    bool setFrequency(const float& frequency, timerCallback callback);
    bool _setPeriod(const float& _period, timerCallback callback);

    //////////////////////////////////////////

    // interval (in microseconds) and duration (in milliseconds). Duration = 0 or not specified => run indefinitely
    // No params and duration now. To be addes in the future by adding similar functions here or to SAMD-hal-timer.c
    bool setInterval(const unsigned long& interval, timerCallback callback)
    {
      return _setPeriod(interval, callback);
    }

    //////////////////////////////////////////

    bool attachInterrupt(const float& frequency, timerCallback callback)
    {
      return setFrequency(frequency, callback);
    }

    //////////////////////////////////////////

    // interval (in microseconds) and duration (in milliseconds). Duration = 0 or not specified => run indefinitely
    // No params and duration now. To be addes in the future by adding similar functions here or to SAMD-hal-timer.c
    bool attachInterruptInterval(const unsigned long& interval, timerCallback callback)
    {
      return _setPeriod(interval, callback);
    }

    //////////////////////////////////////////

    void detachInterrupt()
    {
      // Disable Interrupt
      if (_timerNumber == TIMER_TC3)
      {
        NVIC_DisableIRQ(TC3_IRQn);
      }
      else if (_timerNumber == TIMER_TCC)
      {
        NVIC_DisableIRQ(TCC0_IRQn);
      }
    }

    //////////////////////////////////////////

    void disableTimer()
    {
      // Disable Timer
      if (_timerNumber == TIMER_TC3)
      {
        // Disable TC3
        SAMD_TC3->CTRLA.reg &= ~TC_CTRLA_ENABLE;
        while (SAMD_TC3->STATUS.bit.SYNCBUSY);
      }
      else if (_timerNumber == TIMER_TCC)
      {
        // Disable TCC
        SAMD_TCC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
      }
    }

    //////////////////////////////////////////

    // Duration (in milliseconds). Duration = 0 or not specified => run indefinitely
    void reattachInterrupt()
    {
      // Disable Interrupt
      if (_timerNumber == TIMER_TC3)
      {
        NVIC_EnableIRQ(TC3_IRQn);
      }
      else if (_timerNumber == TIMER_TCC)
      {
        NVIC_EnableIRQ(TCC0_IRQn);
      }
    }

    //////////////////////////////////////////

    // Duration (in milliseconds). Duration = 0 or not specified => run indefinitely
    void enableTimer()
    {
      // Enable Timer
      if (_timerNumber == TIMER_TC3)
      {
        // Enable TC3
        SAMD_TC3->CTRLA.reg |= TC_CTRLA_ENABLE;
        while (SAMD_TC3->STATUS.bit.SYNCBUSY);
      }
      else if (_timerNumber == TIMER_TCC)
      {
        // Enable TCC
        SAMD_TCC->CTRLA.reg |= TCC_CTRLA_ENABLE;
      }
    }

    //////////////////////////////////////////

    // Just stop clock source, clear the count
    void stopTimer()
    {
      // TODO, clear the count
      disableTimer();
    }

    //////////////////////////////////////////

    // Just reconnect clock source, start current count from 0
    void restartTimer()
    {
      // TODO, clear the count
      enableTimer();
    }

    /////////////////////////////////////////////////////////////////////

  private:

    //////////////////////////////////////////

    inline byte getPrescalerBitShift(uint16_t ctrla)
    {
      // prescaler is stored in bits 8 thru 11 of ctrla, so we need to shift 8 bits right and mask off lower 4 bits
      // conveniently, the value is the number of bits to shift i.e. value==0x0A => shift is 10 bits (1024 div)
      return (ctrla >> 8) & 0x000f;
    }

    //////////////////////////////////////////

    void setPeriod_TIMER_TC3(const float& period)
    {
      TcCount16* _Timer = (TcCount16*) TC3;
      uint16_t ctrla    = _Timer->CTRLA.reg;
      bool was_enabled  = (ctrla & TC_CTRLA_ENABLE);

      // get old prescaler from ctrla, convert to bit shift
      uint8_t old_prescaler = getPrescalerBitShift(ctrla);
      uint8_t new_prescaler = 0;

      if (period > 300000)
      {
        // Set prescaler to 1024
        new_prescaler = 10;
      }
      else if (80000 < period && period <= 300000)
      {
        // Set prescaler to 256
        new_prescaler = 8;
      }
      else if (20000 < period && period <= 80000)
      {
        // Set prescaler to 64
        new_prescaler = 6;
      }
      else if (10000 < period && period <= 20000)
      {
        // Set prescaler to 16
        new_prescaler = 4;
      }
      else if (5000 < period && period <= 10000)
      {
        // Set prescaler to 8
        new_prescaler = 3;
      }
      else if (2500 < period && period <= 5000)
      {
        // Set prescaler to 4
        new_prescaler = 2;
      }
      else if (1000 < period && period <= 2500)
      {
        // Set prescaler to 2
        new_prescaler = 1;
      }
      else
      {
        // Set prescaler to 1
        new_prescaler = 0;
      }

      // mask out old prescaler value, and set the new prescaler value
      ctrla = (ctrla & 0xf0ff) | ((new_prescaler) << 8);

      _compareValue = (uint16_t)(TIMER_HZ / ((1 << new_prescaler) / (period / 1000000.0))) - 1;

      if (new_prescaler != old_prescaler && was_enabled)
      {
        // Make sure the count is in a proportional position to where it was
        // to prevent any jitter or disconnect when changing the compare value.
        // But we only need to do this if the precaler changed, AND if the timer was enabled.

        _Timer->READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(0x10); // 0x10 is the offset of the 16-bit count register
        while (_Timer->STATUS.bit.SYNCBUSY);

        uint16_t counter = _Timer->COUNT.reg;

        // cannot update ctrla at the same time as the enabled bit is set
        _Timer->CTRLA.reg &= ~TC_CTRLA_ENABLE;
        while (_Timer->STATUS.bit.SYNCBUSY);

        // need to synchronise the count and cc values
        _Timer->READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(0x18); // 0x18 is the offset of the 16-bit CC0 register
        while (_Timer->STATUS.bit.SYNCBUSY);

        _Timer->CC[0].reg = (uint16_t)(-1); // max, so that changing Count doesn't end up wrapping it back to zero again due to the continuous comparison

        // scale up (or down) counter to match the new prescaler
        // if new prescaler is lower, then scale up counter (but no higher than _compareValue)
        // if new prescaler is higher, then scale down the counter (but no lower than 0)
        if (new_prescaler < old_prescaler)
        {
          counter <<= (old_prescaler - new_prescaler);

          if (counter > _compareValue)
            counter = _compareValue;
        }
        else
        {
          counter >>= (new_prescaler - old_prescaler);
        }

        _Timer->COUNT.reg = counter;
        while (_Timer->STATUS.bit.SYNCBUSY);
      }

      // in all cases, we need to set the (new) CC, and set the (new) prescaler and (re)enable the timer
      _Timer->CC[0].reg = _compareValue;
      while (_Timer->STATUS.bit.SYNCBUSY);

      _Timer->CTRLA.reg = ctrla | TC_CTRLA_ENABLE;
      while (_Timer->STATUS.bit.SYNCBUSY);

      TISR_LOGDEBUG3(F("SAMD21 TC3 period ="), period, F(", _prescaler ="), 1 << new_prescaler);
      TISR_LOGDEBUG1(F("_compareValue ="), _compareValue);
    }

    //////////////////////////////////////////

    void setPeriod_TIMER_TCC(const float& period)
    {
      Tcc* _Timer = (Tcc*) _SAMDTimer;

      _Timer->CTRLA.reg &= ~( TCC_CTRLA_ENABLE |  TCC_CTRLA_PRESCALER_DIV1024 | TCC_CTRLA_PRESCALER_DIV256 | TCC_CTRLA_PRESCALER_DIV64 | \
                              TCC_CTRLA_PRESCALER_DIV16 | TCC_CTRLA_PRESCALER_DIV4 | TCC_CTRLA_PRESCALER_DIV2 | TCC_CTRLA_PRESCALER_DIV1);
      while (_Timer->SYNCBUSY.bit.ENABLE == 1);

      if (period > 300000)
      {
        // Set prescaler to 1024
        _Timer->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1024;
        new_prescaler = 10;
      }
      else if (80000 < period && period <= 300000)
      {
        // Set prescaler to 256
        _Timer->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV256;
        new_prescaler = 8;
      }
      else if (20000 < period && period <= 80000)
      {
        // Set prescaler to 64
        _Timer->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV64;
        new_prescaler = 6;
      }
      else if (10000 < period && period <= 20000)
      {
        // Set prescaler to 16
        _Timer->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV16;
        new_prescaler = 4;
      }
      else if (5000 < period && period <= 10000)
      {
        // Set prescaler to 8
        _Timer->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV8;
        new_prescaler = 3;
      }
      else if (2500 < period && period <= 5000)
      {
        // Set prescaler to 4
        _Timer->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV4;
        new_prescaler = 2;
      }
      else if (1000 < period && period <= 2500)
      {
        // Set prescaler to 2
        _Timer->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV2;
        new_prescaler = 1;
      }
      else
      {
        // Set prescaler to 1
        _Timer->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1;
        new_prescaler = 0;
      }

      _compareValue = (uint16_t)(TIMER_HZ / ((1 << new_prescaler) / (period / 1000000.0))) - 1;

      _Timer->PER.reg = _compareValue;
      while (_Timer->SYNCBUSY.bit.PER == 1);

      _Timer->CC[0].reg = 0xFFF;
      while (_Timer->SYNCBUSY.bit.CC0 == 1);

      TISR_LOGDEBUG3(F("SAMD21 TCC period ="), period, F(", _prescaler ="), (1 << new_prescaler));
      TISR_LOGDEBUG1(F("_compareValue ="), _compareValue);
    }

    //////////////////////////////////////////

}; // class SAMDTimerInterrupt

#endif    // #if (TIMER_INTERRUPT_USING_SAMD51)

#endif    // #ifndef SAMD_TIMERINTERRUPT_HPP
