/****************************************************************************************************************************
  SAMDTimerInterrupt_Impl.h
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

#ifndef SAMD_TIMERINTERRUPT_IMPL_H
#define SAMD_TIMERINTERRUPT_IMPL_H

////////////////////////////////////////////////////

#if (TIMER_INTERRUPT_USING_SAMD51)

  timerCallback TC3_callback;

  //#define SAMD_TC3        ((TcCount16*) _SAMDTimer)
  
////////////////////////////////////////////////////////

  void TC3_Handler() 
  {
    // If this interrupt is due to the compare register matching the timer count
    if (TC3->COUNT16.INTFLAG.bit.MC0 == 1) 
    {
      TC3->COUNT16.INTFLAG.bit.MC0 = 1;
      (*TC3_callback)();
    }
  }
  
////////////////////////////////////////////////////////

  // frequency (in hertz) and duration (in milliseconds). Duration = 0 or not specified => run indefinitely
  // No params and duration now. To be addes in the future by adding similar functions here or to SAMD-hal-timer.c
  bool SAMDTimerInterrupt::setFrequency(const float& frequency, timerCallback callback)
  {
    _period =  (1000000.0f / frequency);
       
    if (_timerNumber == TIMER_TC3)
    {    
      TISR_LOGWARN3(F("SAMDTimerInterrupt: F_CPU (MHz) ="), F_CPU/1000000, F(", TIMER_HZ ="), TIMER_HZ/1000000);
      TISR_LOGWARN3(F("TC_Timer::startTimer _Timer = 0x"), String((uint32_t) _SAMDTimer, HEX), F(", TC3 = 0x"), String((uint32_t) TC3, HEX));
      
      // maxPermittedPeriod = 1,398,101.33us for 48MHz timer clock
      float maxPermittedPeriod = (65536.0f * 1024) / (TIMER_HZ / 1000000.0f );
          
      if (_period > maxPermittedPeriod )
      {
        TISR_LOGERROR3(F("Max permissible _period (us) ="),  maxPermittedPeriod, F(", current _period (us) ="), _period);
        
        return false;
      }

      // Enable the TC bus clock, use clock generator 0
      GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
      
      while (GCLK->SYNCBUSY.reg > 0);

      TC3->COUNT16.CTRLA.bit.ENABLE = 0;
      
      // Use match mode so that the timer counter resets when the count matches the
      // compare register
      TC3->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
      TC3_wait_for_sync();
      
       // Enable the compare interrupt
      TC3->COUNT16.INTENSET.reg = 0;
      TC3->COUNT16.INTENSET.bit.MC0 = 1;

      // Enable IRQ
      NVIC_EnableIRQ(TC3_IRQn);

      _callback     = callback;
      TC3_callback  = callback;

      setPeriod_TIMER_TC3(_period);
           
      return true;
    }
    else
      return false;
  }

///////////////////////////////////////////////////////////////////////////////////////

#elif (TIMER_INTERRUPT_USING_SAMD21)

  static uint16_t TC_GCLK_ID[MAX_TIMER] = { GCM_TCC2_TC3, GCM_TC4_TC5, GCM_TC4_TC5, GCM_TCC0_TCC1, GCM_TCC0_TCC1, GCM_TCC2_TC3 };
  
////////////////////////////////////////////////////////

  timerCallback TC3_callback;  
  timerCallback TCC_callback;
  timerCallback TCC1_callback;
  timerCallback TCC2_callback;
  timerCallback TC4_callback;
  timerCallback TC5_callback;
  
////////////////////////////////////////////////////////

#if USING_TIMER_TC3
  #warning USING_TIMER_TC3
  void TC3_Handler()
  {
    // get timer struct
    TcCount16* TC = (TcCount16*) TC3;
    
    // If the compare register matching the timer count, trigger this interrupt
    if (TC->INTFLAG.bit.MC0 == 1) 
    {
      TC->INTFLAG.bit.MC0 = 1;
      (*TC3_callback)();
    }
  }
#endif
  
////////////////////////////////////////////////////////

#if USING_TIMER_TC4
  #warning USING_TIMER_TC4
  void TC4_Handler()
  {
    // get timer struct
    TcCount16* TC = (TcCount16*) TC4;
    
    // If the compare register matching the timer count, trigger this interrupt
    if (TC->INTFLAG.bit.MC0 == 1) 
    {
      TC->INTFLAG.bit.MC0 = 1;
      (*TC4_callback)();
    }
  }
#endif
  
////////////////////////////////////////////////////////

#if USING_TIMER_TC5
  #warning USING_TIMER_TC5 
  void TC5_Handler()
  {
    // get timer struct
    TcCount16* TC = (TcCount16*) TC5;
    
    // If the compare register matching the timer count, trigger this interrupt
    if (TC->INTFLAG.bit.MC0 == 1) 
    {
      TC->INTFLAG.bit.MC0 = 1;
      (*TC5_callback)();
    }
  }
#endif
  
////////////////////////////////////////////////////////

#if USING_TIMER_TCC
  #warning USING_TIMER_TCC
  void TCC0_Handler()
  {
    // get timer struct
    Tcc* TC = (Tcc*) TCC0;
    
    // If the compare register matching the timer count, trigger this interrupt
    if (TC->INTFLAG.bit.MC0 == 1) 
    {  
      // A compare to cc0 caused the interrupt
      TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
    }

    if (TC->INTFLAG.bit.OVF == 1) 
    {
      (*TCC_callback)();
      
      TC->INTFLAG.bit.OVF = 1;
    }
  }
#endif
  
////////////////////////////////////////////////////////

#if USING_TIMER_TCC1
  #warning USING_TIMER_TCC1  
  void TCC1_Handler()
  {
    // get timer struct
    Tcc* TC = (Tcc*) TCC1;
    
    // If the compare register matching the timer count, trigger this interrupt
    if (TC->INTFLAG.bit.MC0 == 1) 
    {  
      // A compare to cc0 caused the interrupt
      TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
    }

    if (TC->INTFLAG.bit.OVF == 1) 
    {
      (*TCC1_callback)();
      
      TC->INTFLAG.bit.OVF = 1;
    }
  }
#endif
  
////////////////////////////////////////////////////////

#if USING_TIMER_TCC2
  #warning USING_TIMER_TCC2  
  void TCC2_Handler()
  {
    // get timer struct
    Tcc* TC = (Tcc*) TCC2;
    
    // If the compare register matching the timer count, trigger this interrupt
    if (TC->INTFLAG.bit.MC0 == 1) 
    {  
      // A compare to cc0 caused the interrupt
      TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
    }

    if (TC->INTFLAG.bit.OVF == 1) 
    {
      (*TCC2_callback)();
      
      TC->INTFLAG.bit.OVF = 1;
    }
  }
#endif

///////////////////////////////////////////////////////////////////////////////
     
  // frequency (in hertz) and duration (in milliseconds). Duration = 0 or not specified => run indefinitely
  // No params and duration now. To be addes in the future by adding similar functions here or to SAMD-hal-timer.c
  bool SAMDTimerInterrupt::setFrequency(const float& frequency, timerCallback callback)
  {
    _period =  (1000000.0f / frequency);
    
    TISR_LOGDEBUG3(F("_period ="), _period, F(", frequency ="), frequency);
       
    if ( (_timerNumber == TIMER_TC3) || (_timerNumber == TIMER_TC4) || (_timerNumber == TIMER_TC5) )
    {    
      TISR_LOGDEBUG1(F("_timerNumber ="), _timerNumber);
      
      // maxPermittedPeriod = 1,398,101.33us for 48MHz timer clock
      float maxPermittedPeriod = (65536.0f * 1024) / (TIMER_HZ / 1000000.0f );
      
      if (_period > maxPermittedPeriod )
      {
        TISR_LOGERROR3(F("Max permissible _period (us) ="),  maxPermittedPeriod, F(", current _period (us) ="), _period);
        
        return false;
      }
         
      REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID (TC_GCLK_ID[_timerNumber]));
      
      while ( GCLK->STATUS.bit.SYNCBUSY == 1 );
      
      TISR_LOGWARN3(F("SAMDTimerInterrupt: F_CPU (MHz) ="), F_CPU/1000000, F(", TIMER_HZ ="), TIMER_HZ/1000000);
      
      if (_timerNumber == TIMER_TC3)
      {
        TISR_LOGWARN3(F("TC3_Timer::startTimer _Timer = 0x"), String((uint32_t) _SAMDTimer, HEX), F(", TC3 = 0x"), String((uint32_t) TC3, HEX));
      }
      else if (_timerNumber == TIMER_TC4)
      {
        TISR_LOGWARN3(F("TC4_Timer::startTimer _Timer = 0x"), String((uint32_t) _SAMDTimer, HEX), F(", TC4 = 0x"), String((uint32_t) TC4, HEX));
      }
      else if (_timerNumber == TIMER_TC5)
      {
        TISR_LOGWARN3(F("TC5_Timer::startTimer _Timer = 0x"), String((uint32_t) _SAMDTimer, HEX), F(", TC5 = 0x"), String((uint32_t) TC5, HEX));  
      }
        
      SAMD_TC3->CTRLA.reg &= ~TC_CTRLA_ENABLE;

      // Use the 16-bit timer
      SAMD_TC3->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
      
      while (SAMD_TC3->STATUS.bit.SYNCBUSY == 1);

      // Use match mode so that the timer counter resets when the count matches the compare register
      SAMD_TC3->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
      
      while (SAMD_TC3->STATUS.bit.SYNCBUSY == 1);
  
      setPeriod_TIMER_TC3(_period);

      // Enable the compare interrupt
      SAMD_TC3->INTENSET.reg = 0;
      SAMD_TC3->INTENSET.bit.MC0 = 1;

      NVIC_EnableIRQ(TIMER_IRQ[_timerNumber]);

      SAMD_TC3->CTRLA.reg |= TC_CTRLA_ENABLE;
      
      while (SAMD_TC3->STATUS.bit.SYNCBUSY == 1);

      _callback     = callback;
      
      if (_timerNumber == TIMER_TC3)
        TC3_callback  = callback;
      else if (_timerNumber == TIMER_TC4)
        TC4_callback  = callback;
      else if (_timerNumber == TIMER_TC5)
        TC5_callback  = callback;            
    }
    else if ( (_timerNumber == TIMER_TCC) ||(_timerNumber == TIMER_TCC1) || (_timerNumber == TIMER_TCC2) )
    {
      REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(TC_GCLK_ID[_timerNumber]));
    
      while ( GCLK->STATUS.bit.SYNCBUSY == 1 );
      
      TISR_LOGWARN3(F("SAMDTimerInterrupt: F_CPU (MHz) ="), F_CPU/1000000, F(", TIMER_HZ ="), TIMER_HZ/1000000);
      
      if (_timerNumber == TIMER_TCC)
      {
        TISR_LOGWARN3(F("TCC_Timer::startTimer _Timer = 0x"), String((uint32_t) _SAMDTimer, HEX), F(", TCC0 = 0x"), String((uint32_t) TCC0, HEX));
      }
      else if (_timerNumber == TIMER_TCC1)
      {
        TISR_LOGWARN3(F("TCC_Timer::startTimer _Timer = 0x"), String((uint32_t) _SAMDTimer, HEX), F(", TCC1 = 0x"), String((uint32_t) TCC1, HEX));
      }
      else if (_timerNumber == TIMER_TCC2)
      {
        TISR_LOGWARN3(F("TCC_Timer::startTimer _Timer = 0x"), String((uint32_t) _SAMDTimer, HEX), F(", TCC2 = 0x"), String((uint32_t) TCC2, HEX));
      }
     
      SAMD_TCC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TC
      
      while (SAMD_TCC->SYNCBUSY.bit.ENABLE == 1); // wait for sync 
            
      setPeriod_TIMER_TCC(_period);

      // Use match mode so that the timer counter resets when the count matches the compare register
      SAMD_TCC->WAVE.reg |= TCC_WAVE_WAVEGEN_NFRQ;   // Set wave form configuration 
      
      while (SAMD_TCC->SYNCBUSY.bit.WAVE == 1); // wait for sync 

      // Enable the compare interrupt
      SAMD_TCC->INTENSET.reg = 0;
      SAMD_TCC->INTENSET.bit.OVF = 1;
      SAMD_TCC->INTENSET.bit.MC0 = 1;

      NVIC_EnableIRQ(TIMER_IRQ[_timerNumber]);

      SAMD_TCC->CTRLA.reg |= TCC_CTRLA_ENABLE;
      
      while (SAMD_TCC->SYNCBUSY.bit.ENABLE == 1); // wait for sync 

      _callback     = callback;
      
      if (_timerNumber == TIMER_TCC)
        TCC_callback  = callback;
      else if (_timerNumber == TIMER_TCC1)
        TCC1_callback  = callback;
      else if (_timerNumber == TIMER_TCC2)
        TCC2_callback  = callback;
    }
  
    return true;
  }

#endif    // #if (TIMER_INTERRUPT_USING_SAMD51)

#endif    // SAMD_TIMERINTERRUPT_IMPL_H
