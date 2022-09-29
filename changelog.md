# SAMD_TimerInterrupt Library

[![arduino-library-badge](https://www.ardu-badge.com/badge/SAMD_TimerInterrupt.svg?)](https://www.ardu-badge.com/SAMD_TimerInterrupt)
[![GitHub release](https://img.shields.io/github/release/khoih-prog/SAMD_TimerInterrupt.svg)](https://github.com/khoih-prog/SAMD_TimerInterrupt/releases)
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/khoih-prog/SAMD_TimerInterrupt/blob/master/LICENSE)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](#Contributing)
[![GitHub issues](https://img.shields.io/github/issues/khoih-prog/SAMD_TimerInterrupt.svg)](http://github.com/khoih-prog/SAMD_TimerInterrupt/issues)

---
---

## Table of Contents

* [Changelog](#changelog)
  * [Releases v1.10.1](#releases-v1101)
  * [Releases v1.10.0](#releases-v1100)
  * [Releases v1.9.0](#releases-v190)
  * [Releases v1.8.0](#releases-v180)
  * [Releases v1.7.0](#releases-v170)
  * [Releases v1.6.0](#releases-v160)
  * [Releases v1.5.0](#releases-v150)
  * [Releases v1.4.0](#releases-v140)
  * [Releases v1.3.1](#releases-v131)
  * [Releases v1.3.0](#releases-v130)
  * [Releases v1.2.0](#releases-v120)
  * [Releases v1.1.1](#releases-v111)
  * [Releases v1.0.1](#releases-v101)
  * [Releases v1.0.0](#releases-v100)


---
---

## Changelog

### Releases v1.10.1

1. Using float instead of ulong for interval
2. Prevent overflow of SAMD51 TCx by flagging error

### Releases v1.10.0

1. Avoid conflict with Servo library. Check [Cannot use TimerInterrupt_Generic Library in the same time than Servo Library #11](https://github.com/khoih-prog/TimerInterrupt_Generic/discussions/11)
2. Prevent overflow of SAMD21 TCx by flagging error
3. Modify all examples
4. Update `Packages_Patches`

### Releases v1.9.0

1. Add TC4, TC5, TCC1 and TCC2 Timers to SAMD21
2. Add example [SAMD21_MultiTimers](examples/SAMD21_MultiTimers) to demo the how to use all 6 SAMD21 timers simultaneously.
3. Add functions `attachInterruptInterval_MS()` and `setInterval_MS()`
4. Rewrite examples to take advantage of new functions and timers

### Releases v1.8.0

1. Fix bug introduced in v1.7.0 to SAMD21 TC3. Check [Do I have a brick? I'm unable to upload sketches after using this library! #21](https://github.com/khoih-prog/SAMD_TimerInterrupt/discussions/21)

### Releases v1.7.0

1. Optimize code for setInterval() of SAMD21 TC3. Check [setInterval on a running timer results in a period significantly longer than the specified period #17](https://github.com/khoih-prog/SAMD_TimerInterrupt/issues/17)
2. Update `Packages_Patches`

### Releases v1.6.0

1. Fix `multiple-definitions` linker error. Drop `src_cpp` and `src_h` directories
2. Add example [multiFileProject](examples/multiFileProject) to demo for multiple-file project.
3. Add support to many more boards, such as `SAMD21E1xA`, `SAMD21G1xA` and`SAMD21J1xA`
4. Optimize library code by using `reference-passing` instead of `value-passing`
5. Update all examples
6. Update `Packages_Patches`

### Releases v1.5.0

1. Improve frequency precision by using float instead of ulong, Check PR [change variable period from unsigned long to float #7](https://github.com/khoih-prog/SAMD_TimerInterrupt/pull/7)
2. Remove compiler warnings
3. Update `Packages' Patches`
4. Add `strict` option for PIO `lib_compat_mode`
5. Split `changelog.log` from `README.md`

### Releases v1.4.0

1. Fix SAMD21 rare bug caused by not fully init Prescaler. Check [**Bug when going from a >20000us period to a <20000us period. The timer period become 4 times greater.** #3](https://github.com/khoih-prog/SAMD_TimerInterrupt/issues/3)


### Releases v1.3.1

1. Fix compile error to some SAMD21-based boards, such as ADAFRUIT_FEATHER_M0, ARDUINO_SAMD_FEATHER_M0, ADAFRUIT_METRO_M0_EXPRESS, ARDUINO_SAMD_HALLOWING_M0 and ADAFRUIT_BLM_BADGE. Check [Doesn't compile with Adafruit Feather M0 #2](https://github.com/khoih-prog/SAMD_TimerInterrupt/issues/2).


### Releases v1.3.0

1. Add support to **Sparkfun SAMD21 boards** such as **SparkFun_RedBoard_Turbo, SparkFun_Qwiic_Micro, etc.**
2. Add support to **Sparkfun SAMD51 boards** such as **SparkFun_SAMD51_Thing_Plus, SparkFun_SAMD51_MicroMod, etc.**
3. Update examples to support Sparkfun boards.

### Releases v1.2.0

1. Add better debug feature.
2. Optimize code and examples to reduce RAM usage
3. Add Table of Contents

### Releases v1.1.1

1. Add example [**Change_Interval**](examples/Change_Interval) and [**ISR_16_Timers_Array_Complex**](examples/ISR_16_Timers_Array_Complex)
2. Bump up version to sync with other TimerInterrupt Libraries. Modify Version String.

### Releases v1.0.1

1. Add complicated example [ISR_16_Timers_Array](examples/ISR_16_Timers_Array) utilizing and demonstrating the full usage of 16 independent ISR Timers.

### Releases v1.0.0

1. Permit up to 16 super-long-time, super-accurate ISR-based timers to avoid being blocked
2. Using cpp code besides Impl.h code to use if Multiple-Definition linker error.


