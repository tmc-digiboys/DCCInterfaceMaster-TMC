#include <Arduino.h>

#define TMC

// Create a define for all DxCore variants, to improve readability
#if defined(__AVR_DA__) || defined(__AVR_DB__) || defined(__AVR_DD__) || \
    defined(__AVR_EA__) || defined(__AVR_EB__)
  #define AVR_DXCORE
#endif

// Same for the ATMega Timer1 support
#if defined(__AVR_ATmega328P__)  || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega1280__) || defined(__AVR_ATmega32U4__) || \
    defined(__AVR_ATmega644P__)  || defined(__AVR_ATmega644PA__)

  #define ATMEGA_HAS_TIMER1
#endif

// ================= TMC =================
#if defined(TMC)
  #if defined(ARDUINO_ARCH_RP2040)
    #include "variants-TMC/DCCHardware_RP.inc"
  // Others
  #else
    #error No DCCHardware implementation for this processor / board (TMC mode)
  #endif


// ================= Z21PG =================
#elif defined(Z21PG)

  // AVR DxCore
  #if defined(AVR_DXCORE)
    #include "variants-Z21PG/DCCHardware_dxcore_sw_tcb1.inc"

  // Traditional AVRs
  #elif defined(ATMEGA_HAS_TIMER1)
    #include "variants-Z21PG/DCCHardware_AtMega_sw_timer1.inc"

  // ESP32 (all)
  #elif defined(ARDUINO_ARCH_ESP32)
    #include "variants-Z21PG/DCCHardware_ESP32.inc"
    //#include "variants-Z21PG/DCCHardware_ESP32S.inc"

  // ESP8266 / ESP8285
  #elif defined(ARDUINO_ARCH_ESP8266)
    #include "variants-Z21PG/DCCHardware_ESP8266.inc"

  // RP2040 / RP2350 (PIO + DMA)
  #elif defined(ARDUINO_ARCH_RP2040)
    #include "variants-Z21PG/DCCHardware_RP.inc"

  // STM32
  #elif defined(ARDUINO_ARCH_STM32)
    #include "variants-Z21PG/DCCHardware_stm32F4H7xx.inc"

  // Others
  #else
    #error No DCCHardware implementation for this processor / board (Z21PG mode)
  #endif



// ================= HQ =================
#else

  // AVR DxCore
  #if defined(AVR_DXCORE)
    #include "variants-HQ/DCCHardware-DxCore.inc"

  // ESP32 (all)
  #elif defined(ARDUINO_ARCH_ESP32)
    #include "variants-HQ/DCCHardware-ESP32-RMT.inc"

  // RP2040 / RP2350 (PIO + DMA)
  #elif defined(ARDUINO_ARCH_RP2040)
    #include "variants-HQ/DCCHardware-RP-PICO.inc"

  // STM32
  #elif defined(ARDUINO_ARCH_STM32)
    #include "variants-HQ/DCCHardware-STM32-Generic.inc"

  // Others
  #else
    #error No DCCHardware implementation for this processor / board (HQ mode)
  #endif

#endif
