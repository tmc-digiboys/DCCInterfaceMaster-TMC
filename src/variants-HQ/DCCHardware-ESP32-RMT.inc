//************************************************************************************************
//
// File:            DCCHardware-ESP32-RMT.inc
// Purpose:         DCC Waveform Generator for ESP32 processors (RMT)
// Copyright 2026:  Aiko Pras
// History:         2026-02-14 / Aiko Pras - First version
//
//
// RMT-based DCC transmission architecture (overview)
// --------------------------------------------------
//
// The ESP32 RMT peripheral is in principle very well suited for generating
// DCC waveforms: it provides deterministic microsecond timing, hardware-level
// bit generation, and minimal CPU load once transmission has started.
//
// To achieve continuous DCC output with minimal gaps, this module uses
// two transmit buffers ("ping-pong buffers") associated with a single RMT
// channel configured with trans_queue_depth = 2.
//
// While one buffer is being transmitted by RMT, the second buffer is already
// handed to the RMT driver via a second rmt_transmit() call, so that in theory
// RMT should be able to switch seamlessly from the first to the second buffer.
//
// Even with two buffers and back-to-back rmt_transmit() calls, the ESP-IDF RMT
// driver introduces an unavoidable inter-frame delay of roughly 23 µs
// between two separate transmissions. This code compensates for that.
//
// Unlike the typical OpenRemise / ZIMO approach, where the DCC encoder is
// integrated directly into the RMT driver and invoked dynamically as the FIFO
// empties, this implementation translates the entire DCC packet into RMT
// symbols ahead of time.
//
// By doing so:
// - Messages larger than the hardware FIFO (64 symbols) can be transmitted
//   without the encoder having to worry regarding FIFO fill state.
// - The actual CPU-intensive work of converting DCC bits to RMT symbols is
//   already done before rmt_transmit() is called, resulting in shorter gaps
//   (23 µs versus 31/34) between consecutive DCC packets.
// - The copy_encoder used by the RMT driver remains small and simple, because
//   it only needs to stream precomputed symbols into the FIFO rather than
//   compute them on the fly.
//
// This design minimizes inter-frame gaps and CPU overhead, while keeping the
// packet preparation logic conceptually relatively simple.
//
// Synchronization with the application:
// -------------------------------------
// The public interface is synchronous and flag-based. The application calls
// newPacket() only when canAcceptPacket is true. Internally, user data is
// first copied into a staging buffer and later transferred to one of the two
// RMT buffers. This prevents the application from modifying memory that RMT
// is still using.
//
// Why the ISR + FreeRTOS task chain exists:
// ----------------------------------------
// RMT signals the end of a transmission via a TX-done interrupt. The driver
// calls rmtDoneIsr() in ISR context. Because rmt_transmit(), memcpy(), and
// buffer management are NOT ISR-safe (they may block), no application logic
// can run in the ISR.
//
// Therefore:
//   1) rmtDoneIsr() only notifies a FreeRTOS task.
//   2) rmtTxCompletionTask() runs in normal task context and performs:
//      - copying staged user data into the next RMT buffer,
//      - starting the next rmt_transmit(),
//      - setting canAcceptPacket = true for the application.
//
// This indirection is necessary to remain safe, portable, and compatible with
// the ESP-IDF driver model.
//
// Moving from bits towards ESP32 symbols
// --------------------------------------
// In principle we could write our own "encoder", which is called as part of
// the rmt_transmit() call. However, that approach takes too much CPU time
// within that call, and results into longer interpacket gaps.
// Instead we calculate the symbols BEFORE we call rmt_transmit, and use within
// rmt_transmit only the simple "copy_encoder.
//
//
// Portabilty
// ----------
// It should be relatively easy to port this code to other processors. The parts C and D must be
// compltely replaced; the other parts should require miinor changes.
//
//************************************************************************************************
#include <Arduino.h>
#include "../DCCHardware.h"
#include "driver/rmt_tx.h"                 // RMT is used for the normal DCC output signal
#include "driver/rmt_types.h"
#include "driver/gptimer.h"                // A GP Timer is used for the Railcom gap timer


//================================================================================================
//                                      Defines
//================================================================================================
// The values for RMT_START_OFFSET and GPT_OFFSET were determined empirically on an ESP32-WROOM
// module. Measurements were taken at 240 MHz and 160 MHz. Our goal was to set the second half of
// the end bit to 55 µs, which corresponds to the lower threshold specified by the RCN210 standard.
// Due to timing jitter, this 55 µs interval can increase but will not decrease, providing us 6 µs
// of headroom. If this jitter margin proves insufficient, increasing the preamble length may help.
// An unexpected behavior was observed: when the AUX pin generates a RailCom gap, the effective
// delay increases. The cause of this effect is currently unclear.
#if F_CPU == 240000000
  #define RMT_START_OFFSET 25         // Time the RMT needs to output the first symbol (in us)
  #define GPT_OFFSET 7                // Compensation for the GP Timer
#elif F_CPU == 160000000
  #define RMT_START_OFFSET 36
  #define GPT_OFFSET 10
#else
  #define RMT_START_OFFSET ((25UL * 240UL) / (F_CPU / 1000000UL))
  #define GPT_OFFSET       ((7UL  * 240UL) / (F_CPU / 1000000UL))
#endif

#define TCS  ( 29 + GPT_OFFSET)     // RCN-217: cutout start   (26..32 us)
#define TCE  (463 + GPT_OFFSET)     // RCN-217: cutout end     (454..488 us)
#define TTS1 ( 80 + GPT_OFFSET)     // RCN-217: start channel 1 (Unused by the current code)
#define TTS2 (193 + GPT_OFFSET)     // RCN-217: start channel 2 (Unused by the current code)

#define MAX_SYMBOLS 128             // Long enough to hold a 6 byte DCC packet

#define RAILCOM_CUTOUT_LENGTH 4     // see RCN-217 (Ausgabe 24.11.2025)


//================================================================================================
//                            A: Objects and Internal Data Structures
//================================================================================================

//************************************************************************************************
// A.1. Constructor definition and instantiation
//************************************************************************************************
DccPacketEngine::DccPacketEngine(){
  dccRailPin = 11;                                // Pin for DCC rail Signal, with RailCom support
  dccRailAuxPin = 13;                             // Pin for Enable and RailCom gap (optional)
  dccMonitorPin = 0xFF;                           // Not use in this HQ variant
}

// dccPacketEngine is the object that controls packet transmission for different hardware
DccPacketEngine dccPacketEngine;


//************************************************************************************************
// A.2. Declaration and initialisation of internal data structures
//************************************************************************************************
using dcc_waveform_symbol_t = rmt_symbol_word_t;    // We use RMT symbols
using aux_pin_t = gpio_num_t;                       // We use RSP32 HPIO

struct DccPacket {
  dcc_waveform_symbol_t symbols[MAX_SYMBOLS];
  size_t numberOfSymbols;
};

struct DccSymbols {
  dcc_waveform_symbol_t one;    // DCC "1"
  dcc_waveform_symbol_t zero;   // DCC "0"
  dcc_waveform_symbol_t endbit; // End bit
  dcc_waveform_symbol_t gap;    // Railcom gap
};
static DccSymbols s;            // Short name, to improve readability in (inline) helper functions


struct DccTxEngine {                      // Processor independent
  bool isInitialized = false;             // Flag to ensure we initialise only once
  // Aux pin
  aux_pin_t auxPin;                       // Pin for the Power ON/OFF / RailCom cutout
  bool auxPinConfigured;                  // To protect against invalid pin values
  bool auxEnableLevel = 0;                // 0..1: The value of the AUX pin once it is enabled
  bool auxOutputActive = false;           // Flag used by RunOutputSignal / StopOutputSignal
  // Prepared packets
  DccPacket userPacket;                   // Placeholder for the user packet
  DccPacket idlePacket;                   // Precomputed idle packet
  DccPacket resetPacket;                  // Precomputed reset packet
  DccPacket* defaultPacket;               // Pointer ot either an idle or a reset packet
  // Railcom
  bool railComAllowedOnDcc = false;       // DCC pin: should a RC gap be generated in the DCC signal?
  bool railComAllowedOnAux = false;       // AUX pin: is this pin (also) used for the RailCom gap?
  volatile bool railComActiveOnAux;       // Timer ISR: should a RC gap be generated in the Aux signal
  volatile bool railComCutoutInProgress;  // Timer ISR: set flag at RC gap start, clear at RC gap end
  // Service Mode
  volatile bool smActive;                 // SM active (long preamble, resets, SM flow allowed)
  volatile bool isServiceModePacket;      // We have to send Service Mode Packets
  volatile uint8_t smRepeatsRemaining;    // Number of retransmissions left for this SM packet
  volatile uint8_t smMaxRepeats = 10;     // Number of SM retransmissions after first send
  // Preamble length
  volatile uint8_t preambleLength = 17;   // Length of the preamble in normal operation
  volatile uint8_t preambleLengthSM = 20; // Length of the preamble in Service Mode
  // Polarity of the DCC signal
  bool dccSignalInverted = false;
};

struct EspTxEngine {                      // Processor dependent
  // RMT hardware
  rmt_channel_handle_t channel;           // The RMT channel handle
  rmt_encoder_handle_t encoder;           // The RMT encoder handle
  rmt_transmit_config_t config;           // The RMT configuration
  // FreeRTOS callback
  TaskHandle_t rmtCompletionTaskHandle;   // The FreeRTOS task handle
  // The RMT hardware switches between two ping-pong queues with RMT symbols
  dcc_waveform_symbol_t rmtPingPongBuf[2][MAX_SYMBOLS];
  uint8_t currentBufferIndex;             // 0 or 1
};

// Object instantiation
static DccTxEngine dccTx;                     // This object holds generic data
static EspTxEngine espTx;                     // This object holds ESP32 specific data
static gptimer_handle_t railComTimer = NULL;  // ESP32 uses a GP Timer object for RailCom


// Note that we do not (re)set auxEnableLevel, railComAllowedOnDcc, railComAllowedOnAux,
// smMaxRepeats, preambleLength, preambleLengthSM and dccSignalInverted,
// since the user may already have set these values.
static void initTxEngines() {
  // Aux pin state / Railcom
  dccTx.railComActiveOnAux = dccTx.railComAllowedOnAux;
  dccTx.railComCutoutInProgress = false;      // railComCutoutInProgress set / clear: timer ISR
  // Service Mode settings
  dccTx.smActive = false;
  dccTx.isServiceModePacket = false;
  dccTx.smRepeatsRemaining = 0;
  // Clear the user packet
  dccTx.userPacket.numberOfSymbols = 0;
  // Initialise the default packet
  dccTx.defaultPacket = &dccTx.idlePacket;
  // Reset RMT packet engine state
  espTx.currentBufferIndex = 0;               // Ping-pong buffer starts with 0
}


//================================================================================================
//                      B: Helper Functions for DCC Bits and Aux Pin
//================================================================================================

//************************************************************************************************
// B1: Aux pin
//************************************************************************************************
// The dccRailAuxPin serves two related purposes: when enabled via RunOutputSignal(), it
// forces the Aux pin to the active level to enable an H-Bridge, but it can also be
// automatically toggled during a RailCom cutout to generate the RailCom gap.
static inline void enableAuxOutput() {
  if (dccTx.auxPinConfigured) gpio_set_level(dccTx.auxPin, dccTx.auxEnableLevel);
}

static inline void disableAuxOutput() {
  if (dccTx.auxPinConfigured) gpio_set_level(dccTx.auxPin, !dccTx.auxEnableLevel);
}

static inline void initAuxPin() {
  // The AUX pin wil not be used if its value is an invalid pin (such as 0xFF)
  // If the pin is valid, it starts with the opposite of the dccTx.auxEnableLevel
  // This variable may be changed via a setter function.
  if (GPIO_IS_VALID_OUTPUT_GPIO(dccPacketEngine.dccRailAuxPin)) {
    dccTx.auxPinConfigured = true;
    dccTx.auxPin = static_cast<aux_pin_t>(dccPacketEngine.dccRailAuxPin);
    gpio_set_direction(dccTx.auxPin, GPIO_MODE_OUTPUT);
    disableAuxOutput();     // default off.
  }
  else {
    dccTx.auxPinConfigured = false;
    dccPacketEngine.dccRailAuxPin = 0xFF;
  }
}


//************************************************************************************************
// B.2 Translating DCC bytes into RMT symbols
//************************************************************************************************
// This part translates a DCC byte array into a RMT symbol structure.
static void createDccBits() {
  // DCC "1"
  s.one.level0 = 1;
  s.one.duration0 = 58;
  s.one.level1 = 0;
  s.one.duration1 = 58;
  // DCC "0"
  s.zero.level0 = 1;
  s.zero.duration0 = 100;
  s.zero.level1 = 0;
  s.zero.duration1 = 100;
  // EndBit
  s.endbit.level0 = 1;
  s.endbit.duration0 = 58;
  s.endbit.level1 = 0;
  s.endbit.duration1 = 58 - RMT_START_OFFSET; // The RMT_START_OFFSET is determined via measurements
  // Railcom gap
  s.gap.level0 = 1;
  s.gap.duration0 = 29;
  s.gap.level1 = 0;
  s.gap.duration1 = 439;
};


// Helper functions to push a specific RMT symbol on the buffer
static void pushStartBit(dcc_waveform_symbol_t *buf, int &i) { buf[i++] = s.zero; }
static void pushEndBit(dcc_waveform_symbol_t *buf, int &i) { buf[i++] = s.endbit; }
static void pushRailComGap(dcc_waveform_symbol_t *buf, int &i) { buf[i++] = s.gap; }
static void pushBit(dcc_waveform_symbol_t *buf, int &i, bool bit) { buf[i++] = bit ? s.one : s.zero;}
static void pushByte(dcc_waveform_symbol_t *buf, int &i, uint8_t value) {
  for (int b = 7; b >= 0; b--) { pushBit(buf, i, value & (1 << b)); }
}


static void pushPreamble(dcc_waveform_symbol_t *buf, int &i) {
  if (dccTx.smActive) {
    // Service mode has a longer preamble and no Railcum gap
    for (int p = 0; p < dccTx.preambleLengthSM; p++) pushBit(buf, i, 1);
    }
  else {
    // Normal mode.
    int length = dccTx.preambleLength;
    if (dccTx.railComAllowedOnDcc) {             // We should generate a railcom gap
      pushRailComGap(buf, i);
      length = length - 4;
      for (int p = 0; p < length; p++) pushBit(buf, i, 1);
    }
    else for (int p = 0; p < length; p++) pushBit(buf, i, 1);
  }
}

static void prepareIdlePacket() {
  int i = 0;                                  // counter within the buffer
  pushPreamble(dccTx.idlePacket.symbols, i);
  pushStartBit(dccTx.idlePacket.symbols, i);
  pushByte(dccTx.idlePacket.symbols, i, 0xFF);   // First byte
  pushStartBit(dccTx.idlePacket.symbols, i);
  pushByte(dccTx.idlePacket.symbols, i, 0x00);   // Second byte
  pushStartBit(dccTx.idlePacket.symbols, i);
  pushByte(dccTx.idlePacket.symbols, i, 0xFF);   // XOR byte
  pushEndBit(dccTx.idlePacket.symbols, i);
  dccTx.idlePacket.numberOfSymbols = i;          // number of RMT symbols
}

static void prepareResetPacket() {
  int i = 0;                                  // counter within the buffer
  pushPreamble(dccTx.resetPacket.symbols, i);
  pushStartBit(dccTx.resetPacket.symbols, i);
  pushByte(dccTx.resetPacket.symbols, i, 0x00);  // First byte
  pushStartBit(dccTx.resetPacket.symbols, i);
  pushByte(dccTx.resetPacket.symbols, i, 0x00);  // Second byte
  pushStartBit(dccTx.resetPacket.symbols, i);
  pushByte(dccTx.resetPacket.symbols, i, 0x00);  // XOR byte
  pushEndBit(dccTx.resetPacket.symbols, i);
  dccTx.resetPacket.numberOfSymbols = i;         // number of RMT symbols
}


//================================================================================================
//                      C: RailCom Cutout Timer (ESP32 / GPTimer)
//================================================================================================

//************************************************************************************************
// C1: Inline helpers
//************************************************************************************************
// This timer is used to control the RailCom cutout pin. The timing reference is (re)started
// on every RMT_TX_DONE event, i.e. once per transmitted DCC packet.
// Only two timer values are necessary (see RCN-217):
// - TCS: start the RailCom cutout,
// - TCE: end the RailCom cutout.
// TTS1 and TTS2 are defined by RCN-217 as decoder timing windows for RailCom feedback.
// For Z21PG-style operation they are NOT required, because RailCom reception is handled
// asynchronously from the Arduino main loop rather than being tightly scheduled by this timer.
// The intermediate alarms (TTS1/TTS2) are therefore retained only as optional hooks for
// possible future extensions, such as precisely timed enabling of an ADC, comparator,
// or UART receiver.


static inline void setRailComTimerAlarm(uint64_t newAlarmValue) {
  gptimer_alarm_config_t alarmConfig = {};                              // Allocate space
  alarmConfig.alarm_count = newAlarmValue;                              // New value
  alarmConfig.flags.auto_reload_on_alarm = false;                       // One-shot operation
  ESP_ERROR_CHECK(gptimer_set_alarm_action(railComTimer, &alarmConfig));
}


static bool IRAM_ATTR railComTimerIsr(gptimer_handle_t timer,
  const gptimer_alarm_event_data_t *edata, void *user_ctx) {
  // Should we generate a RailCom cutout on the Aux pin?
  if (dccTx.railComActiveOnAux) {
    // alarmTime holds the timer value when the alarm was triggered.
    uint64_t alarmTime = edata->alarm_value;
    // This timer is called twice. First after packet transmission completed,
    // thus during the rmtDoneIsr, in which the alarmTime was set to 29us (TCS).
    // The second time the timer is called is from here, after 29us has passed.
    if ((alarmTime < TCE) && !dccTx.smActive){
      // 29us have passed. Start a cutout only if we are not is Service Mode
      disableAuxOutput();
      dccTx.railComCutoutInProgress = true;
      setRailComTimerAlarm(TCE);}
    else {
      // 436 us have passed. Enable the Aux signal only, if it was previously
      // stopped at the start of the cutout
      if (dccTx.railComCutoutInProgress) enableAuxOutput();
      dccTx.railComCutoutInProgress = false;
      gptimer_stop(railComTimer);}
    //
    // Possible future extension
    //  if (now < TTS1) {
    //    startRailComCutout();
    //    setRailComTimerAlarm(TTS1)}
    //  else if (now < TTS2) {
    //    do something;
    //    setRailComTimerAlarm(TTS2); }
    //  else if (now < TCE) {
    //    do something else;
    //    setRailComTimerAlarm(TCE); }
    //  else {
    //    stopRailComCutout();
    //    gptimer_stop(railComTimer);
    //  }
  }
  // Tell FreeRTOS whether an immediate context switch is required or not
  return false;     // no context switch needed
}


static inline void initRailComTimer() {
  // Configure ESP32 General Purpose Timer
  gptimer_config_t timerConfig = {};                                    // New empty timer config
  timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;                        // Default clock
  timerConfig.direction = GPTIMER_COUNT_UP;                             // Increment
  timerConfig.resolution_hz = 1000000;                                  // 1 MHz => 1 µs per tick
  timerConfig.intr_priority = 0;                                        // Default priority
  // Create a timer instance
  ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &railComTimer));
  // Init the Timer call back function
  gptimer_event_callbacks_t cbs = {};                                   // New empty call back
  cbs.on_alarm = railComTimerIsr;                                       // the call back name
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(railComTimer, &cbs, NULL));
  // Create and initialise the alarm configuration
  setRailComTimerAlarm(TCS);                                            // Time Cutout Start
  // Enable the timer, but do not yet start the timer!!
  ESP_ERROR_CHECK(gptimer_enable(railComTimer));
}


//================================================================================================
//                D: ESP32 RMT Transmission Engine and FreeRTOS Task Handling
//================================================================================================

//************************************************************************************************
// D1: FreeRTOS
//************************************************************************************************
// FreeRTOS task called after RMT transmission completed
// Helper function, to copy RMT symbols and call rmt_transmit
static inline void callRmtTransmit(DccPacket &packet) {
  // The RMT hardware has completed the previous packet and just started with the current packet
  // Now is the time to already fill the next packet, so we are sure it is waiting in the
  // RMT queue before the current packet has been completely transmitted
  espTx.currentBufferIndex ^= 1;                         // 0 => 1 / 1 => 0
  dcc_waveform_symbol_t* rmtBuffer = espTx.rmtPingPongBuf[espTx.currentBufferIndex];
  size_t bytesToCopy = packet.numberOfSymbols * sizeof(dcc_waveform_symbol_t);
  // Copy the precomputed RMT symbols from packet) into the active buffer (rmtBuffer)
  // This allows the application to prepare the next packet before the current packet completes
  memcpy(rmtBuffer, packet.symbols, bytesToCopy);
  // Start RMT transmission of the active buffer
  ESP_ERROR_CHECK(rmt_transmit(
      espTx.channel,
      espTx.encoder,
      rmtBuffer,
      bytesToCopy,
      &espTx.config
  ));
}


// Task for switching channels, starting RMT and setting the canAcceptPacket flag
// This task blocks until it is notified by the RMT TX-done ISR.
// After waking up, it performs application-specific actions
// (e.g. allowing a new packet, switching channel, starting a new TX).
static void rmtTxCompletionTask(void *pvParameters) {
  uint32_t notificationValue;        // FreeRTOS task notification value (used only as a wake-up signal)
  while (1) {                        // FreeRTOS tasks typically run forever
    // Block this task until the ISR sends a task notification.
    xTaskNotifyWait(
        0,                           // No notification bits are cleared on entry
        0xFFFFFFFF,                  // Clear all notification bits on exit
        &notificationValue,          // Receives the notification value
        portMAX_DELAY                // Wait indefinitely (no polling)
    );
    // We can proces a new packet provided we don't have to repeat earlier SM packets
    if (dccPacketEngine.isServiceModeRepeating() == false) {
      // Part 1: No SM repeating.
      if (dccTx.userPacket.numberOfSymbols > 0) {               // set in send()
        // Part 1A: We received a new packet from the user
        callRmtTransmit(dccTx.userPacket);                      // send packet
        if (dccTx.isServiceModePacket) {                        // set in send()
          dccTx.smRepeatsRemaining = dccTx.smMaxRepeats;        // initialise
        }
        else {
          // Part 1B: No SM packet, so no retransmission needed
          // Clear numberOfSymbols to avoid we send it again, and tell the user
          // we accept a new packet
          dccTx.userPacket.numberOfSymbols = 0;
          dccPacketEngine.canAcceptPacket = true;
        }
      }
      else {
        // No new packet: send the default packet
        callRmtTransmit(*dccTx.defaultPacket);
      }
    }
    else {
      // Part 2: We are in SM and have to repeat the message
      callRmtTransmit(dccTx.userPacket);
      // Decrement counter (Note: decrease is atomic operation)
      uint8_t repeats = dccTx.smRepeatsRemaining;
      if (repeats > 0) dccTx.smRepeatsRemaining = repeats - 1;
      if (dccTx.smRepeatsRemaining == 0) {
        dccTx.userPacket.numberOfSymbols = 0;
        dccPacketEngine.canAcceptPacket = true;
      }
    }
  }
}


//************************************************************************************************
// D2: ISR called once RMT transmission completed
//************************************************************************************************
bool IRAM_ATTR rmtDoneIsr(rmt_channel_handle_t chan,
  const rmt_tx_done_event_data_t *edata, void *user_ctx) {
    // Step 1: Flag that FreeRTOS may set to pdTRUE if a higher-priority task is woken
    BaseType_t woken = pdFALSE;
    // Step 2: Start the RailCom timer
    gptimer_set_raw_count(railComTimer, 0);             // Reset the counter value
    setRailComTimerAlarm(TCS);                          // Time Cutout Start
    gptimer_start(railComTimer);
    // xTaskNotifyFromISR is a FreeRTOS call to notify
    // Notify the RMT task from ISR context; FreeRTOS may update 'woken'
    xTaskNotifyFromISR(espTx.rmtCompletionTaskHandle, 1, eSetBits, &woken);
    // Tell FreeRTOS whether an immediate context switch is required or not
    return woken == pdTRUE;
}


//************************************************************************************************
// D3: Support functions for the RMT
//************************************************************************************************
inline rmt_tx_channel_config_t configureRmtTxChannel(uint8_t pin, bool invert) {
  // Configure a new channel
  // We don't use DMA, since DMA is not implemented on all ESP32 variants
  // Note: RMT FIFO = 64 symbols, but packets may be larger since they are streamed by copy_encoder
  rmt_tx_channel_config_t config = {};
  config.gpio_num = static_cast<gpio_num_t>(pin);
  config.clk_src = RMT_CLK_SRC_DEFAULT;    // Default clock
  config.resolution_hz = 1000000;          // 1MHz = 1μs per tick
  config.mem_block_symbols = (size_t)64;   // RMT memory per channel
  config.trans_queue_depth = (size_t)2;    // ping-pong software transmit queue (FreeRTOS)
  config.intr_priority = 1;                // shared ISR, priority 1 (higher than default)
  config.flags.invert_out = invert;        // invert signal if true
  return config;
}


static inline void createRmtEncoder() {
  // An encoder is responsible for encoding user data into RMT symbols
  // and then writing these symbols into the RMT FIFO buffer.
  // The copy_encoder used in this code is small, simple and relatively fast,
  // because it only streams precomputed symbols into the FIFO rather than
  // compute them on the fly.
  rmt_copy_encoder_config_t encoder_config = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &espTx.encoder));
}


static inline void setupRmtChannel(bool invert = false) {
  if (GPIO_IS_VALID_OUTPUT_GPIO(dccPacketEngine.dccRailPin)) {
    rmt_tx_channel_config_t cfg = configureRmtTxChannel(dccPacketEngine.dccRailPin, invert);
    ESP_ERROR_CHECK(rmt_new_tx_channel(&cfg, &espTx.channel));
    createRmtEncoder();
    ESP_ERROR_CHECK(rmt_enable(espTx.channel));
  }
  else dccPacketEngine.dccRailPin = 0xFF;
}


static inline void createRmtTxCompletionTask() {
  xTaskCreate(
    rmtTxCompletionTask,             // task name
    "rmtTxCompletionTask",           // for debugging
    2048,                            // stack size
    NULL,                            // task parameters (not needed here)
    5,                               // task priority (important, but not top priority)
    &espTx.rmtCompletionTaskHandle   // task handle
  );
}


static inline void startRmtWaveformGenerator() {
  // Set (or reset) state variables to their initial value.
  initTxEngines();
  // We create the RMT channel for the dccRailPin here
  setupRmtChannel(dccTx.dccSignalInverted);
  espTx.config.loop_count = 0;                 // Send this packet only once
  // Register a callback that triggers when a packet transmission completed.
  rmt_tx_event_callbacks_t cbs = {};
  cbs.on_trans_done = rmtDoneIsr;
  rmt_tx_register_event_callbacks(espTx.channel, &cbs, NULL);
  // Start the RMT by filling both RMT buffers with an idle packet
  // This ensures that the RMT can immediately switch to the other queue once
  // the first packet has been transmitted
  callRmtTransmit(dccTx.idlePacket);
  callRmtTransmit(dccTx.idlePacket);
  // Tell the user we can accept a new packet
  dccPacketEngine.canAcceptPacket = true;
}


static inline void stopRmtWaveformGenerator() {
  if (espTx.channel) {                               // RMT channel exists?
    ESP_ERROR_CHECK(rmt_disable(espTx.channel));     // disable RMT hardware
    ESP_ERROR_CHECK(rmt_del_channel(espTx.channel)); // release RMT channel
    espTx.channel = nullptr;
  }
  if (espTx.encoder) {                               // Encoder exists?
    ESP_ERROR_CHECK(rmt_del_encoder(espTx.encoder)); // release RMT encoder
    espTx.encoder = nullptr;
  }
}

//================================================================================================
//                                 E: DccPacketEngine class methods
//================================================================================================

//************************************************************************************************
// E1: setUp and Send
//************************************************************************************************
void DccPacketEngine::setupWaveformGenerator() {
  if (dccTx.isInitialized) return;
  // Step1: Set up the Aux pin
  initAuxPin();
  // Step2: create bit symbols and predefined DCC packets
  createDccBits();
  prepareIdlePacket();
  prepareResetPacket();
  // Step 3: Setup he RailCom timer
  initRailComTimer();
  // Step 4: After completion of each RMT tramission, we need a task to set the next
  createRmtTxCompletionTask();
  // STep 5: Start the RMT with idel packets
  startRmtWaveformGenerator();
  // Step 6: Initialization is done
  dccTx.isInitialized = true;
}

void DccPacketEngine::send(const uint8_t* data, uint8_t size) {
  // Sends a DCC packet (3–6 bytes including XOR).
  if (size == 0 || size > MaxDccSize) return;   // Illegal packet size
  if (canAcceptPacket) {                     // The user was allowed to send a new packet
    canAcceptPacket = false;
    // Check if this is a SM packet
    // RCN-216: Service Mode packets have 0111xxxx as first byte
    dccTx.isServiceModePacket = dccTx.smActive && ((data[0] & 0xF0) == 0b01110000);
    // Copy the received data to nextPacket
    int i = 0;                               // counter within the userPacket buffer
    pushPreamble(dccTx.userPacket.symbols, i);
    for (int b = 0; b < size; b++) {         // Push all user bytes, preceded by a start bit
      pushStartBit(dccTx.userPacket.symbols, i);
      pushByte(dccTx.userPacket.symbols, i, data[b]);
    }
    pushEndBit(dccTx.userPacket.symbols, i);
    dccTx.userPacket.numberOfSymbols = i;       // number of RMT symbols
  }
}


//******************************************************************************
// E2. Service Mode interface methods
//******************************************************************************
void DccPacketEngine::enterServiceMode(void) {
  // Enable Service Mode behaviour (long preamble, reset packets, SM flow).
  dccTx.smActive = true;
  dccTx.defaultPacket = &dccTx.resetPacket;
}

void DccPacketEngine::leaveServiceMode(void) {
  // The current implementation of DCCPacketScheduler calls leaveServiceMode
  // before every packet, and not, as one might expected, only after Service
  // Mode is over.
  if (dccTx.smActive == false) return;
  // Leave Service Mode immediately. Ongoing SM retransmissions are cancelled.
  dccTx.railComActiveOnAux = dccTx.railComAllowedOnAux;
  dccTx.smRepeatsRemaining = 0;
  dccTx.smActive = false;
  dccTx.defaultPacket = &dccTx.idlePacket;
}

bool DccPacketEngine::isServiceModeEnabled(void) {
  // Returns true if the system is currently in Service Mode.
  return dccTx.smActive;
}

void DccPacketEngine::setServiceModeMaxRepeats(uint8_t value) {
  dccTx.smMaxRepeats = value;
}

bool DccPacketEngine::isFirstServiceModePacket(void) {
  // True only for the first transmission of a new SM packet.
  return dccTx.smActive && (dccTx.smRepeatsRemaining == dccTx.smMaxRepeats);
}

bool DccPacketEngine::isServiceModeRepeating(void) {
  // True while the current SM packet is being retransmitted
  return dccTx.smActive && (dccTx.smRepeatsRemaining > 0);
}

void DccPacketEngine::stopServiceModeRepeats(void) {
  // Stop retransmission of the current SM packet (e.g. after ACK).
  dccTx.smRepeatsRemaining = 0;
}


//******************************************************************************
// E3. Inrush Protection Mode interface methods
//******************************************************************************
// Inrush protection is not implemented for this variant: empty stubs
void DccPacketEngine::enterInrushMode(void) {}
void DccPacketEngine::leaveInrushMode(void) {}
bool DccPacketEngine::isInrushModeEnabled(void) {return false;}


//******************************************************************************
// E.4. RailCom interface methods
//******************************************************************************
// Enable / disable generation of the cutout in the DCC signal on the DCC pin
void DccPacketEngine::setRailCom(bool active) {
  dccTx.railComAllowedOnDcc = active;
  prepareIdlePacket();           // We need to generate new idle / reset packets
  prepareResetPacket();
}

 // Is generation of the RailCom gap on the DCC pin enabled?
bool DccPacketEngine::getRailCom(void) {
  return dccTx.railComAllowedOnDcc;
}

// Enable / disable generation of a RailCom gap pulse on the Aux pin
void DccPacketEngine::setRailComGapInAux(bool useForRcCutout) {
  dccTx.railComAllowedOnAux = useForRcCutout;
  dccTx.railComActiveOnAux = dccTx.railComAllowedOnAux;
}

// This routine allows higher-layer software to determine whether a RailCom gap
// is currently in progress, so that rail voltage and/or current measurements
// can be safely controlled. Requires that the setRailComGapInAux() is set.
bool DccPacketEngine::railComGap(void) {
  return dccTx.railComCutoutInProgress;
}

//******************************************************************************
// E5: Start / Stop interface methods
//******************************************************************************
// In this HQ driver code enables (disables) the signal on the AUX pin. The pin
// can therefore be used to enable (disable) a connected H-Bridge. The
// polarity of the enable signal can be inverted by using setAuxActiveLevel().
// The AUX signal may include the RailCom cutout signal; the gap has the
// opposite polarity as dccTx.auxEnableLevel (thus the same level as disable).
// The DCC output will never be disabled, allowing switches and other
// accessories to remain operational.
void DccPacketEngine::StopOutputSignal() {
  if (!dccTx.auxOutputActive) return;
  dccTx.railComActiveOnAux = false;
  dccTx.railComCutoutInProgress = false;
  dccTx.auxOutputActive = false;
  disableAuxOutput();
}

void DccPacketEngine::RunOutputSignal() {
  if (dccTx.auxOutputActive) return;
  dccTx.railComActiveOnAux = dccTx.railComAllowedOnAux;
  dccTx.auxOutputActive = true;
  enableAuxOutput();
}

//******************************************************************************
// E6: Setters to change certain aspects of the DCC encoder
//******************************************************************************
void DccPacketEngine::setDccSignalInverted(bool inverted) {
  // Inverts the polarity of the DCC signal. No impact on AUX signal
  if (dccTx.dccSignalInverted == inverted) return;
  dccTx.dccSignalInverted = inverted;
  if (dccTx.isInitialized) {
    canAcceptPacket = false;
    stopRmtWaveformGenerator();
    startRmtWaveformGenerator();
  }
}

void DccPacketEngine::setPreambleLength(uint8_t value) {
  // RCN-211 (23.07.2023): Eine Zentrale muss mindestens 17 Synchronbits senden
  if (value < 17) return;
  dccTx.preambleLength = value;      // Length of normal preamble
}

void DccPacketEngine::setPreambleLengthSM(uint8_t value) {
  // RCN-216 (24.11.2025): im Programmiermodus mindestens 20 Synchronbits
  if (value < 20) return;
  dccTx.preambleLengthSM = value;    // Length of preamble in Service Mode
}

void DccPacketEngine::setAuxActiveLevel(bool activeHigh) {
  dccTx.auxEnableLevel = activeHigh;
}

// Not used by this variant
void DccPacketEngine::useMonitorPinforInrush(bool value) {(void)value;}
