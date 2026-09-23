//************************************************************************************************
//
// File:            DCCHardware_RP.inc - TMC variant
// Purpose:         DCC Waveform Generator for RP2040 (Raspberry Pi Pico) using PIO + DMA
// Copyright 2026:  Aiko Pras
// History:         2026-03-30 / Aiko Pras
//                  2026-09-22 (AP) - Adjusted for SDK version >= 2 and 4 Mhz clock
//                  2026-09-23 (AP) - Inrush protection added, to prevent DRV887x OCP
//
// Resources:
// - Pins: Two or three GPIO pins that can be controlled by a PIO state machine
// - PIO: one PIO block with one state machine
// - DMA: one DMA channel to feed the PIO TX FIFO
//
// Pin usage:
// The pins that the PIO uses for output MUST be consecutive pins.
// Therefore dccRailPin is taken as first pin, and:
// - dccRailAuxPin becomes dccRailPin + 1, and
// - dccMonitorPin becomes dccRailPin + 2.
// If the application configures dccMonitorPin as 0xFF, it will be ignored.
//
// Monitor pin
// ===========
// The monitor pin originates from the Z21PG design, where it was used for LocoNet
// RailSync and S88. The idea there was that switches and signals kept working even
// while the main track was short-circuited. It therefore always carried valid DCC
// packets, regardless of Power On/Off status.
// On future TMC boards this signal will no longer be used, and dccMonitorPin now
// defaults to 0xFF. This leaves the pin unused by the PIO and free for other use.
//
// Design choices
// ==============
// This driver uses the RP2040 PIO (Programmable IO) and DMA to generate a precise,
// jitter-free DCC waveform.
//
// - A PIO state machine generates the DCC signal based on precomputed waveform symbols.
// - DMA feeds the PIO TX FIFO with symbol data, avoiding CPU involvement during transmission.
//
// This architecture ensures highly accurate timing without ISR latency constraints,
// since waveform timing is handled entirely by the PIO hardware.
//
// Symbol-based waveform generation
// ================================
// The DCC signal is encoded as a sequence of half-bits:
//   - Logical '1'  → short pulse (≈58 µs total)
//   - Logical '0'  → long pulse  (≈100 µs total)
//
// Two halfbits form one symbol. Symbols are precomputed and stored in packet buffers.
// Symbols consists of phases (level + duration), similar to ESP32 RMT.

// Special symbols include:
//   - End bit
//   - gap1: first 29us of a cutout start bit
//   - gap1: Second 29us of a cutout start bit
//   - gap1: remaining (three) cutout bits
//
// DMA-driven transmission
// =======================
// Transmission is handled by a DMA channel:
// - DMA transfers 32-bit symbols directly into the PIO TX FIFO.
// - When a packet is fully transmitted, the DMA raises an interrupt.
// - The ISR does NOT generate waveform data, but only selects the next packet
//   and programs the DMA for the next transfer.
// This keeps ISR execution time minimal and deterministic.
//
// Packet scheduling
// =================
// The driver uses a packet-level scheduling model:
// - Default packets (idle or reset) are transmitted continuously.
// - When the user calls send(), a new packet is prepared and queued.
// - The DMA ISR switches to the new packet at the next packet boundary.
// Service Mode (SM) packets are automatically repeated as required.
//
// Ping-pong buffer
// ================
// Two packet buffers are used to avoid race conditions:
//  - sendPacketIndex : owned by send(), indicates which buffer is being filled.
//  - isrPacketIndex  : owned by the DMA ISR, indicates which buffer is being transmitted.
//
// Workflow:
// 1. send() builds a complete packet in the inactive buffer.
// 2. send() toggles sendPacketIndex and sets userPacketReady.
// 3. At the next DMA interrupt, the ISR switches to the new buffer.
//
// This producer/consumer model ensures:
// - No memcpy inside ISR
// - No data races
// - Constant ISR execution time
//
// RailCom support
// ===============
// RailCom cutout is supported on the DCC and AUX pins.
// The monitor pin has never a cutout
//
// Service Mode
// ============
// Service Mode packets:
// - Use a longer preamble
// - Are automatically repeated (configurable count)
// - Temporarily replace idle packets with reset packets
//
// The ISR manages repetition without CPU intervention in the main loop.
//
// Inrush protection waveform
// ===========================
// For inrush protection of DRV887x H-Bridges we need a pulse signal that is 2,5us high
// (this value must be shorter than the OCP reaction time of 3us), followed by a low signal
// (coast or brake, depending on the signal-inversion setting) of something between 10 and 20us.
// The length of this low signal may be modified but should be long enough to avoid thermal
// overload. To avoid any DC offset on the tracks, the high pulse will alter between both
// output pins. Note: the idea for this inrush protection mechanism was copied from DCC-ex.
//
// Reaction time and multiple nFAULT events:
// A default packet (idle/reset) is handed to the DMA as a single, atomic transfer and
// cannot be interrupted mid-flight. Since such a packet can take several milliseconds,
// while DRV887x tRETRY is only 2 ms, the application code may see multiple nFAULT/retry
// cycles before the inrush pattern appears on the track.
// This is a known and accepted limitation of the DMA-driven packet architecture.
//
// Call order
// ==========
// setupWaveformGenerator() must be called first, before any other method of this class.
// enterInrushMode() and RunOutputSignal()/StopOutputSignal() may be called in either
// order relative to each other.
//
//************************************************************************************************//
#include <Arduino.h>
#include "DCCHardware.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "DCCHardware_RP-PIO.inc"

/*
// For debuging
#define monitor Serial2
#define PIN_MONITOR   4
#define PIN_TEST1     5
#define PIN_TEST2     6
#define PIN_TEST3     7

#define DEBUG0     (sio_hw->gpio_togl = 1u << PIN_MONITOR)
#define DEBUG1     (sio_hw->gpio_togl = 1u << PIN_TEST1)
#define DEBUG2     (sio_hw->gpio_togl = 1u << PIN_TEST2)
#define DEBUG3     (sio_hw->gpio_togl = 1u << PIN_TEST3)
*/

//================================================================================================
//                                      Defines
//================================================================================================
#define MHZ_PIO    4                  // PIO clock speed, in MHz
#define PIO_OFFSET 4                  // Overhead induced by the PIO instructions, in PIO cycles

// Convert a duration in microseconds to a PIO N value
#define US_TO_PIO(us) ((us) * MHZ_PIO - PIO_OFFSET)

#define ZERO US_TO_PIO(100)           // RCN-210: zero half bit   (95.... us)
#define ONE  US_TO_PIO( 58)           // RCN-210: one half bit    (55..61 us)
#define TCS  US_TO_PIO( 29)           // RCN-217: cutout start    (26..32 us)
#define TTS1 US_TO_PIO( 80)           // RCN-217: start channel 1 (Unused by the current code)
#define TTS2 US_TO_PIO(193)           // RCN-217: start channel 2 (Unused by the current code)

#define INRUSH_ON  US_TO_PIO( 2.5)    // On-time:  must stay below tOCP (~3 us)
#define INRUSH_OFF US_TO_PIO(17.5)    // Off-time: configurable, keep between 10 and 20 us
#define INRUSH_REPEATS 25             // Number of ON/OFF cycles per inrush packet (25 => 1 ms)

#define MAX_SYMBOLS 128               // Long enough to hold a 6 byte DCC packet / inrush pattern
#define RAILCOM_CUTOUT_LENGTH 4       // see RCN-217 (Ausgabe 24.11.2025)
#define PIO_USED pio0                 // Which PIO will we use?

// Longest possible packet: preamble + 6 bytes (start bit + 8 bits each) + end bit,
// plus one extra symbol when a RailCom cutout is inserted
#define MAX_PREAMBLE (MAX_SYMBOLS - (MaxDccSize * 9) - 2)   // = 72


//================================================================================================
//                            A: Objects and Internal Data Structures
//================================================================================================

//************************************************************************************************
// A.1. Constructor definition and instantiation
//************************************************************************************************
DccPacketEngine::DccPacketEngine(){
  // Set initial pin values.
  // Normal usage: the application overwrites these values before setup (setupWaveformGenerator)
  dccRailPin = 2;                              // Pin for DCC rail Signal, with RailCom support
  dccRailAuxPin = 3;                           // Pin for Enable and RailCom gap (optional)
  dccMonitorPin = 0xFF;                        // Pin for the DCC-Monitor (deprecated)
}

// dccPacketEngine is the object that controls packet transmission for different hardware
DccPacketEngine dccPacketEngine;


//************************************************************************************************
// A.2. Declaration and initialisation of internal data structures
//************************************************************************************************
// Symbol definition similar to that of the ESP32-RMT. A union is used, to improve readability
union dcc_waveform_symbol_t {
    uint32_t value;
    struct {
        uint32_t levelA    : 3;    // use 3 bits
        uint32_t durationA : 13;   // use 13 bits
        uint32_t levelB    : 3;
        uint32_t durationB : 13;
    } bits;
};

struct DccPacket {
  dcc_waveform_symbol_t symbols[MAX_SYMBOLS];
  uint8_t numberOfSymbols;
};

struct DccSymbols {
  dcc_waveform_symbol_t one;    // DCC "1"
  dcc_waveform_symbol_t zero;   // DCC "0"
  dcc_waveform_symbol_t endbit; // DCC "1"
  dcc_waveform_symbol_t gap1;   // Railcom gap - first 29 us
  dcc_waveform_symbol_t gap2;   // Railcom gap - second 29 us
  dcc_waveform_symbol_t gap3;   // Railcom gap - remaining 3 ONE bits
};
static DccSymbols s;            // Short name, to improve readability in (inline) helper functions

struct DccTxEngine {                      // Processor independent
  uint pinsInUse = 0;                     // To distinguish between multiple setupWaveformGenerator calls
  // User Packet
  DccPacket userPacket[2];                // packet ping-pong buffer
  uint8_t sendPacketIndex;                // owned by send()
  volatile uint8_t isrPacketIndex;        // owned by DMA-ISR
  volatile bool userPacketReady;          // send() => DMA-ISR handover flag
  bool dccOutputActive = false;           // Flag used by RunOutputSignal / StopOutputSignal
  // Prepared packets
  DccPacket idlePacket;                   // Precomputed idle packet
  DccPacket resetPacket;                  // Precomputed reset packet
  DccPacket inrushPacket;                 // Precomputed inrush pattern
  bool inrushAlsoOnMonitorPin = true;     // The inrush pattern also appears on the monitor pin
  volatile bool inrushActive = false;     // True while inrush protection mode is active
  // Railcom
  bool railComAllowedOnDcc = false;       // DCC pin: should a RC gap be generated in the DCC signal?
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


struct PioTxEngine {                      // Processor dependent
  // We use one PIO, one state machine and a DMA
  PIO pio;                                // pio0 / pio1 (pio2 RP2350)
  uint smDcc;                             // DCC state machine in the PIO
  int dmaChannel;                         // The DMA channel for the PIO
};

// Object instantiation
static DccTxEngine dccTx;                 // This object holds generic data
static PioTxEngine pioTx;                 // This object holds PIO specific data

// Note that we do not (re)set railComAllowedOnDcc, smMaxRepeats, preambleLength,
// preambleLengthSM and dccSignalInverted, since the user may already have set these values.
static void initTxEngines() {
  // Service Mode settings
  dccTx.smActive = false;
  dccTx.isServiceModePacket = false;
  dccTx.smRepeatsRemaining = 0;
  // Clear the user packet
  dccTx.userPacket[0].numberOfSymbols = 0;
  dccTx.userPacket[1].numberOfSymbols = 0;
  dccTx.userPacketReady = false;
  dccTx.sendPacketIndex = 0;
  dccTx.isrPacketIndex  = 1;                  // opposite of dccTx.sendPacketIndex
}


//================================================================================================
//                      B: Helper Functions for DCC Bits and Pins
//================================================================================================

//************************************************************************************************
// B1: Pins
//************************************************************************************************
// setPinInverted may be called at any moment in time.
// The driver uses gpio_set_outover to change the pin's polarity.
// This call works for all "signal generators" (GPIO, PIO, UART, I2C,SPI etc.).
// Note that Arduino's digitalWrite(), but also pio_gpio_init, resets the inversion logic.
void setPinInverted(uint pin, bool inverted) {
  if (pin >= NUM_BANK0_GPIOS) return;
  if (inverted) gpio_set_outover(pin, GPIO_OVERRIDE_INVERT);
  else gpio_set_outover(pin, GPIO_OVERRIDE_NORMAL);
}


//************************************************************************************************
// B.2 Translating DCC bytes into symbols
//************************************************************************************************
static void createDccBits() {
  // A DCC bit consists of two halfbits: A and B
  // Step 1A: Determine for both halfbits the pin level for the two rail pins
  uint8_t levelA = dccTx.dccOutputActive ? 0b001 : 0b000;  // pin 0
  uint8_t levelB = dccTx.dccOutputActive ? 0b010 : 0b000;  // pin 1
  uint8_t gapLevelA = 0b000;                               // both pins off
  uint8_t gapLevelB = 0b000;                               // both pins off
  // Step 1B: monitor pin (bit2) always mirrors phase A, regardless of power status
  levelA |= 0b100;
  gapLevelA |= 0b100;
  // Step 2: add the timing bits
  // DCC "1"
  s.one.bits.levelA = levelA;
  s.one.bits.durationA = ONE;
  s.one.bits.levelB = levelB;
  s.one.bits.durationB = ONE;
  // DCC "0"
  s.zero.bits.levelA = levelA;
  s.zero.bits.durationA = ZERO;
  s.zero.bits.levelB = levelB;
  s.zero.bits.durationB = ZERO;
  // EndBit
  s.endbit.bits.levelA = levelA;
  s.endbit.bits.durationA = ONE;
  s.endbit.bits.levelB = levelB;
  s.endbit.bits.durationB = ONE;
  // Railcom start bit - Part 1
  s.gap1.bits.levelA = levelA;
  s.gap1.bits.durationA = TCS;
  s.gap1.bits.levelB = gapLevelA;
  s.gap1.bits.durationB = TCS;
  // Railcom start bit - Part 2
  s.gap2.bits.levelA = gapLevelB;
  s.gap2.bits.durationA = TCS;
  s.gap2.bits.levelB = gapLevelB;
  s.gap2.bits.durationB = TCS;
  // Railcom start bit - Part 3
  s.gap3.bits.levelA = gapLevelA;
  s.gap3.bits.durationA = ONE;
  s.gap3.bits.levelB = gapLevelB;
  s.gap3.bits.durationB = ONE;
};


// Helper functions to push a specific RMT symbol on the buffer
static void pushStartBit(dcc_waveform_symbol_t *buf, int &i) { buf[i++] = s.zero;}
static void pushEndBit(dcc_waveform_symbol_t *buf, int &i) { buf[i++] = s.endbit;}
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
    if (dccTx.railComAllowedOnDcc) {  // Cutout needed?
      buf[i++] = s.gap1;              // First part of Railcom startbit
      buf[i++] = s.gap2;              // Second part of Railcom startbit
      buf[i++] = s.gap3;              // Three preamble bits, only on the monitor pin
      buf[i++] = s.gap3;
      buf[i++] = s.gap3;
      length = length - 4;
    }
    for (int p = 0; p < length; p++) pushBit(buf, i, 1);
  }
}


static void prepareIdlePacket() {
  int i = 0;                                     // counter within the buffer
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
  int i = 0;                                     // counter within the buffer
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

static inline void createDccDefaultPackets() {
  // Step 1: Create the bit lengths that will be used in subsequent packets
  createDccBits();
  // Step 2: (re)create the Idle and Reset packets
  prepareIdlePacket();
  prepareResetPacket();
}


static inline void createInrushPattern() {
  // The inrush pulse train consists of two alternating "quarter-bits": IN1-on and IN2-on,
  // each followed by an off-phase — analogous to the A/B halfbits of a regular DCC bit.
  // Step 1A: Determine the pin levels for the two rail pins
  uint8_t levelIn1 = dccTx.dccOutputActive ? 0b001 : 0b000;  // pin 0 on
  uint8_t levelIn2 = dccTx.dccOutputActive ? 0b010 : 0b000;  // pin 1 on
  uint8_t levelOff = 0b000;                                  // both rail pins off
  // Step 1B: monitor pin (bit2) always mirrors the ON phase, regardless of power status
  uint8_t monitorBit = dccTx.inrushAlsoOnMonitorPin ? 0b100 : 0b000;
  levelIn1 |= monitorBit;
  levelIn2 |= monitorBit;
  // Step 2: build the alternating symbol pairs
  // INRUSH_ON and INRUSH_OFF are #defines that calculate the durations
  for (int r = 0; r < INRUSH_REPEATS; r++) {
    int i = r * 2;
    // Symbol A: IN1 on, then off
    dccTx.inrushPacket.symbols[i].bits.levelA    = levelIn1;
    dccTx.inrushPacket.symbols[i].bits.durationA = INRUSH_ON;
    dccTx.inrushPacket.symbols[i].bits.levelB    = levelOff;
    dccTx.inrushPacket.symbols[i].bits.durationB = INRUSH_OFF;
    // Symbol B: IN2 on, then off (opposite polarity, for DC balance)
    dccTx.inrushPacket.symbols[i+1].bits.levelA    = levelIn2;
    dccTx.inrushPacket.symbols[i+1].bits.durationA = INRUSH_ON;
    dccTx.inrushPacket.symbols[i+1].bits.levelB    = levelOff;
    dccTx.inrushPacket.symbols[i+1].bits.durationB = INRUSH_OFF;
  }
  dccTx.inrushPacket.numberOfSymbols = INRUSH_REPEATS * 2;
}


//================================================================================================
//                                          C: PIO DMA
//================================================================================================
// transmitNextPacket uses two DMA calls:
// - dma_channel_set_read_addr      => the adres of the next (symbols) packet
// - dma_channel_set_trans_count    => the length (in symbols) of the next packet
// The third parameter indicates that the DMA should restart
static inline void transmitNextPacket(DccPacket* packet) {
  dma_channel_set_read_addr(
      pioTx.dmaChannel,
      packet->symbols,
      false
  );
  dma_channel_set_trans_count(
      pioTx.dmaChannel,
      packet->numberOfSymbols,
      true
  );
}


static inline void procesNextPacket() {
  // Priority 1: inrush protection
  if (dccTx.inrushActive) {
    // while inrush protection is active, we continuously send inrush "packets", which last
    // around 1ms each.
    dccTx.smRepeatsRemaining = 0;            // abort repeating ongoing SM messages, if any
    dccTx.userPacketReady = false;           // discard any pending (possibly SM) packet
    transmitNextPacket(&dccTx.inrushPacket);
  }
  // Priority 2: SM was entered earlier and the SM packet must still be repeated
  else if (dccPacketEngine.isServiceModeRepeating()) {
    transmitNextPacket(&dccTx.userPacket[dccTx.isrPacketIndex]);
    // Decrement counter (Note: decrease is atomic operation)
    uint8_t repeats = dccTx.smRepeatsRemaining;
    if (repeats > 0) dccTx.smRepeatsRemaining = repeats - 1;
  }
  // Priority 3: normal case. Send a new user packet, or idle/reset
  else {
    if (dccTx.userPacketReady) {
      // We received a new packet from the user via the send() method.
      // Switch ISR to the buffer that was just filled by send():
      // sendPacketIndex has already been toggled in send(), so "ready" is the other one.
      dccTx.isrPacketIndex = dccTx.sendPacketIndex ^ 1;
      transmitNextPacket(&dccTx.userPacket[dccTx.isrPacketIndex]);
      dccTx.userPacketReady = false;
      if (dccTx.isServiceModePacket) {                  // set in send()
        dccTx.smRepeatsRemaining = dccTx.smMaxRepeats;  // initialise repeat counter
      }
    } else {
      // No new packet received from the user: we should send an idle (normal) packet
      // except if we are in service mode, in which case we have to send a reset packet.
      if (!dccTx.smActive) {transmitNextPacket(&dccTx.idlePacket);}
        else {transmitNextPacket(&dccTx.resetPacket);}
    }
  }
  if (dccTx.smRepeatsRemaining == 0 && !dccTx.inrushActive) dccPacketEngine.canAcceptPacket = true;
}


// The DMA triggers an ISR once all data is handed to the FIFO/PIO
static void dma_isr() {
  if (dma_hw->ints0 & (1u << pioTx.dmaChannel)) {
    dma_hw->ints0 = 1u << pioTx.dmaChannel;   // clear interrupt
    procesNextPacket();
  }
}

static void setupDma() {
  // Step 1: get a DMA channel
  pioTx.dmaChannel = dma_claim_unused_channel(true);     // TODO: Add error handling code
  // Step 2: create a configuration:
  // - Provide 32 bit words to the PIO
  // - Increment DMA address after read
  // - Do not increment DMA address after write
  // - DMA triggered once channel gets empty by PIO TX (dreq = Data Request)
  dma_channel_config cnfg = dma_channel_get_default_config(pioTx.dmaChannel);
  channel_config_set_transfer_data_size(&cnfg, DMA_SIZE_32);
  channel_config_set_read_increment(&cnfg, true);
  channel_config_set_write_increment(&cnfg, false);
  channel_config_set_dreq(&cnfg, pio_get_dreq(pioTx.pio, pioTx.smDcc, true));
  // Step 3: perform the configuration
  dma_channel_configure(
    pioTx.dmaChannel,
    &cnfg,
    &(pioTx.pio->txf[pioTx.smDcc]),                      // Target: PIO TX FIFO
    dccTx.idlePacket.symbols,                            // Source: our packet
    dccTx.idlePacket.numberOfSymbols,                    // Length of the data
    false                                                // do not start yet
  );
  // Step 4: Configure DMA completion interrupt
  dma_channel_set_irq0_enabled(pioTx.dmaChannel, true);  // Allow DMA to raise IRQ0
  irq_set_exclusive_handler(DMA_IRQ_0, dma_isr);         // Register the CPU interrupt handler
  irq_set_enabled(DMA_IRQ_0, true);                      // Enable the interrupt in the NVIC
  // Step 5: Start DMA channel A
  dma_channel_start(pioTx.dmaChannel);
}

//================================================================================================
//                               D: PIO Transmission Engine
//================================================================================================
static void PioSmDccInit(PIO pio, uint sm, uint offset, uint pin) {
  // Step 1: Assigns to the variable pioConfig a default config, which may
  // subsequently be modified. "offset" is the PIO programs offset
  // dccPulseGenerator_program_get_default_config is a static inline function generated
  // by the pioasm (see toggle.pio.h)
  pio_sm_config pioConfig = dccPulseGenerator_program_get_default_config(offset);
  // Step 2: Configure the GPIO pins so they are controlled by the PIO instead of the SIO
  // Since we use multiple pins, pio_gpio_init should be called once per pin.
  // Note that we MUST use consecutive GPIO pins (e.g. 2, 3, 4)

  pio_gpio_init(pio, pin);
  pio_gpio_init(pio, pin+1);
  // Step 3: Configure the pins. The parameter "2" represents the number of pins
  // and the parameter "true" tells that the pins are used for output (false = input)
  pio_sm_set_consecutive_pindirs(pio, sm, pin, 2, true);
  // Step 4: Specify which pins the PIO instruction "out pins, x" will operate upon.
  // The parameter "pin" is the start pin, and "2" is the number of consecutive pins.
  // Note that the PIO has four different pin-mappings, which can be configured
  // independently, for the following instructions: set pins, x / out pins, x /
  // in pins, x and sideset pins, x
  sm_config_set_out_pins(&pioConfig, pin, 2);
  // Step 5: Configure the clock to tick every 250 ns.
  // Use F_CPU to compensate for different clock speeds.
  // Use MHZ_PIO to allow easy changes to the clock
  float pio_clk = (float)MHZ_PIO * 1e6f;
  float clkdiv = (float)F_CPU / pio_clk;
  sm_config_set_clkdiv(&pioConfig, clkdiv);
  // Step 6: Configure the shift register
  // The variable "pull_threshold" specifies the number of bits to shift out before
  // auto/conditional re-pulling of the OSR. Since the PIO program performs 'out x, 32"
  // any value 1..32 should work. In initial tests the value 32 didn't seem to work
  // whereas the value 1 did work
  bool shift_right = true;     // shift right
  bool autopull = true;
  //uint pull_threshold = 1;      // threshold in bits to shift out before auto/conditional re-pulling of the OSR
  uint pull_threshold = 32;
  sm_config_set_out_shift(&pioConfig, shift_right, autopull, pull_threshold);
  // Step 7: Initialse the state machine with the provided configuration
  pio_sm_init(pio, sm, offset, &pioConfig);
}

static void addThirdPioPin(uint pin) {
  pio_gpio_init(pioTx.pio, pin+2);
  pio_sm_set_consecutive_pindirs(pioTx.pio, pioTx.smDcc, pin, 3, true);
  // We can't use sm_config_set_out_pins, since that would require a reinitialise of the SM
  // Therefore we modify the PINCTRL register directly
  pioTx.pio->sm[pioTx.smDcc].pinctrl = (pioTx.pio->sm[pioTx.smDcc].pinctrl
    & ~PIO_SM0_PINCTRL_OUT_COUNT_BITS) | (3u << PIO_SM0_PINCTRL_OUT_COUNT_LSB);
}

static void setupPio() {
  // Step 1: Select which PIO we will use. The RP2040 has pio0 and pio1, RP2350 has also pio2
  pioTx.pio = PIO_USED;
  // Step 2: The PIO can only address 32 consecutive GPIOs at a time. That is okay for the RP2040.
  // However, the RP2350 has 48 pins. If we use a RP2350, we must select via gpio_base = 0
  // (pins 0..31) or gpio_base = 16 (pins 16..47) which pins we use.
  // All three pins we use (dccRailPin, +1, +2) must fit within a single such window,
  // so we choose the window based on the highest of those three pins.
  if ((dccPacketEngine.dccRailPin + 2) <= 31) pio_set_gpio_base(pioTx.pio, 0);
    else pio_set_gpio_base(pioTx.pio, 16);
  // Step 3: On this PIO, ask which of the four state machines are available (0..3)
  // The parameter "true" says that this state machine will also be assigned
  pioTx.smDcc = pio_claim_unused_sm(pioTx.pio, true);
  // Step 4: Load the PIO code of the state machine into this PIO's instruction memory
  // The start address is returned as parameter and assigned to "offset"
  uint dccOffset = pio_add_program(pioTx.pio, &dccPulseGenerator_program);
  // Step 5: Initialise  and start the DCC-pin state machine (true => enable, false => disable)
  PioSmDccInit(pioTx.pio, pioTx.smDcc, dccOffset, dccPacketEngine.dccRailPin);
  pio_sm_set_enabled(pioTx.pio, pioTx.smDcc, true);
}


//================================================================================================
//                                 E: DccPacketEngine class methods
//================================================================================================

//************************************************************************************************
// E1: setUp and Send
//************************************************************************************************
// As part of setup(), the packet scheduler code disables dccMonitorPin by default (0xFF).
// To enable the monitor pin, enable_additional_DCC_output() should be called, which sets
// dccMonitorPin to some valid value, after which setupWaveformGenerator is called a second time.
// The code below can handle this particular call sequence, although it also works the first time
// provided dccMonitorPin is a valid pin number.
void DccPacketEngine::setupWaveformGenerator() {
  switch (dccTx.pinsInUse) {
    case 0:
      if ((uint8_t)(dccRailPin + 1) < NUM_BANK0_GPIOS) {   // We need at least 2 pins
        // Step 1: Set default values
        initTxEngines();
        // Step2: create bit symbols and default packets, as well as the inrush pattern
        createDccDefaultPackets();
        createInrushPattern();
        // Step 3: Setup the PIO and DMA that feeds the PIO
        setupPio();
        setupDma();
        // Step 4: Invert the DCC signal, if needed
        setDccSignalInverted(dccTx.dccSignalInverted);
        // Step 5: Initialization is done
        dccRailAuxPin = dccRailPin + 1;                // tell the user which pin is used
        dccTx.pinsInUse = 2;
        if (dccMonitorPin < NUM_BANK0_GPIOS) {         // Should we already configure the third pin
          addThirdPioPin(dccPacketEngine.dccRailPin);  // Use dccRailPin as start pin
          dccMonitorPin = dccRailPin + 2;              // tell the user which pin is used
          dccTx.pinsInUse = 3;
        }
        dccPacketEngine.canAcceptPacket = true;
      }
    break;
    case 2:
      // If the dccMonitorPin has "some" valid value ()
      if (dccMonitorPin < NUM_BANK0_GPIOS) {           // We need a third pin
        addThirdPioPin(dccPacketEngine.dccRailPin);    // Use dccRailPin as start pin
        dccMonitorPin = dccRailPin + 2;                // tell the user which pin is used
        dccTx.pinsInUse = 3;
      }
    break;
    default:                                           // Should not occur
    break;
  }
}

void DccPacketEngine::send(const uint8_t* data, uint8_t size) {
  // Sends a DCC packet (3–6 bytes including XOR).
  if (size == 0 || size > MaxDccSize) return;   // Illegal packet size
  if (!canAcceptPacket) return;                 // User not allowed to send yet
  canAcceptPacket = false;
  // Select the packet buffer owned by send()
  uint8_t index = dccTx.sendPacketIndex;
  DccPacket& packet = dccTx.userPacket[index];
  // Check if this is a Service Mode packet
  // RCN-216: Service Mode packets have 0111xxxx as first byte
  dccTx.isServiceModePacket = dccTx.smActive && ((data[0] & 0xF0) == 0b01110000);
  // Build the packet into the selected buffer
  int i = 0;                                      // counter within the packet buffer
  pushPreamble(packet.symbols, i);
  for (int b = 0; b < size; b++) {                // Push all user bytes, preceded by a start bit
    pushStartBit(packet.symbols, i);
    pushByte(packet.symbols, i, data[b]);
  }
  pushEndBit(packet.symbols, i);
  // Publish: numberOfSymbols last
  packet.numberOfSymbols = (uint8_t)i;
  // Hand over to ISR
  dccTx.sendPacketIndex ^= 1;                     // switch to the other buffer for the next send()
  dccTx.userPacketReady = true;
}


//******************************************************************************
// E2. Service Mode interface methods
//******************************************************************************
void DccPacketEngine::enterServiceMode(void) {
  // Enable Service Mode behaviour (long preamble, reset packets, SM flow).
  dccTx.smActive = true;
}

void DccPacketEngine::leaveServiceMode(void) {
  // The current implementation of DCCPacketScheduler calls leaveServiceMode
  // before every packet, and not, as one might expected, only after Service
  // Mode is over. If needed, cancel ongoing retransmissions
  dccTx.smRepeatsRemaining = 0;
  dccTx.smActive = false;
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
void DccPacketEngine::enterInrushMode(void) {
  // send sequence of short pulses to prevent DRV887x OCP
  dccTx.inrushActive = true;
}

void DccPacketEngine::leaveInrushMode(void) {
  // Back to normal mode
  dccTx.inrushActive = false;
}

bool DccPacketEngine::isInrushModeEnabled(void) {
// True while inrush mode is active
  return dccTx.inrushActive;
}


//******************************************************************************
// E.4. RailCom interface methods
//******************************************************************************
// Enable / disable generation of the cutout in the DCC signal on the DCC pin
void DccPacketEngine::setRailCom(bool active) {
  dccTx.railComAllowedOnDcc = active;
  createDccDefaultPackets();
}

// Is generation of the RailCom gap on the DCC pin enabled?
bool DccPacketEngine::getRailCom(void) {
  return dccTx.railComAllowedOnDcc;
}

// This routine allows higher-layer software to determine whether a RailCom gap
// is currently in progress, so that rail voltage and/or current measurements
// can be safely controlled. In this implementation we check the value of the
// output pin via gpio_get_all64().
// The idea is that, during the cutout, the DCC and the inverted pin will have
// the same value (0 or 1, depending on setDccSignalInverted). We sample twice
// since the PIO pin changes make take place during a sample.
bool DccPacketEngine::railComGap(void) {
  if (dccTx.inrushActive) return false;  // avoid false positives during inrush
  bool cutoutDetected = false;
  if (dccTx.dccOutputActive) {
    uint64_t pins = gpio_get_all64();
    bool pin0 = (pins >> dccRailPin) & 1;
    bool pin1 = (pins >> (dccRailPin + 1)) & 1;
    cutoutDetected = (pin0 == pin1);
    if (cutoutDetected) {  // To avoid glitches, take a second sample
      pins = gpio_get_all64();
      pin0 = (pins >> dccRailPin) & 1;
      pin1 = (pins >> (dccRailPin + 1)) & 1;
      cutoutDetected = (pin0 == pin1);
    }
  }
  return cutoutDetected;
}


//******************************************************************************
// E5: Start / Stop interface methods
//******************************************************************************
// This driver enables (disables) the signal on the DCC and inverted DCC pins.
// The Monitor pin output will never be disabled, allowing switches and other
// accessories to remain operational.
//
// When StopOutputSignal() is called, the flag dccOutputActive is disabled,
// and createDccDefaultPackets and createInrushPattern are called.
// These subroutines modify the bit mask for the 3 PIO pins, ensuring that the
// DCC and inverted DCC pins will always stay low.
void DccPacketEngine::StopOutputSignal() {
  if (!dccTx.dccOutputActive) return;
  dccTx.dccOutputActive = false;
  createDccDefaultPackets();
  createInrushPattern();
}

void DccPacketEngine::RunOutputSignal() {
  if (dccTx.dccOutputActive) return;
  dccTx.dccOutputActive = true;
  createDccDefaultPackets();
  createInrushPattern();
}

//******************************************************************************
// E6: Setters to change certain aspects of the DCC encoder
//******************************************************************************
void DccPacketEngine::setDccSignalInverted(bool inverted) {
  // Inverts the polarity of the DCC and inverted DCC signal. Usefull to invert
  // the signal to disable the H-Bridge. No impact on Monitor signal.
  dccTx.dccSignalInverted = inverted;
  setPinInverted(dccPacketEngine.dccRailPin, inverted);
  setPinInverted(dccPacketEngine.dccRailPin + 1, inverted);
}

void DccPacketEngine::setPreambleLength(uint8_t value) {
  // RCN-211 (23.07.2023): Eine Zentrale muss mindestens 17 Synchronbits senden
  if (value < 17 || value > MAX_PREAMBLE)  return;
  dccTx.preambleLength = value;      // Length of normal preamble
  createDccDefaultPackets();         // Reset and Idle packet must be regenerated
}

void DccPacketEngine::setPreambleLengthSM(uint8_t value) {
  // RCN-216 (24.11.2025): im Programmiermodus mindestens 20 Synchronbits
  if (value < 20 || value > MAX_PREAMBLE) return;
  dccTx.preambleLengthSM = value;    // Length of preamble in Service Mode
  createDccDefaultPackets();         // Reset and Idle packet must be regenerated
}

void DccPacketEngine::useMonitorPinforInrush(bool value) {
  if (dccTx.inrushActive) return;   // Avoid rebuilding a buffer the DMA may be reading
  dccTx.inrushAlsoOnMonitorPin = value;
  createInrushPattern();             // Rebuild, so the change takes effect immediately
}

void DccPacketEngine::setRailComGapInAux(bool active) {(void)active;}         // Unused in this driver
void DccPacketEngine::setAuxActiveLevel(bool activeHigh) {(void)activeHigh;}  // Unused in this driver
