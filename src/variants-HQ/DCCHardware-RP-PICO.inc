//************************************************************************************************
//
// File:            DCCHardware-RP-PICO-PIO.cpp
// Purpose:         DCC Waveform Generator for RP2040 (Raspberry Pi Pico) using PIO + DMA
// Copyright 2026:  Aiko Pras
// History:         2026-03-22 / Aiko Pras - Updated for PIO/DMA architecture
//
// Resources:
// - DCC pin: any GPIO pin that can be controlled by a PIO state machine
// - AUX pin: any GPIO pin (optional), used for H-bridge enable and/or RailCom cutout
// - PIO: one PIO block with two state machines (one for DCC, one for AUX)
// - DMA: one DMA channel to feed the PIO TX FIFO
//
// Design choices
// ==============
// This driver uses the RP2040 PIO (Programmable IO) and DMA to generate a precise,
// jitter-free DCC waveform.
//
// - A PIO state machine generates the DCC signal based on precomputed waveform symbols.
// - A second PIO state machine (optional) generates the RailCom cutout on the AUX pin.
// - DMA feeds the PIO TX FIFO with symbol data, avoiding CPU involvement during transmission.
//
// This architecture ensures highly accurate timing without ISR latency constraints,
// since waveform timing is handled entirely by the PIO hardware.
//
// Symbol-based waveform generation
// ================================
// The DCC signal is encoded as a sequence of "symbols", each representing a half-bit:
//   - Logical '1'  → short pulse (≈58 µs total)
//   - Logical '0'  → long pulse  (≈100 µs total)
//
// Each symbol consists of two phases (level + duration), similar to ESP32 RMT.
// These symbols are precomputed and stored in packet buffers.
//
// Special symbols include:
//   - End bit
//   - RailCom cutout gap
//   - Start-of-packet marker (used to trigger AUX cutout timing in the PIO)
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
// RailCom cutout can be generated in two ways:
// 1. On the DCC signal itself (gap in waveform)
// 2. On the AUX pin via a dedicated PIO state machine
//
// A special "marker" symbol is inserted at the start of each packet to
// synchronize the AUX cutout timing with the DCC waveform.
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
//************************************************************************************************//
#include <Arduino.h>
#include "DCCHardware.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "DCCHardware-RP-PICO-PIO.inc"

// For debug purposes
/*
#define monitor Serial
#define PIN_MONITOR   4
#define PIN_TEST1     5
#define PIN_TEST2     6
#define PIN_TEST3     7

// For debugging
#define DEBUG0     (sio_hw->gpio_togl = 1u << PIN_MONITOR)
#define DEBUG1     (sio_hw->gpio_togl = 1u << PIN_TEST1)
#define DEBUG2     (sio_hw->gpio_togl = 1u << PIN_TEST2)
#define DEBUG3     (sio_hw->gpio_togl = 1u << PIN_TEST3)
*/


//================================================================================================
//                                      Defines
//================================================================================================
#define PIO_OFFSET 7                  // Overhead induced by the PIO instructions
#define PIO_MARKER_OFFSET 6           // Overhead from the start of packet marker
#define ZERO (100 - PIO_OFFSET)       // RCN-210: zero half bit   (95.... us)
#define ONE  ( 58 - PIO_OFFSET)       // RCN-210: one half bit    (55..61 us)
#define TCS  ( 29 - PIO_OFFSET)       // RCN-217: cutout start    (26..32 us)
#define TCE  (464 - PIO_OFFSET - TCS) // RCN-217: cutout end      (454..488 us)
#define TTS1 ( 80 - PIO_OFFSET)       // RCN-217: start channel 1 (Unused by the current code)
#define TTS2 (193 - PIO_OFFSET)       // RCN-217: start channel 2 (Unused by the current code)

#define MAX_SYMBOLS 128               // Long enough to hold a 6 byte DCC packet
#define RAILCOM_CUTOUT_LENGTH 4       // see RCN-217 (Ausgabe 24.11.2025)
#define PIO_USED pio0                 // Which PIO will we use?

//================================================================================================
//                            A: Objects and Internal Data Structures
//================================================================================================

//************************************************************************************************
// A.1. Constructor definition and instantiation
//************************************************************************************************
DccPacketEngine::DccPacketEngine(){
  dccRailPin = 2;                                 // Pin for DCC rail Signal, with RailCom support
  dccRailAuxPin = 3;                              // Pin for Enable and RailCom gap (optional)
  dccMonitorPin = 0xFF;                           // Not used in this HQ variant
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
        uint32_t levelA    : 1;
        uint32_t durationA : 15;
        uint32_t levelB    : 1;
        uint32_t durationB : 15;
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
  dcc_waveform_symbol_t gap;    // Railcom gap
  dcc_waveform_symbol_t marker; // Marker indicating the start of a new DCC packet
};
static DccSymbols s;            // Short name, to improve readability in (inline) helper functions

struct DccTxEngine {                      // Processor independent
  bool isInitialized = false;             // Flag to ensure setupWaveformGenerator is called once only
  // Aux pin
  uint auxPin;                            // Pin for the Power ON/OFF / RailCom cutout
  bool auxPinConfigured;                  // To protect against invalid pin values
  bool auxEnableLevel = 0;                // 0..1: The value of the AUX pin once it is enabled
  bool auxOutputActive = false;           // Flag used by RunOutputSignal / StopOutputSignal
  // User Packet
  DccPacket userPacket[2];                // packet ping-pong buffer
  uint8_t sendPacketIndex;                // owned by send()
  volatile uint8_t isrPacketIndex;        // owned by DMA-ISR
  volatile bool userPacketReady;          // send() => DMA-ISR handover flag
  // Prepared packets
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


struct PioTxEngine {                      // Processor dependent
  // We use one PIO and two state machines
  PIO pio;                                // pio0 / pio1 (pio2 RP2350)
  uint smDcc;                             // DCC state machine in the PIO
  uint smAux;                             // RailCom cutout state machine in the PIO
  // The DMA channel for the PIO
  int dmaChannel;
};

// Object instantiation
static DccTxEngine dccTx;                 // This object holds generic data
static PioTxEngine pioTx;                 // This object holds PIO specific data

// Note that we do not (re)set railComAllowedOnDcc, railComAllowedOnAux, smMaxRepeats, preambleLength,
// preambleLengthSM and dccSignalInverted, since the user may already have set these values.
static void initTxEngines() {
  // Aux pin state / Railcom
  dccTx.railComActiveOnAux = dccTx.railComAllowedOnAux;
  dccTx.railComCutoutInProgress = false;
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
  // Initialise the default packet
  dccTx.defaultPacket = &dccTx.idlePacket;
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
//
// initAuxPin() is called only once, at the start of setupWaveformGenerator, thus before the
// RailCom timer is initialised.
static inline void initAuxPin(uint pin) {
  // The pin wil not be used if its value is an invalid pin (such as 0xFF)
  if (pin <= NUM_BANK0_GPIOS) {
    dccTx.auxPinConfigured = true;
    dccTx.auxPin = pin;
    pinMode(dccTx.auxPin, OUTPUT);
    digitalWriteFast(dccTx.auxPin, !dccTx.auxEnableLevel);
  }
  else {
    dccTx.auxPinConfigured = false;
    dccPacketEngine.dccRailAuxPin = 0xFF;
  }
}

// setPinInverted may be called at any moment in time.
// The driver uses gpio_set_outover to change the pin's polarity.
// This call works for all "signal generators" (GPIO, PIO, UART, I2C,SPI etc.).
// Note that Arduino's digitalWrite(), but also pio_gpio_init, resets the inversion logic.
void setPinInverted(uint pin, bool inverted) {
  if (pin > NUM_BANK0_GPIOS) return;
  if (inverted) gpio_set_outover(pin, GPIO_OVERRIDE_INVERT);
  else gpio_set_outover(pin, GPIO_OVERRIDE_NORMAL);
}


//************************************************************************************************
// B.2 Translating DCC bytes into RMT symbols
//************************************************************************************************
// This part translates a DCC byte array into a RMT symbol structure.
static void createDccBits() {
  // DCC "1"
  s.one.bits.levelA = 1;
  s.one.bits.durationA = ONE;
  s.one.bits.levelB = 0;
  s.one.bits.durationB = ONE;
  // DCC "0"
  s.zero.bits.levelA = 1;
  s.zero.bits.durationA = ZERO;
  s.zero.bits.levelB = 0;
  s.zero.bits.durationB = ZERO;
  // EndBit
  s.endbit.bits.levelA = 1;
  s.endbit.bits.durationA = ONE;
  s.endbit.bits.levelB = 0;
  s.endbit.bits.durationB = ONE;
  // Railcom gap
  s.gap.bits.levelA = 1;
  s.gap.bits.durationA = TCS;
  s.gap.bits.levelB = 0;
  s.gap.bits.durationB = TCE;
  // DCC Start of packet marker (0x00000000)
  // The marker entends the duration of levelB of the endbit
  s.marker.bits.levelA = 0;
  s.marker.bits.durationA = 0;
  s.marker.bits.levelB = 0;
  s.marker.bits.durationB = 0;
};


// Helper functions to push a specific RMT symbol on the buffer
static void pushStartBit(dcc_waveform_symbol_t *buf, int &i) { buf[i++] = s.zero; }
static void pushEndBit(dcc_waveform_symbol_t *buf, int &i) { buf[i++] = s.endbit; }
static void pushRailComGap(dcc_waveform_symbol_t *buf, int &i) { buf[i++] = s.gap; }
static void pushBit(dcc_waveform_symbol_t *buf, int &i, bool bit) { buf[i++] = bit ? s.one : s.zero;}
static void pushByte(dcc_waveform_symbol_t *buf, int &i, uint8_t value) {
  for (int b = 7; b >= 0; b--) { pushBit(buf, i, value & (1 << b)); }
}

static void pushMarker(dcc_waveform_symbol_t *buf, int &i) {
  // Inserts a special marker symbol before the packet, to inform the PIO that a cutout
  // should be generated in the AUX signal
  if (dccTx.railComActiveOnAux) {buf[i++] = s.marker;}
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
  int i = 0;                                     // counter within the buffer
  pushMarker(dccTx.idlePacket.symbols, i);       // Start of a new packet
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
  pushMarker(dccTx.resetPacket.symbols, i);
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
  // Step 1: compensate the second half of the DCC packet endbit for the inclusion
  // of a start of packet (Railcom AUX) marker before the preamble.
  // Since the PIO_MARKER_OFFSET is applied to the packet's end bit, it always runs
  // 1 packet behind. The first packet after a change therefore has incorrect timing.
  if (dccTx.railComActiveOnAux) {s.endbit.bits.durationB = (ONE - PIO_MARKER_OFFSET);}
    else s.endbit.bits.durationB = ONE;
  // Step 2: we must (re)create the Idle and Reset packets, to ensure the
  // start of packet (Railcom AUX) marker is in/excluded
  prepareIdlePacket();
  prepareResetPacket();
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
  // We can process a new packet provided we don't have to repeat earlier SM packets
  if (dccPacketEngine.isServiceModeRepeating() == false) {
    // Part 1: No SM repeating.
    if (dccTx.userPacketReady) {                                // set in send()
      // Part 1A: We received a new packet from the user
      // Switch ISR to the buffer that was just filled by send():
      // sendPacketIndex has already been toggled in send(), so "ready" is the other one.
      dccTx.isrPacketIndex = dccTx.sendPacketIndex ^ 1;
      transmitNextPacket(&dccTx.userPacket[dccTx.isrPacketIndex]);
      dccTx.userPacketReady = false;
      if (dccTx.isServiceModePacket) {                          // set in send()
        dccTx.smRepeatsRemaining = dccTx.smMaxRepeats;          // initialise
      }
    } else {
      // No new packet: send the default packet
      transmitNextPacket(dccTx.defaultPacket);
    }
  } else {
    // Part 2: We are in SM and have to repeat the message
    transmitNextPacket(&dccTx.userPacket[dccTx.isrPacketIndex]);
    // Decrement counter (Note: decrease is atomic operation)
    uint8_t repeats = dccTx.smRepeatsRemaining;
    if (repeats > 0) dccTx.smRepeatsRemaining = repeats - 1;
  }
  if (dccTx.smRepeatsRemaining == 0) dccPacketEngine.canAcceptPacket = true;
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
  // Step 1: Set the pinmux, such that "pin" will from now on be used by the PIO
  // If multiple pins should be used, pio_gpio_init should be called once per pin
  pio_gpio_init(pio, pin);
  // Step 2: Configure the pin. The parameter "1" represents the number of pins
  // abd the parameter "true" teels that the pin is used for output (false = input)
  pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
  // Step 3: Assigns to the variable pioConfig a default config, which may
  // subsequently be modified. "offset" is the PIO programs offset
  // dccPulseGenerator_program_get_default_config is a static inline function generated
  // by the pioasm (see toggle.pio.h)
  pio_sm_config pioConfig = dccPulseGenerator_program_get_default_config(offset);
  // Step 4: Configure the clock to tick every 1 us.
  // Use F_CPU to compensate for different clock speeds.
  float pio_clk = 1e6; // 1 MHz
  float clkdiv = (float)F_CPU / pio_clk;
  sm_config_set_clkdiv(&pioConfig, clkdiv);
  // Step 5: Specify which pins the PIO instruction "out pins, 1" will operate upon.
  // The parameter "pin" is the start pin, and "1" specifies the number of pins
  // Note that the PIO has four different pin-mappings, which can be configured
  // independently, for the following instructions: set pins, x / out pins, x /
  // in pins, x and mov pins, x
  sm_config_set_out_pins(&pioConfig, pin, 1);
  // Step 6: Configure the shift register
  // The variable "pull_threshold" specifies the number of bits to shift out before
  // auto/conditional re-pulling of the OSR. Since the PIO program performs 'out x, 32"
  // any value 1..32 should work. IN initial tests the value 32 didn't seem to work
  // whereas the value 1 did work
  bool shift_right = true;     // shift right
  bool autopull = true;
  //uint pull_threshold = 1;      // threshold in bits to shift out before auto/conditional re-pulling of the OSR
  uint pull_threshold = 32;
  sm_config_set_out_shift(&pioConfig, shift_right, autopull, pull_threshold);
  // Step 7: Initialse the state machine with the provided configuration
  pio_sm_init(pio, sm, offset, &pioConfig);
}

static void PioSmAuxInit(PIO pio, uint sm, uint offset, uint pin) {
  // See PioSmDccInit for details of every step
  pio_gpio_init(pio, pin);
  pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
  pio_sm_config pioConfig = railcomCutOut_program_get_default_config(offset);
  float pio_clk = 1e6; // 1 MHz
  float clkdiv = (float)F_CPU / pio_clk;
  sm_config_set_clkdiv(&pioConfig, clkdiv);
  // Specify which pins the PIO instruction "set pins" will operate upon.
  sm_config_set_set_pins(&pioConfig, pin, 1);
  pio_sm_init(pio, sm, offset, &pioConfig);
}

static void setupPio() {
  // Step 1: Select which PIO we will use. The RP2040 has pio0 and pio1, RP2350 has also pio2
  pioTx.pio = PIO_USED;
  // Step 2: On this PIO, ask which of the four state machines are available (0..3)
  // The parameter "true" says that this state machine will also be assigned
  pioTx.smDcc = pio_claim_unused_sm(pioTx.pio, true);
  pioTx.smAux = pio_claim_unused_sm(pioTx.pio, true);
  // Step 3: Load the PIO code of both state machine into this PIO's instruction memory
  // The start address is returned as parameter and assigned to "offset"
  uint dccOffset = pio_add_program(pioTx.pio, &dccPulseGenerator_program);
  uint auxOffset = pio_add_program(pioTx.pio, &railcomCutOut_program);
  // Step 4: Initialise  and start the DCC-pin state machine (true => enable, false => disable)
  PioSmDccInit(pioTx.pio, pioTx.smDcc, dccOffset, dccPacketEngine.dccRailPin);
  pio_sm_set_enabled(pioTx.pio, pioTx.smDcc, true);
  // Step 5: Initialise the AUX-pin state machine, but do not start it
  PioSmAuxInit(pioTx.pio, pioTx.smAux, auxOffset, dccPacketEngine.dccRailAuxPin);
}

static inline void setRailComActiveOnAux(bool auxHasCutOut) {
  // Step 1: Set the flag to include RailCom markers in the DCC signal
  dccTx.railComActiveOnAux = auxHasCutOut;
  // Step 2: Set the size of the DCC endbit and include markers in DCC packets
  createDccDefaultPackets();
  // Step 3: Enable / Disable the PIO State Machine for the AUX signal.
  // Connect the pin to either the PIO, or SIO (Single-Cycle IO = standard)
  if (auxHasCutOut) {
    pio_gpio_init(pioTx.pio, dccTx.auxPin);            // Attach PIN to PIO.
    pio_sm_restart(pioTx.pio, pioTx.smAux);
    pio_sm_set_enabled(pioTx.pio, pioTx.smAux, true);
  }
  else {
    pio_sm_set_enabled(pioTx.pio, pioTx.smAux, false);
    gpio_set_function(dccTx.auxPin, GPIO_FUNC_SIO);    // attach PIN to SIO
  }
  // Step 4: Since pio_gpio_init resets any inversion logic, check inversion again.
  if (dccTx.auxEnableLevel) setPinInverted(dccTx.auxPin, true);
}


//================================================================================================
//                                 E: DccPacketEngine class methods
//================================================================================================

//************************************************************************************************
// E1: setUp and Send
//************************************************************************************************
void DccPacketEngine::setupWaveformGenerator() {
  if (dccTx.isInitialized) return;
  // Step 1: Set default values
  initTxEngines();
  // Step2: create bit symbols and default packets
  createDccBits();
  createDccDefaultPackets();
  // Step 3: Setup the PIO and DMA that feeds the PIO
  setupPio();
  setupDma();
  // Step 4: Invert the DCC signal, if needed
  if (dccTx.dccSignalInverted) setPinInverted(dccPacketEngine.dccRailPin, true);
  // Step 5: Set up the Aux pin. Initialise without a RailCom gap on AUX. Invert, if needed.
  initAuxPin(dccPacketEngine.dccRailAuxPin);
  setRailComActiveOnAux(dccTx.railComAllowedOnAux);
  // Step 6: Initialization is done
  dccPacketEngine.canAcceptPacket = true;
  dccTx.isInitialized = true;
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
  pushMarker(packet.symbols, i);                  // Start of a new packet
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
// DRV8874 Inrush protection not implemented in this variant
void DccPacketEngine::startInrushMode(uint16_t ms) {(void)ms;}
bool DccPacketEngine::isInrushModeEnabled(void) {return false;}


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

// Enable / disable generation of a RailCom gap pulse on the Aux pin
void DccPacketEngine::setRailComGapInAux(bool active) {
  dccTx.railComAllowedOnAux = active;
  setRailComActiveOnAux(active);
}

// This routine allows higher-layer software to determine whether a RailCom gap
// is currently in progress, so that rail voltage and/or current measurements
// can be safely controlled. In this implementation we check the value of the
// output pin via gpio_get(pin).
bool DccPacketEngine::railComGap(void) {
  dccTx.railComCutoutInProgress = false;
  if (dccTx.railComAllowedOnAux && dccTx.auxOutputActive) {
    bool pinLevel = gpio_get(dccTx.auxPin);
    if (dccTx.auxEnableLevel == 1) {dccTx.railComCutoutInProgress = !pinLevel;}
      else dccTx.railComCutoutInProgress = pinLevel;
  }
  return dccTx.railComCutoutInProgress;
}


//******************************************************************************
// E5: Start / Stop interface methods
//******************************************************************************
// This HQ driver code enables (disables) the signal on the AUX pin. The pin
// can therefore be used to enable (disable) a connected H-Bridge. The
// polarity of the enable signal can be inverted by using setAuxActiveLevel().
// The AUX signal may include the RailCom cutout signal; the gap has the
// same polarity as the disable level.
// The DCC output will never be disabled, allowing switches and other
// accessories to remain operational.

// When StopOutputSignal() is called, the following steps are required:
// - Check dccTx.auxOutputActive and return immediately if the AUX output
//   is already disabled. This flag is used to coordinate RunOutputSignal()
//   and StopOutputSignal().
// - Disable / enable the PIO State Machine for the AUX signal (RC cutout).
// - Set the AUX pin to the requested level. There are multiple options:
///  1) Use digitalWriteFast(). This call doesn't reset the pin's inversion
//      logic: don't use auxEnableLevel, since this would invert twice
//   2) Use digitalWrite(). This call reset the pin's inversion logic:
//      use auxEnableLevel
void DccPacketEngine::StopOutputSignal() {
  if (!dccTx.auxOutputActive) return;
  dccTx.auxOutputActive = false;                        // Flag RunOutputSignal
  setRailComActiveOnAux(false);                         // Stop the PIO SM
  digitalWriteFast(dccTx.auxPin, HIGH);                 // AUX output disabled
  // digitalWriteFast(dccTx.auxPin, !dccTx.auxEnableLevel);
  dccTx.railComCutoutInProgress = false;
}

void DccPacketEngine::RunOutputSignal() {
  if (dccTx.auxOutputActive) return;
  dccTx.auxOutputActive = true;                         // Flag StopOutputSignal
  digitalWriteFast(dccTx.auxPin, LOW);                  // AUX output Enabled
  // digitalWrite(dccTx.auxPin, dccTx.auxEnableLevel);
  if (dccTx.railComAllowedOnAux) setRailComActiveOnAux(true); // Restart AUX SM
}


//******************************************************************************
// E6: Setters to change certain aspects of the DCC encoder
//******************************************************************************
void DccPacketEngine::setDccSignalInverted(bool inverted) {
  // Inverts the polarity of the DCC signal. No impact on AUX signal
  dccTx.dccSignalInverted = inverted;
  setPinInverted(dccPacketEngine.dccRailPin, inverted);
}

void DccPacketEngine::setAuxActiveLevel(bool activeHigh) {
  dccTx.auxEnableLevel = activeHigh;
  setPinInverted(dccTx.auxPin, activeHigh);
}

void DccPacketEngine::setPreambleLength(uint8_t value) {
  // RCN-211 (23.07.2023): Eine Zentrale muss mindestens 17 Synchronbits senden
  if (value < 17) return;
  dccTx.preambleLength = value;      // Length of normal preamble
  createDccDefaultPackets();         // Reset and Idle packet must be regenerated
}

void DccPacketEngine::setPreambleLengthSM(uint8_t value) {
  // RCN-216 (24.11.2025): im Programmiermodus mindestens 20 Synchronbits
  if (value < 20) return;
  dccTx.preambleLengthSM = value;    // Length of preamble in Service Mode
  createDccDefaultPackets();         // Reset and Idle packet must be regenerated
}
