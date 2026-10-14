//************************************************************************************************
//
// File:            DCCHardware_RP.inc - TMC variant
// Purpose:         DCC Waveform Generator for RP2040 / RP2350 using PIO + DMA
// Copyright 2026:  Aiko Pras
// History:         2026-03-30 / Aiko Pras
//                  2026-09-22 (AP) - Adjusted for SDK version >= 2 and 4 MHz clock
//                  2026-09-23 (AP) - Inrush protection added, to prevent DRV887x OCP
//                  2026-10-14 (AP) - Changed from PWM to PH/EN mode. Inrush removed.
//                                    Monitor pin no longer supported (two pins only).
//                                    PIO clock back from 4 MHz to 1 MHz.
//
// Resources:
// - Pins: Two consecutive GPIO pins that can be controlled by a PIO state machine
// - PIO: one PIO block with one state machine
// - DMA: one DMA channel to feed the PIO TX FIFO
//
// Pin usage
// =========
// This driver is intended for DRV887x H-bridges in PH/EN mode:
// - dccRailPin    => PH: carries the DCC waveform
// - dccRailAuxPin => EN: LOW (inactive) during the RailCom gap and after StopOutputSignal()
// The PIO drives both pins as one consecutive group, starting with EN. Therefore
// dccRailAuxPin must be dccRailPin - 1; otherwise both pins are set to 0xFF.
// On the RP2350B both pins must lie within GPIO 0..31 or GPIO 16..47.
// dccMonitorPin is not supported and will be ignored.
//
// Design choices
// ==============
// This driver uses the RP2040 / RP2350 PIO (Programmable IO) and DMA to generate a
// precise, jitter-free DCC waveform.
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
//   - Logical '1' → two half-bits of 58 us
//   - Logical '0' → two half-bits of 100 us
//
// Each symbol holds one DCC bit: two phases (A and B), each with a level for both pins
// (2 bits: EN and PH) and a duration (14 bits). See DCCHardware_RP-PIO.inc.
// Symbols are precomputed and stored in packet buffers, similar to the ESP32 RMT.
//
// Special symbols:
//   - endbit: the DCC end bit
//   - gap: 29 us DCC (first half of the cutout start bit), then 435 us cutout
// The gap thus starts 29 us and ends 464 us after the end bit
// (RCN-217: TCS 26..32 us, TCE 454..488 us).
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
// With setRailCom(true) the gap is generated on both pins. EN is then LOW, so the
// DRV887x brakes (both outputs low) and shorts the track, as required by RCN-217.
// setRailComGapInAux() has no function in this driver.
//
// Service Mode
// ============
// Service Mode packets:
// - Use a longer preamble
// - Have no RailCom gap
// - Are automatically repeated (configurable count)
// - Temporarily replace idle packets with reset packets
//
// The ISR manages repetition without CPU intervention in the main loop.
//
// Call order
// ==========
// setupWaveformGenerator() must be called first, before any other method of this class.
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
#define MHZ_PIO    1                  // PIO clock speed, in MHz
#define PIO_OFFSET 4                  // Overhead induced by the PIO instructions, in PIO cycles

// Convert a duration in microseconds to a PIO N value
#define US_TO_PIO(us) ((us) * MHZ_PIO - PIO_OFFSET)

#define ZERO US_TO_PIO(100)           // RCN-210: zero half bit   (95.... us)
#define ONE  US_TO_PIO( 58)           // RCN-210: one half bit    (55..61 us)
#define TCS  US_TO_PIO( 29)           // RCN-217: cutout start    (26..32 us)
#define TTS1 US_TO_PIO( 80)           // RCN-217: start channel 1 (Unused by the current code)
#define TTS2 US_TO_PIO(193)           // RCN-217: start channel 2 (Unused by the current code)

#define MAX_SYMBOLS 128               // Long enough to hold a 6 byte DCC packet
#define RAILCOM_CUTOUT_LENGTH 4       // 4 Preamble bits. See RCN-217 (Ausgabe 24.11.2025)
#define TGAP US_TO_PIO(RAILCOM_CUTOUT_LENGTH * 116 - 29)   // Cutout after TCS: 435 us

#define PIO_USED pio0                 // Which PIO will we use?

// Longest possible packet: preamble + 6 bytes (start bit + 8 bits each) + end bit
#define MAX_PREAMBLE (MAX_SYMBOLS - (MaxDccSize * 9) - 2)   // = 72


//================================================================================================
//                            A: Objects and Internal Data Structures
//================================================================================================

//************************************************************************************************
// A.1. Constructor definition and instantiation
//************************************************************************************************
DccPacketEngine::DccPacketEngine(){
  // Default pins for the TMC-LZ210 board (RP2350B).
  // Normal usage: the application overwrites these values before setup (setupWaveformGenerator)
  dccRailPin = 33;                             // PH: DCC signal              (GPIO33 => IN2/PH)
  dccRailAuxPin = 32;                          // EN: Enable and RailCom gap  (GPIO32 => IN1/EN)
  dccMonitorPin = 0xFF;                        // Not supported in this driver
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
        uint32_t levelA    : 2;    // use 2 bits
        uint32_t durationA : 14;   // use 14 bits
        uint32_t levelB    : 2;
        uint32_t durationB : 14;
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
  dcc_waveform_symbol_t gap;    // Railcom gap: 29 us cutout start, then 435 us cutout
};
static DccSymbols s;            // Short name, to improve readability in (inline) helper functions

struct DccTxEngine {                      // Processor independent
  bool isInitialized = false;             // Set by setupWaveformGenerator()
  // User Packet
  DccPacket userPacket[2];                // packet ping-pong buffer
  uint8_t sendPacketIndex;                // owned by send()
  volatile uint8_t isrPacketIndex;        // owned by DMA-ISR
  volatile bool userPacketReady;          // send() => DMA-ISR handover flag
  bool dccOutputActive = false;           // Flag used by RunOutputSignal / StopOutputSignal
  // Prepared packets
  DccPacket idlePacket;                   // Precomputed idle packet
  DccPacket resetPacket;                  // Precomputed reset packet
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
  // Polarity of the DCC and Enable signal
  bool dccSignalInverted = false;         //
  bool auxEnableLevel = true;             // Level of EN once enabled (DRV887x: HIGH)
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
// preambleLengthSM, dccSignalInverted and auxEnableLevel since the user may
// already have set these values.
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
  // Step 1: Determine for both halfbits the levels of EN (bit 0) and PH (bit 1)
  uint8_t en = dccTx.dccOutputActive ? 0b01 : 0b00;   // EN on/off
  uint8_t levelA   = 0b10 | en;                       // PH=ON,  EN=on/off
  uint8_t levelB   = 0b00 | en;                       // PH=OFF, EN=on/off
  uint8_t levelGap = 0b00;                            // PH=OFF, EN=OFF (brake)
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
  // RailCom gap: first half of the cutout start bit, followed by the cutout
  s.gap.bits.levelA = levelA;
  s.gap.bits.durationA = TCS;
  s.gap.bits.levelB = levelGap;
  s.gap.bits.durationB = TGAP;
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
      buf[i++] = s.gap;               // Cutout start bit + three preamble bits
      length = length - RAILCOM_CUTOUT_LENGTH;
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
  // Priority 1: SM was entered earlier and the SM packet must still be repeated
  if (dccPacketEngine.isServiceModeRepeating()) {
    transmitNextPacket(&dccTx.userPacket[dccTx.isrPacketIndex]);
    // Decrement counter (Note: decrease is atomic operation)
    uint8_t repeats = dccTx.smRepeatsRemaining;
    if (repeats > 0) dccTx.smRepeatsRemaining = repeats - 1;
  }
  // Priority 2: normal case. Send a new user packet, or idle/reset
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
  // Step 1: Assigns to the variable pioConfig a default config, which may
  // subsequently be modified. "offset" is the PIO programs offset
  // dccPulseGenerator_program_get_default_config is a static inline function generated
  // by the pioasm (see toggle.pio.h)
  pio_sm_config pioConfig = dccPulseGenerator_program_get_default_config(offset);
  // Step 2: Configure the GPIO pins so they are controlled by the PIO instead of the SIO
  // Since we use multiple pins, pio_gpio_init should be called once per pin.
  // Note that we MUST use consecutive GPIO pins (EN, PH)

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
  // Step 5: Configure the clock to tick every 1 us.
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

static void setupPio() {
  // Step 1: Select which PIO we will use. The RP2040 has pio0 and pio1, RP2350 has also pio2
  pioTx.pio = PIO_USED;
  // Step 2: The PIO can only address 32 consecutive GPIOs at a time. That is okay for the RP2040.
  // However, the RP2350 has 48 pins. If we use a RP2350, we must select via gpio_base = 0
  // (pins 0..31) or gpio_base = 16 (pins 16..47) which pins we use.
  // Both pins we use must fit within a single such window, so we choose the window based on
  // the highest of these two pins, which is dccRailPin.
  if ((dccPacketEngine.dccRailPin) <= 31) pio_set_gpio_base(pioTx.pio, 0);
    else pio_set_gpio_base(pioTx.pio, 16);
  // Step 3: On this PIO, ask which of the four state machines are available (0..3)
  // The parameter "true" says that this state machine will also be assigned
  pioTx.smDcc = pio_claim_unused_sm(pioTx.pio, true);
  // Step 4: Load the PIO code of the state machine into this PIO's instruction memory
  // The start address is returned as parameter and assigned to "offset"
  uint dccOffset = pio_add_program(pioTx.pio, &dccPulseGenerator_program);
  // Step 5: Initialise  and start the DCC-pin state machine (true => enable, false => disable)
  PioSmDccInit(pioTx.pio, pioTx.smDcc, dccOffset, dccPacketEngine.dccRailAuxPin);
  pio_sm_set_enabled(pioTx.pio, pioTx.smDcc, true);
}


//================================================================================================
//                                 E: DccPacketEngine class methods
//================================================================================================

//************************************************************************************************
// E1: setUp and Send
//************************************************************************************************
// As part of setup(), the packet scheduler code should call setupWaveformGenerator.
// The dccRailPin number must be one higher than the dccRailAuxPin number.
// If not, setupWaveformGenerator returns with both pins set to 0xFF
void DccPacketEngine::setupWaveformGenerator() {
  if (dccTx.isInitialized) return;
  if ((dccRailPin < NUM_BANK0_GPIOS) && (dccRailPin == dccRailAuxPin + 1)) {
    // Step 1: Set default values
    initTxEngines();
    // Step2: create bit symbols and default packets
    createDccDefaultPackets();
    // Step 3: Setup the PIO and DMA that feeds the PIO
    setupPio();
    setupDma();
    // Step 4: Restore pin polarities (pio_gpio_init resets them)
    setDccSignalInverted(dccTx.dccSignalInverted);
    setAuxActiveLevel(dccTx.auxEnableLevel);
    // Step 5: Initialization is done
    dccTx.isInitialized = true;
    dccPacketEngine.canAcceptPacket = true;
  }
  else {   // pin numbers are invalid
    dccRailPin = 0xFF;
    dccRailAuxPin = 0xFF;
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
// Inrush protection is no longer implemented for this variant
// The empty stubs will be removed later
void DccPacketEngine::enterInrushMode(void) {}
void DccPacketEngine::leaveInrushMode(void) {}
bool DccPacketEngine::isInrushModeEnabled(void) {return false;}
void DccPacketEngine::useMonitorPinforInrush(bool value) {(void)value;}


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
// EN pin via gpio_get_all64().
// During the gap EN is at its disable level (LOW for the DRV887x), while track
// power is on (dccOutputActive). gpio_get_all64() reads the pin level after
// inversion, so we compare with auxEnableLevel.
bool DccPacketEngine::railComGap(void) {
  if (!dccTx.isInitialized || !dccTx.dccOutputActive) return false;
  bool cutoutDetected = ((gpio_get_all64() >> dccRailAuxPin) & 1) != dccTx.auxEnableLevel;
  if (cutoutDetected) {  // To avoid glitches, take a second sample
    cutoutDetected = ((gpio_get_all64() >> dccRailAuxPin) & 1) != dccTx.auxEnableLevel;
  }
  return cutoutDetected;
}


//******************************************************************************
// E5: Start / Stop interface methods
//******************************************************************************
// StopOutputSignal() and RunOutputSignal() switch the H-bridge off and on via EN.
// Both change dccOutputActive and call createDccDefaultPackets(), which rebuilds the
// symbols: after StopOutputSignal() EN is at its disable level (LOW for the DRV887x),
// after RunOutputSignal() at its enable level. PH keeps carrying DCC in both cases.
// Packets that were already built (the current packet, a pending user packet and its
// Service Mode repeats) keep their old EN level.
void DccPacketEngine::StopOutputSignal() {
  if (!dccTx.dccOutputActive) return;
  dccTx.dccOutputActive = false;
  createDccDefaultPackets();
}

void DccPacketEngine::RunOutputSignal() {
  if (dccTx.dccOutputActive) return;
  dccTx.dccOutputActive = true;
  createDccDefaultPackets();
}

//******************************************************************************
// E6: Setters to change certain aspects of the DCC encoder
//******************************************************************************
void DccPacketEngine::setDccSignalInverted(bool inverted) {
  // Inverts the polarity of the DCC signal (PH). No impact on the Enable signal.
  dccTx.dccSignalInverted = inverted;
  setPinInverted(dccPacketEngine.dccRailPin, inverted);
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

void DccPacketEngine::setAuxActiveLevel(bool activeHigh) {
  // The PIO generates EN active-high; invert only for active-low hardware
  bool activeLow = !activeHigh;
  dccTx.auxEnableLevel = activeHigh;
  setPinInverted(dccRailAuxPin, activeLow);
}

void DccPacketEngine::setRailComGapInAux(bool active) {(void)active;}
// Has no function in TMC mode.
// With setRailCom(true) the gap is always generated on both PH and EN.
// EN LOW is the only way to ensure that the DRV887x puts a brake signal on the tracks,
// as required by RCN217. If PH stays 435 us HIGH or LOW, a DC offset will be the result
