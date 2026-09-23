//******************************************************************************************************
//
// File:            DCCHardware_common.inc
// Purpose:         DCC Waveform Generator - Processor independent include file
// Copyright 2026:  Aiko Pras
// History:         2026-01-17 / Aiko Pras - First version
//
//
// Historical credits:
// - Wolfgang Kufer (OpenDCC, http://opendcc.de)
//     * Original DCC waveform generation concepts and basic routines
//     * Provides timing calculations for '1' and '0' bits
//
// - Don Goodman-Wilson (Railstars CmdrArduino)
//     * Adapted OpenDCC waveform generation for Arduino
//     * Packet scheduling logic
//     * Initial integration with Arduino timers and interrupts
//
// - Philipp Gahtow (DCCInterfaceMaster, https://github.com/Digital-MoBa/DCCInterfaceMaster)
//     * Rewrote waveform generator for multiple MCU platforms (Arduino/ESP)
//     * Modified ISR and preamble handling
//     * Added support for service mode, RailCom cutouts and 3 DCC output pins
//
//
// This file contains the processor-independent core of the DCC waveform generator.
// It implements the logical DCC protocol flow as an interrupt-driven state machine,
// but makes no assumptions about timers, registers, ports, interrupt vectors, or
// the mechanism used to toggle the rail outputs. All hardware-specific aspects
// (timer configuration, ISR binding, interrupt prologue, and physical pin control)
// are supplied by the including driver file.
//
// The state machine models the complete DCC bitstream, including preamble generation,
// start, data and end bits, Service Mode behaviour and RailCom cutouts. The actual rail
// signals may be generated either by software-controlled pin toggling (bit-banging)
// or by hardware peripherals such as timer compare units.
//
// A fundamental design choice is the explicit distinction between the first and second
// halfbit of every DCC bit. During the first halfbit, output transitions are triggered
// (or delegated to hardware) and the timing for the next halfbit is configured. During
// the second halfbit, the timing remains unchanged and only the logical DCC state
// machine is advanced. This separation allows accurate bit timing, safe control of
// complementary rail outputs, and a clean split between physical signal generation
// and protocol flow.
//
// Toggling strategy and H-bridge safety:
// - All rail transitions use toggles rather than set/clear to maintain
//   consistent behavior between software- and hardware-generated approaches.
// - When both normal and inverted rail outputs are enabled, their toggles are
//   executed back-to-back to minimize temporal offset and produce the cleanest
//   zero-crossing of the H-bridge output signal.
//   Therefore, the toggle order within each half-bit is critical.
//
// Service Mode packet retransmissions are handled inside the ISR rather than in the
// scheduler. This ensures that repeats are tightly coupled to the actual transmission
// flow and allows higher software layers to detect acknowledgements reliably, without
// being affected by scheduler timing, buffering, or packet queue latency.
//
// A detailed description of the state machine, its states, and their transitions can
// be found in: extras/variants-Z21PG/Statemachine.
//
//******************************************************************************************************
//******************************************************************************************************
// 1. Defines
//******************************************************************************************************
#define RAILCOM_CUTOUT_LENGTH 4	  	// see RCN-217 (Ausgabe 24.11.2025)


//******************************************************************************************************
// 2. Declaration and initialisation (constructor) of external object
//******************************************************************************************************
// Constructor definition
DccPacketEngine::DccPacketEngine(){
  dccRailPin = 6;                                 // Pin for DCC rail Signal, with RailCom support
  dccRailAuxPin = 0xFF;	                          // Pin for DCC rail Signal, with RailCom support. Inverted
  dccMonitorPin = 0xFF;	                          // Pin for a continuous DCC Signal, ignoring the RailCom gap / Railpower
  canAcceptPacket = true;                         // ISR is ready to receive the first packet from the user update() code
}

// dccPacketEngine is the object that controls packet transmission for different hardware
DccPacketEngine dccPacketEngine;


//******************************************************************************************************
// 3. Declaration and initialisation of internal data structures
//******************************************************************************************************
// Enumerated type describing the ISR state machine for DCC signal generation.
// Each state spans one or more DCC bits (2 × 58 µs for a '1', or 2 × 100 µs for a '0').
// A DCC bit consists of two halfbits; the flag 'firstHalfBit' indicates which half is active.
// This allows the ISR to control timing, evaluate state transitions, and update variables.
// The dccRailComStartBit is a special case: it is logically a '1' bit,
// but the signal is suppressed halfway through the first halfbit (after 29 µs).
// Therefore, the first halfbit of the RailCom start bit is split into two phases.
// The flag 'firstHalfBitRC' indicates which phase of this RailCom halfbit is active.
// The state 'dccPreambleStartBit' is used only immediately after the RailCom gap to
// reset variables and reinitialize the DCC bit stream.
typedef enum  {
  dccRailComStartBit,                 // 29us pulse to indicate the start of the gap
  dccRailComGap,                      // Together with the RailCom startbit, a gap of four 1s
  dccPreambleStartBit,                // First bit of the preamble, Always 1
  dccPreamble,                        // Sequence of 1 bits
  dccStartBitPacket,                  // Bit between the preamble and first dccData byte. Always 0
  dccStartBitData,                    // Bit between dccData bytes. Always 0
  dccData,                            // Eight data bits. Each bit is a 0 or 1
  dccEndBit                           // Last bit of the DCC packet. Always 1
} DCC_output_state_t;

// Define the structure for all variables used by the ISR
// The inital values for preambleLength is taken from RCN-211 (Ausgabe 23.07.2023)
// The inital values for preambleLengthSM is taken from RCN-216 (Ausgabe 24.11.2025)
struct DCC_ISR_State {
  volatile DCC_output_state_t state;             // Initial state of the ISR state machine
  volatile uint8_t bitsRemaining;                // Bits remaining in current phase (preamble or data byte)
  volatile uint8_t bytesRemainingInPacket;       // How many bytes left to send
  volatile uint8_t nextPacket[MaxDccSize];       // Next packet waiting to be sent
  volatile uint8_t nextPacketSize;               // Number of bytes in next packet
  volatile uint8_t currentPacket[MaxDccSize];    // Current packet being sent
  volatile uint8_t currentPacketSize;            // Number of bytes in current packet
  volatile uint8_t defaultPacket[3];             // Pre-generated default IDLE packet (Reset for SM)
  volatile uint8_t defaultPacketSize;            // Always 3 bytes
  volatile bool firstHalfBit;	                   // The ISR starts with the first DCC Halfbit
  volatile bool firstHalfBitRC;	                 // And the first part of the Railcom start bit
  volatile bool smActive;                        // SM active (long preamble, resets, SM flow allowed)
  volatile bool isServiceModePacket;             // We have to send Service Mode Packets
  volatile uint8_t smRepeatsRemaining;           // Number of retransmissions left for this SM packet
  volatile uint8_t smMaxRepeats;	               // Number of SM retransmissions after first send
  volatile bool railComCutoutInProgress;	       // In the gap, this flag is guaranteed to be true
  volatile bool railCom;                         // Should we generate a RailCom gap?
  volatile bool railPower;	                     // Controls if the railsignal is on/off
  volatile uint8_t preambleLength = 17;          // Length of the preamble in normal operation
  volatile uint8_t preambleLengthSM = 20;        // Length of the preamble in Service Mode
  bool dccSignalInverted = false;                // Polarity of the DCC signal
};

static DCC_ISR_State dccISR;                     // Create an instance of the struct for the ISR

static void initIsr() {
  // Note that we do not (re)set railCom, railPower, preambleLength, preambleLengthSM
  // and dccSignalInverted, since the user may already have set these values.
  dccISR.state = dccPreambleStartBit;            // We start in the dccPreambleStartBit state
  dccISR.bitsRemaining = dccISR.preambleLength;  // With a normal preamble (thus SM is false)
  dccISR.bytesRemainingInPacket = 0;             // We start with an empty packet
  dccISR.firstHalfBit = true;	                   // We must start with the first DCC Halfbit
  dccISR.firstHalfBitRC = true;	                 // And the first part of the Railcom start bit
  // Service Mode settings
  dccISR.smActive = false;
  dccISR.isServiceModePacket = false;
  dccISR.smRepeatsRemaining = 0;
  dccISR.smMaxRepeats = 10;
  // RailCom settings
  dccISR.railComCutoutInProgress = false;
  // Packet initialisation
  for (uint8_t i = 0; i < MaxDccSize; ++i) dccISR.currentPacket[i] = 0;
  dccISR.currentPacketSize = 0;
  for (uint8_t i = 0; i < MaxDccSize; ++i) dccISR.nextPacket[i] = 0;
  dccISR.nextPacketSize = 0;
  dccISR.defaultPacket[0] = 0xFF;
  dccISR.defaultPacket[1] = 0x00;
  dccISR.defaultPacket[2] = 0xFF;
  dccISR.defaultPacketSize = 3;
}


//******************************************************************************
// 4. Setup the Interrupt timer and configure the output pins
//******************************************************************************
void DccPacketEngine::setupWaveformGenerator() {
  // Step 1: stop de DCC Interrupt timer
  // Prevents the ISR from running during hardware reconfiguration.
  disableDccTimer();
  // Step 2: Maps DCC output signals to MCU port registers and bitmasks.
  // Unused pins (0xFF) are explicitly disabled to prevent ISR pin access.
  initDccPin(dccRail, dccRailPin);
  initDccPin(dccRailInv, dccRailAuxPin);
  initDccPin(dccMonitor, dccMonitorPin);
  // Step 3: Configure the DCC timer. This prepares the timer registers and
  // interrupt source, but does not yet start the timer / DCC signal generation.
  enableDccTimer();
  // Step 4: Force all DCC output pins into a known safe initial state.
  // Normal rail starts LOW, inverted rail HIGH (complementary),
  // monitor output LOW. No toggling is allowed yet.
  if (!dccISR.dccSignalInverted) {
    DCCOUT_LOW;
    DCCINV_HGH;
    DCCMON_LOW;
  }
  else {
    DCCOUT_HGH;
    DCCINV_LOW;
    DCCMON_HGH;
  }
  // Step 5: Enable toggling of the DCC output channels.  From this point on,
  // the ISR is allowed to toggle these pins according to the DCC state machine.
  ENABLE_DCCOUT;
  ENABLE_DCCINV;
  ENABLE_DCCMON;
  // Step 6: Reinitialise the DCC state machine.
  // This resets all ISR state so the first interrupt starts in a
  // well-defined condition (dccPreambleStartBit with a '1'-bit preamble).
  initIsr();
  // Step 7: Start the DCC timer and enable interrupt-driven waveform generation.
  // From this point on, the ISR takes full control of timing and pin toggling.
  startDccTimer();
}

//******************************************************************************
// 5. Packet send interface method
//******************************************************************************
void DccPacketEngine::send(const uint8_t* data, uint8_t size) {
  if (size == 0 || size > MaxDccSize) return;   // Illegal packet size
  // Check if this is a SM packet
  // RCN-216: Service Mode packets have 0111xxxx as first byte
  dccISR.isServiceModePacket = dccISR.smActive && ((data[0] & 0xF0) == 0b01110000);
  // Copy the received data to nextPacket
  noInterrupts();
  for (uint8_t i = 0; i < size; i++) dccISR.nextPacket[i] = data[i];
  dccISR.nextPacketSize = size;
  dccPacketEngine.canAcceptPacket = false;      // ISR must first copy Packet
  interrupts();
}

//******************************************************************************
// 6. Service Mode interface methods
//******************************************************************************
void DccPacketEngine::enterServiceMode(void) {
  // Enable Service Mode behaviour (long preamble, reset packets, SM flow).
  dccISR.smActive = true;
  // Pre-generate the reset packet (to make the ISR faster)
  noInterrupts();
  dccISR.defaultPacket[0] = 0;
  dccISR.defaultPacket[1] = 0;
  dccISR.defaultPacket[2] = 0;
  interrupts();
}

void DccPacketEngine::leaveServiceMode(void) {
  // Leave Service Mode immediately. Ongoing SM retransmissions are cancelled.
  dccISR.smActive = false;
  dccISR.smRepeatsRemaining = 0;
  // Pre-generate the idle packet (to make the ISR faster)
  noInterrupts();
  dccISR.defaultPacket[0] = 0xFF;
  dccISR.defaultPacket[1] = 0;
  dccISR.defaultPacket[2] = 0xFF;
  interrupts();
}

bool DccPacketEngine::isServiceModeEnabled(void) {
  // Returns true if the system is currently in Service Mode.
  return dccISR.smActive;
}

void DccPacketEngine::setServiceModeMaxRepeats(uint8_t value) {
  dccISR.smMaxRepeats = value;
}

bool DccPacketEngine::isFirstServiceModePacket(void) {
  // True only for the first transmission of a new SM packet.
  return dccISR.smActive && (dccISR.smRepeatsRemaining == dccISR.smMaxRepeats);
}

bool DccPacketEngine::isServiceModeRepeating(void) {
  // True while the ISR is busy retransmitting the current SM packet.
  return dccISR.smActive && (dccISR.smRepeatsRemaining > 0);
}

void DccPacketEngine::stopServiceModeRepeats(void) {
  // Stop retransmission of the current SM packet (e.g. after ACK).
  dccISR.smRepeatsRemaining = 0;
}


//******************************************************************************
// 7. Inrush Protection Mode interface methods
//******************************************************************************
// Inrush protection is not implemented for this variant: empty stubs
void DccPacketEngine::enterInrushMode(void) {}
void DccPacketEngine::leaveInrushMode(void) {}
bool DccPacketEngine::isInrushModeEnabled(void) {return false;}


//******************************************************************************
// 8. RailCom interface methods
//******************************************************************************
void DccPacketEngine::setRailCom(bool active) { // Enable / disable generation of the RailCom gap
  dccISR.railCom = active;
}

bool DccPacketEngine::getRailCom(void) { // Is generation of the RailCom gap enabled?
  return dccISR.railCom;
}

// This routine allows higher-layer software to determine whether a RailCom gap
// is currently in progress, so that rail voltage and/or current measurements
// can be safely controlled.
// To avoid measurement errors, such measurements must not be performed during
// a RailCom gap. Therefore, a conservative signaling approach is used.
// The start of the gap is indicated immediately when the RailCom start bit begins,
// while the end of the gap is indicated slightly after the actual gap.
// This is implemented using the boolean flag 'railComCutoutInProgress', which
// is set at the start of the RailCom start bit and cleared at the first bit of
// the subsequent preamble.
bool DccPacketEngine::railComGap(void) {
  return dccISR.railComCutoutInProgress;
}


//******************************************************************************
// 9. Start / Stop interface methods
//******************************************************************************
void DccPacketEngine::StopOutputSignal() {
  // This routine may be called when a short circuit is detected or when the user
  // presses the emergency stop button. In that case the normal and inverted DCC
  // outputs must stop toggling and be forced LOW, so no DCC power is present on
  // the rails. The monitor output is not disabled, allowing switches and other
  // accessories to remain operational.
  noInterrupts();
  DCCOUT_LOW;
  DCCINV_LOW;
  DISABLE_DCCOUT;
  DISABLE_DCCINV;
	dccISR.railPower = false;
  interrupts();
}

void DccPacketEngine::RunOutputSignal() {
  // Enabling power is delicate, because we must ensure that the normal and
  // inverted DCC signals always remain complementary and are never high at the
  // same time. For this reason, toggling is enabled at the packet end bit:
  // during the first half-bit the inverted signal is allowed to toggle again,
  // and during the second half-bit the normal signal is re-enabled.
	dccISR.railPower = true;
}


//******************************************************************************
// 10. Setters to change certain aspects of the DCC encoder
//******************************************************************************
void DccPacketEngine::setPreambleLength(uint8_t value) {
  // RCN-211 (23.07.2023): Eine Zentrale muss mindestens 17 Synchronbits senden
  if (value < 17) return;
  dccISR.preambleLength = value;      // Length of normal preamble
}

void DccPacketEngine::setPreambleLengthSM(uint8_t value) {
  // RCN-216 (24.11.2025): im Programmiermodus mindestens 20 Synchronbits
  if (value < 20) return;
  dccISR.preambleLengthSM = value;      // Length of preamble in Service Mode
}

void DccPacketEngine::setDccSignalInverted(bool inverted) {
  // Changes the polarity of the DCC signal(s)
  if (dccISR.dccSignalInverted == inverted) return;
  dccISR.dccSignalInverted = inverted;
  dccPacketEngine.setupWaveformGenerator();  // restart
}

// The following functions are not used by the Z21PG variants
void DccPacketEngine::setAuxActiveLevel(bool activeHigh) {(void)activeHigh;}
void DccPacketEngine::setRailComGapInAux(bool useForRcCutout) {(void)useForRcCutout;}
void DccPacketEngine::useMonitorPinforInrush(bool value) {(void)value;}


//******************************************************************************
// 11. Inline functions for bit toggle
//******************************************************************************
// First half-bit toggle
// During the first half-bit, the normal rail output starts LOW and the
// inverted rail output starts HIGH. To maintain proper H-bridge operation
// and clean zero-crossing:
//   - When both rails are enabled, the inverted rail is toggled first,
//     immediately followed by the normal rail.
//   - The monitor output is toggled if enabled.
// This order is critical to avoid shoot-through.
inline void dccFirstHalfBitToggle(void) {
  if (DCCOUT_IS_ENABLED && DCCINV_IS_ENABLED) {
    DCCINV_TGL;
    DCCOUT_TGL;
  } else {
    if (DCCINV_IS_ENABLED) DCCINV_TGL;
    if (DCCOUT_IS_ENABLED) DCCOUT_TGL;
    }
  if (DCCMON_IS_ENABLED) DCCMON_TGL;
}

// Second half-bit toggle
// During the second half-bit, the normal rail output starts HIGH and the
// inverted rail output starts LOW.
// To maintain proper H-bridge operation and clean zero-crossing:
//   - When both rails are enabled, the normal rail is toggled first,
//     immediately followed by the inverted rail.
//   - The monitor output is toggled if enabled.
// This order is critical to avoid shoot-through.
inline void dccSecondHalfBitToggle(void) {
  if (DCCOUT_IS_ENABLED && DCCINV_IS_ENABLED) {
    DCCOUT_TGL;
    DCCINV_TGL;
  } else {
    if (DCCOUT_IS_ENABLED) DCCOUT_TGL;
    if (DCCINV_IS_ENABLED) DCCINV_TGL;
    }
  if (DCCMON_IS_ENABLED) DCCMON_TGL;
}


//******************************************************************************
//**********************************  ISR   ************************************
//******************************************************************************
// This is the Interrupt Service Routine (ISR).
// The ISR is entered everytime the timer expires.
// Thus every 100us (0-bit), 58us (1-bit or 29us (RailCom start bit)

ISR_START {

  ISR_PROLOGUE;                 // Processor specific code may be here
  uint8_t mask;                  // helper to determine which bit we are
  //
  // ===========================================================================
  // PART 1: FIRST HALFBIT: TOGGLE PINS AND CHANGE TIMERS (IF NEEDED)
  // ===========================================================================
  if (dccISR.firstHalfBit) {
    // At the start of the first half-bit, the normal rail output is low while
    // the inverted rail output is high. When pin transitions are generated
    // in software, the required state change is performed explicitly here
    // by toggling the outputs.
    // In the hardware-assisted approach (toggle-on-compare-match),
    // dccFirstHalfBitToggle() is implemented as an empty inline function,
    // as the required transition is handled automatically by the timer.
    dccFirstHalfBitToggle();

    switch (dccISR.state) {

      case dccRailComStartBit:
        if (dccISR.firstHalfBitRC) {        // Is this the first part of the RailCom halfbit?
          TMR_RAILCOM_STARTBIT;             // Set the timer, to trigger after 29us
          DISABLE_DCCINV;                   // The Inverse pin will not toggle during the RC gap
          DISABLE_DCCMON;                   // The Monitor pin will skip the toggle after 29us
          dccISR.railComCutoutInProgress = true;  // The RailCom gap just started
          // Next ISR (in 29us from now) we must handle the second part of the RailCom start bit.
          dccISR.firstHalfBitRC = false;
          // Since normal DCC halfbits are either 58us or 100us, we must stay in the first DCC halfbit.
          // However, at the end of the ISR there there will always be a change of DCC halfbits.
          // To avoid an extra if statement at the end of the ISR, we do "a trick" here, and
          // also a change from the first DCC halfbit to the second.
          // Changing twice means no change at all.
          dccISR.firstHalfBit = !dccISR.firstHalfBit;
        }
        else {                              // First halfbit, but second part of the RC startbit
          DISABLE_DCCOUT;                   // The normal pin will not toggle during the RC gap
          ENABLE_DCCMON;                    // The Monitor pin will continue to toggle at ISRs
          dccISR.firstHalfBitRC = true;     // Next time back to the first part of the RC startbit
        }
      break;

      case dccRailComGap:
        // Nothing to do
      break;

      case dccPreambleStartBit:
        if (dccISR.railPower)               // Only if their there should be power on the rails
          ENABLE_DCCINV;                    // We restart the inverted signal
        dccISR.railComCutoutInProgress = false; // Only now we are certain that the gap is over
      break;

      case dccPreamble:
        // Nothing to do
      break;

      // About to send a data byte, but have to preceed the data with a '0' (start bit).
      // Send that '0', then move to dccData
      case dccStartBitPacket:
        TMR_ZERO;
        // We can proces a new packet provided we don't have to repeat earlier SM packets
        if (!dccPacketEngine.isServiceModeRepeating()) {
          // Did the application code already gave us a new packet?
          // In that case dccPacketEngine.canAcceptPacket is false
          if (!dccPacketEngine.canAcceptPacket) {
            // use next user packet
            for (uint8_t i = 0; i < dccISR.nextPacketSize; i++)
              dccISR.currentPacket[i] = dccISR.nextPacket[i];
            dccISR.currentPacketSize = dccISR.nextPacketSize;
            // Setup SM retransmission if required
            if (dccISR.isServiceModePacket)
              dccISR.smRepeatsRemaining = dccISR.smMaxRepeats;
          } else {
            // No new packet: load default packet
            dccISR.currentPacket[0] = dccISR.defaultPacket[0];
            dccISR.currentPacket[1] = dccISR.defaultPacket[1];
            dccISR.currentPacket[2] = dccISR.defaultPacket[2];
            dccISR.currentPacketSize = 3;
          }
          dccPacketEngine.canAcceptPacket = true; // ready for next packet
        } else {
          // still repeating SM packet, decrement counter
          if (dccISR.smRepeatsRemaining > 0)
            dccISR.smRepeatsRemaining = dccISR.smRepeatsRemaining - 1;
        }
      break;

      case dccStartBitData:
        TMR_ZERO;             // The bit between Data bytes should always be 0
      break;

      case dccData:
        mask = 1 << (dccISR.bitsRemaining - 1);
        // Determine if we have to send a 0 or a 1
        if (dccISR.currentPacket[dccISR.currentPacketSize - dccISR.bytesRemainingInPacket] & mask)
          TMR_ONE;
          else TMR_ZERO;
      break;

      case dccEndBit:
        TMR_ONE;             // The DCC end bit should always be 1
      break;

    } // END SWITCH CASE
  }  // END firstHalfBit

  // ===========================================================================
  // PART 2: SECOND HALFBIT: UPDATE THE STATE MACHINE
  // ===========================================================================
  else {
    // At the start of the first half-bit, the normal rail output is high while
    // the inverted rail output is low. When pin transitions are generated
    // in software, the required state change is performed explicitly here
    // by toggling the outputs.
    // In the hardware-assisted approach (toggle-on-compare-match),
    // dccFirstHalfBitToggle() is implemented as an empty inline function,
    // as the required transition is handled automatically by the timer.
    dccSecondHalfBitToggle();

    switch (dccISR.state) {

      case dccRailComStartBit:
        TMR_ONE;
        dccISR.bitsRemaining = dccISR.bitsRemaining - 1;
        dccISR.state = dccRailComGap;
      break;

      case dccRailComGap:
        dccISR.bitsRemaining = dccISR.bitsRemaining - 1;
        if (dccISR.bitsRemaining == 0) {      // Last half of last RailCom Gap bit
          if (dccISR.railPower)               // Only if the rails may be powered
            ENABLE_DCCOUT;                    // The normal pin is allowed to toggle again
          dccISR.bitsRemaining = dccISR.preambleLength - RAILCOM_CUTOUT_LENGTH;
          dccISR.state = dccPreambleStartBit;
        }
      break;

      case dccPreambleStartBit:
        dccISR.bitsRemaining = dccISR.bitsRemaining - 1;
        dccISR.state = dccPreamble;
      break;

      case dccPreamble:
        dccISR.bitsRemaining = dccISR.bitsRemaining - 1;
        if (dccISR.bitsRemaining == 0) dccISR.state = dccStartBitPacket;
      break;

      case dccStartBitPacket:
        dccISR.state = dccData;
        dccISR.bitsRemaining = 8;	                              	// Reset the counter for bit sending
        dccISR.bytesRemainingInPacket	= dccISR.currentPacketSize;	// Reset the counter to the packet_size
      break;

      case dccStartBitData:
        dccISR.state = dccData;
        dccISR.bitsRemaining = 8;	                       // Reset the counter for bit sending
      break;

      case dccData:
        dccISR.bitsRemaining = dccISR.bitsRemaining - 1; // One bit less to go for this byte
        if(dccISR.bitsRemaining == 0) {                  // No more bits for this byte?
          // One byte less for this packet
          dccISR.bytesRemainingInPacket = dccISR.bytesRemainingInPacket - 1;
          if(dccISR.bytesRemainingInPacket == 0)         // No more bytes to go for this packet?
            dccISR.state = dccEndBit;                    // Ready for this packet
          else dccISR.state = dccStartBitData;           // This packet has still bytes left
        };
      break;

      case dccEndBit:
        if (!dccPacketEngine.isServiceModeEnabled()) {
          if (dccISR.railCom) {                    // We must generate a RailCom gap
            dccISR.state = dccRailComStartBit;
            dccISR.bitsRemaining = RAILCOM_CUTOUT_LENGTH;
          }
          else {
            if (dccISR.railPower)                   // Only if the rails may be powered
              ENABLE_DCCOUT;                        // The normal pin is allowed to toggle again
            dccISR.state = dccPreambleStartBit;
            dccISR.bitsRemaining = dccISR.preambleLength; // Normal preamble (RCN 211)
            }
          }
        else {
          dccISR.state = dccPreamble;
          dccISR.bitsRemaining = dccISR.preambleLengthSM; // Long preamble (RCN 216)
        }
      break;

    } // End switch
  } // End second half bit

  // Next ISR we will take the other halfbit
  dccISR.firstHalfBit = !dccISR.firstHalfBit;

  ISR_EPILOGUE;                 // Processor specific code may be here

}; // End ISR
