//******************************************************************************
//
// file:      DCCHardware.h
// purpose:   Common class file for all hardware variants
// author:    Aiko Pras
// version:   2026-02-01 V1.0.1 ap initial version
//            2026-09-23 V1.1.0 ap Inrush protection added, to prevent DRV887x OCP
//            2026-10-14 V1.2.0 ap Old inrush code removed. TMC variant added
//            2026-10-16 V1.2.1 ap New inrush code added
//
// history:   This is a further development of earlier DCCHardware.h files.
//            It has been changed into a clean C++ class file and avoids the
//            "leaking" of variables that occured in earlier versions
//
// hardware:  Should be able to run / adapt to all possible processors, and
//            therefore be processor independent
//
// -----------------------------------------------------------------------------
// Synchronization with the application:
// -----------------------------------------------------------------------------
//
// The public interface is synchronous and flag-based. If the canAcceptPacket
// flag is true, the application knows that it can perform a send() call.
//
// -----------------------------------------------------------------------------
// OUTPUT MODES AND PIN BEHAVIOUR
// -----------------------------------------------------------------------------
//
// The behaviour of the DCC output pins depends on whether the hardware layer is
// operating in "Z21PG" mode, in "HQ" mode or "TMC" mode. All modes share the
// same public interface, but they map the abstract functions of this class to
// different electrical behaviour on the pins.
//
// If an invalid pin is specified, the driver will change its value to 0xFF.
//
// -----------------------------------------------------------------------------
// Z21PG MODE (backward compatible)
// -----------------------------------------------------------------------------
//
// Z21PG mode is fully compatible with earlier versions of this library and with
// existing (L6203-based) hardware designs.
//
// In this mode:
//
// - dccRailPin
//   Carries the primary DCC waveform. If RailCom is enabled via setRailCom(),
//   this signal will contain a RailCom gap at the appropriate position within
//   the packet; otherwise it is a continuous DCC signal.
//
// - dccRailAuxPin
//   Outputs the inverted version of the signal present on dccRailPin. This is
//   intended for H-bridge drivers that require complementary inputs.
//
// - StopOutputSignal()
//   Forces both dccRailPin and dccRailAuxPin LOW, effectively disabling the rail
//   waveform at the output level.
//
// - RunOutputSignal()
//   Re-enables normal DCC generation on both dccRailPin and dccRailAuxPin.
//
// - dccMonitorPin (optional)
//   If configured to a valid pin (i.e. not 0xFF), this pin outputs a continuous
//   DCC waveform that never contains a RailCom gap and is not affected by
//   StopOutputSignal(). This makes it suitable as an auxiliary signal for
//   external systems such as Loconet, S88 or monitoring purposes.
//
// -----------------------------------------------------------------------------
// HQ MODE (using dedicated hardware for DCC waveform generation)
// -----------------------------------------------------------------------------
//
// In HQ mode the generation of the main DCC waveform remains unchanged, but the
// role of the auxiliary pin is repurposed to better support modern rail drivers
// that separate waveform generation from RailCom cutout control (such as DRV8874).
//
// In this mode:
//
// - dccRailPin
//   Carries the primary DCC waveform exactly as in Z21PG mode. If RailCom is
//   enabled via setRailCom(), the waveform will still contain a RailCom gap.
//
// - dccRailAuxPin
//   No longer carries an inverted DCC signal. Instead, it produces a dedicated
//   RailCom control signal that is HIGH only for the exact duration of the
//   RailCom gap. Outside the gap this pin is LOW. This signal can be used
//   directly by hardware that requires an explicit cutout or "force-low"
//   control input during RailCom reception.
//
// - StopOutputSignal()
//   Forces dccRailAuxPin to its inactive level (default HIGH).
//
// - RunOutputSignal()
//   Re-enables the dccRailAuxPin signal.
//
// - dccMonitorPin
//   Has no function in HQ mode and does not produce any signal.
//
// -----------------------------------------------------------------------------
// TMC MODE (RP2040 / RP2350; DRV887x H-bridge in PH/EN mode)
// -----------------------------------------------------------------------------
//
// Pin roles are as in HQ (dccRailPin = DCC, dccRailAuxPin = enable/cutout),
// but both pins are driven by a single PIO state machine. EN and PH therefore
// always change at exactly the same moment. Unlike HQ, the aux pin is
// active-high by default (see setAuxActiveLevel()), which fits the DRV8874.
//
// In this mode:
//
// - dccRailPin
//   Carries the primary DCC waveform and connects to PH. If RailCom is
//   enabled via setRailCom(), the waveform contains a RailCom gap.
//
// - dccRailAuxPin
//   Connects to EN. It is HIGH while the output is active, and LOW during the
//   RailCom gap and after StopOutputSignal() (default active level). If EN is
//   LOW, the DRV887x brakes (both outputs low), which shorts the track as
//   required by RCN-217 during the cutout.
//   The PIO drives both pins as one consecutive group, starting with EN.
//   dccRailAuxPin must therefore be dccRailPin - 1; otherwise both pins are
//   set to 0xFF.
//
// - StopOutputSignal()
//   Forces dccRailAuxPin (EN) LOW. dccRailPin keeps carrying DCC.
//
// - RunOutputSignal()
//   Re-enables dccRailAuxPin (EN).
//
// - setDccSignalInverted()
//   Inverts dccRailPin (PH) only; EN is not affected.
//
// - setAuxActiveLevel()
//   Sets the active level of dccRailAuxPin (EN). Default is HIGH, as
//   required by the DRV887x. PH is not affected.
//
// - dccMonitorPin
//   Has no function in TMC mode and does not produce any signal.
//
// - startInrushMode()
//   High capacitive loads trigger the DRV8874 overcurrent protection (OCP).
//   startInrushMode() temporarily replaces EN by short pulses, to charge
//   such loads without OCP.
//
// ----------------------------------------------------------------------------------------------------
// This source file is subject of the GNU general public license 3,
// that is available at the world-wide-web at http://www.gnu.org/licenses/gpl.txt
//
//******************************************************************************************************
#pragma once
#include <Arduino.h>

#define MaxDccSize 6                              // DCC messages can have a length upto this value

// The interface between the outside world and DCCHardware
class DccPacketEngine {
  public:

    // DCC output pins
    uint8_t dccRailPin;   	                      // Main DCC waveform; may include RailCom gap; TMC: Phase signal (PH)
    uint8_t dccRailAuxPin;	                      // Z21PG: inverted DCC; HQ: RailCom gap signal; TMC: Enable signal (EN)
    uint8_t dccMonitorPin;                        // Optional: continuous DCC without RailCom gap (Z21PG only; unused in HQ and TMC)

    // DCC message specific
    volatile bool canAcceptPacket;                // Flag that tells the user that a new packet may be sent
    void send(const uint8_t* data, uint8_t size); // Sends a DCC packet (3–6 bytes including XOR).

    // Service Mode (SM): Methods to enter / leave
    void enterServiceMode(void);                  // send Long preamble, resets and repeat SM packets
    void leaveServiceMode(void);                  // Back to normal mode
    bool isServiceModeEnabled(void);              // True while SM (long preamble / resets) is active

    // Service Mode packet transmission and repeats
    void setServiceModeMaxRepeats(uint8_t value); // Set maximum number of SM packet repeats
    bool isFirstServiceModePacket(void);          // True only for the first packet of a new SM repeat sequence
    bool isServiceModeRepeating(void);            // True while the ISR is repeating the current SM packet
    void stopServiceModeRepeats(void);            // Abort SM retransmissions (e.g. after ACK)

    // Inrush protection (TMC only; empty stubs in the other variants)
    void startInrushMode(uint16_t ms);            // Short EN pulses for ms, ending at a packet boundary
    bool isInrushModeEnabled(void);               // True while inrush mode is active

    // RailCom specific
    void setRailCom(bool active);                 // Enable / disable generation of the RailCom gap
    bool getRailCom(void);                        // Is generation of the RailCom gap enabled?
    bool railComGap(void);                        // Does the DCC Hardware generate a Railcom gap at this moment?

    // Init, start and stop
    void StopOutputSignal(void);                  // Z21PG: both rail pins LOW; HQ & TMC: Aux pin inactive
    void RunOutputSignal(void);                   // Re-enables the signal on the pins
    void setupWaveformGenerator();                // Setup and start the waveform generator

    // Settings
    void setDccSignalInverted(bool inverted);     // Changes the polarity of the DCC signal(s)
    void setPreambleLength(uint8_t value);        // Length of normal preamble (>= 17)
    void setPreambleLengthSM(uint8_t value);      // Length of preamble in Service Mode (>= 20)
    void setAuxActiveLevel(bool activeHigh);      // HQ and TMC: the AuxPin value after RunOutputSignal()
    void setRailComGapInAux(bool useForRcCutout); // Only HQ: is the AuxPin used for the RC cutout?

    DccPacketEngine();                            // Constructor declaration
};

extern DccPacketEngine dccPacketEngine;           // The dccPacketEngine must be accessible externally
