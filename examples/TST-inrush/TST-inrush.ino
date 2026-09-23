// Test file for the inrush protection
//
// hardware:  Olimex Pico2 XL (RP2350B). Pin numbers are taken from the
//            hardware_config.h of the TMC-LZ210 firmware.
//
// Relevant commands for the packet scheduler
// void setup(uint8_t pin, uint8_t pin2, uint8_t steps = DCC128, uint8_t format = ROCO, uint8_t power = ON);
// void enable_additional_DCC_output(uint8_t pin); //extra DCC signal for S88/LocoNet without Shutdown and Railcom
// void disable_additional_DCC_output(void);
//
// Relevant methods of the packet engine (DCCHardware.h)
// void enterInrushMode(void);                    // send sequence of short pulses to prevent DRV887x OCP
// void leaveInrushMode(void);                    // Back to normal mode
// bool isInrushModeEnabled(void);                // True while inrush mode is active
// void useMonitorPinforInrush(bool value);       // The monitor pin is also used for inrush protection
#include <Arduino.h>
#include <hardware/adc.h>
#include <DCCPacketScheduler_new.h>
#include <DCCHardware.h>


#define monitor Serial2

// DCC output pins. The RP driver needs three consecutive pins, starting at PIN_DCC
#define PIN_DCC         32   // GPIO32 — DCC signal
#define PIN_AUX         33   // GPIO33 — DCC signal inverted (H-bridge)
#define PIN_MONITOR     34   // GPIO34 — RailSync / monitor signal (no RailCom gap, no power off)

// DRV887x control and measurement pins
#define PIN_DCC_ACTIVE  36   // GPIO36 — DRV887x nSLEEP / enable (active high)
#define PIN_DCC_FAULT   35   // GPIO35 — DRV887x nFAULT (active low, open drain)
#define PIN_DCC_SENSE   44   // GPIO44 — IPROPI current sense (ADC4 on the RP2350B)

// support.h uses the #defines above, and is therefore included after them
#include "support.h"


void setup() {
  monitor.begin(115200);
  delay(500);
  monitor.println("Inrush test started");

  pinMode(PIN_DCC, OUTPUT);
  pinMode(PIN_AUX, OUTPUT);
  pinMode(PIN_MONITOR, OUTPUT);
  pinMode(PIN_DCC_ACTIVE, OUTPUT);
  pinMode(PIN_DCC_FAULT, INPUT_PULLUP);       // nFAULT is open drain

  digitalWrite(PIN_DCC_ACTIVE, HIGH);         // Wake up the H-bridge

  adc_init();                                 // Current sense (IPROPI)
  adc_gpio_init(PIN_DCC_SENSE);
  adc_select_input(ADC_CHANNEL);

  dps.setup(PIN_DCC, PIN_AUX, DCC128, SwitchFormat);   // with Railcom
  dps.enable_additional_DCC_output(PIN_MONITOR);
  dps.setpower(ON);
  dps.setrailcom(true);

  accessory = true;
  showMenu();
  showStatus();
}


void handleCommand(char cmd, long value, bool hasValue) {
  bool on = hasValue ? (value != 0) : true;   // For the on/off commands: no digit means "on"
  switch (cmd) {
    case 'i':                                 // Single inrush burst
      if (hasValue) burstMs = value;
      startBursts(1);
    break;
    case 'r':                                 // Repeated inrush bursts
      startBursts(hasValue ? value : 3);
    break;
    case 'e': dccPacketEngine.enterInrushMode();  burstState = BurstIdle; break;
    case 'q': dccPacketEngine.leaveInrushMode();  burstState = BurstIdle; break;
    case 'd': if (hasValue) burstMs = value; break;
    case 'g': if (hasValue) gapMs = value; break;
    case 'p': dps.setpower(on ? ON : OFF); break;
    case 's': if (on) dccPacketEngine.enterServiceMode(); else dccPacketEngine.leaveServiceMode(); break;
    case 'c': dps.setrailcom(on); break;
    case 'v': dccPacketEngine.setDccSignalInverted(on); break;
    case 'm': dccPacketEngine.useMonitorPinforInrush(on); break;
    case 'o': if (on) dps.enable_additional_DCC_output(PIN_MONITOR);
              else dps.disable_additional_DCC_output();
    break;
    case 'h': digitalWrite(PIN_DCC_ACTIVE, on); break;
    case 'a': accessory = on; break;
    case 'l': loco = on; break;
    case 'n': faultReaction = on; break;
    case 'w': adcPrinting = on; break;
    case 'z': adcMax = 0; faultCount = 0; break;
    case '?': showMenu(); break;
    default:
      monitor.print("Unknown command: ");
      monitor.println(cmd);
    return;
  }
  showStatus();
}


void loop() {
  char cmd;
  long value;
  bool hasValue;
  if (readSerialCommand(monitor, cmd, value, hasValue)) handleCommand(cmd, value, hasValue);

  sendAccessory();
  sendLoco();
  updateBursts();
  updateFault();
  updateAdc();

  dps.update();
}
