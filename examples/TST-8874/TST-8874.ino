// Test file for the DRV8874 - PH/EN mode
//
// hardware:  Olimex Pico2 XL (RP2350B).
//
// Relevant commands for the packet scheduler
// void setup(uint8_t pin, uint8_t pin2, uint8_t steps = DCC128, uint8_t format = ROCO, uint8_t power = ON);
// void setpower(uint8_t state, bool notify = false);
// void setrailcom(bool rc = true);
//
// Relevant methods of the packet engine (DCCHardware.h)
// void setDccSignalInverted(bool inverted);      // Changes the polarity of the DCC signal (PH)
// void startInrushMode(uint16_t ms);             // Short EN pulses for ms, ending at a packet boundary
// bool isInrushModeEnabled(void);                // True while inrush mode is active
#include <Arduino.h>
#include <hardware/adc.h>
#include <DCCPacketScheduler_new.h>
#include <DCCHardware.h>


#define monitor Serial2

// DCC output pins. The RP driver needs two consecutive pins: PIN_DCC = PIN_EN + 1
#define PIN_EN          32   // GPIO32 — EN signal (H-bridge)
#define PIN_DCC         33   // GPIO33 — PH signal (H-bridge)

// DRV887x control and measurement pins
#define PIN_DCC_ACTIVE  36   // GPIO36 — DRV887x nSLEEP / enable (active high)
#define PIN_DCC_FAULT   35   // GPIO35 — DRV887x nFAULT (active low, open drain)
#define PIN_DCC_SENSE   44   // GPIO44 — IPROPI current sense (ADC4 on the RP2350B)

// support.h uses the #defines above, and is therefore included after them
#include "support.h"
#include "accessory.h"                        // accessory test


void setup() {
  monitor.begin(115200);
  delay(500);
  monitor.println("DRV8874 PH/EN test started");

  pinMode(PIN_DCC_ACTIVE, OUTPUT);
  digitalWrite(PIN_DCC_ACTIVE, LOW);          // Keep the H-bridge asleep during setup
  pinMode(PIN_DCC_FAULT, INPUT_PULLUP);       // nFAULT is open drain
  attachInterrupt(digitalPinToInterrupt(PIN_DCC_FAULT), onFault, FALLING);

  adc_init();                                 // Current sense (IPROPI)
  adc_gpio_init(PIN_DCC_SENSE);
  adc_select_input(ADC_CHANNEL);

  dps.setup(PIN_DCC, PIN_EN, DCC128, SwitchFormat);   // Also switches the rail power on
  dps.setrailcom(true);
  if (dccPacketEngine.dccRailPin == 0xFF) monitor.println("Invalid DCC pins");

  digitalWrite(PIN_DCC_ACTIVE, HIGH);         // Wake up the H-bridge, now that the PIO drives EN

  accessory = true;                           // accessory test
  showMenu();
  showAccessoryMenu();                        // accessory test
  showStatus();
  showAccessoryStatus();                      // accessory test
}


void handleCommand(char cmd, long value, bool hasValue) {
  bool on = hasValue ? (value != 0) : true;   // For the on/off commands: no digit means "on"
  // Echo the received command, so it is clear that it arrived and how it was interpreted
  monitor.print("> ");
  monitor.print(cmd);
  if (hasValue) monitor.print(value);
  monitor.println();
  switch (cmd) {
    case 'p': dps.setpower(on ? ON : OFF); break;
    case 'c': dps.setrailcom(on); break;
    case 'v': dccInverted = on; dccPacketEngine.setDccSignalInverted(on); break;
    case 'h': digitalWrite(PIN_DCC_ACTIVE, on); break;
    case 'i':                                 // Manual inrush; i<ms> also sets the length
      if (hasValue) inrushMs = value;
      dccPacketEngine.startInrushMode(inrushMs);
    break;
    case 'n': faultReaction = on; break;
    case 'a': accessory = on; break;          // accessory test
    case 'l': loco = on; break;
    case 'w': adcPrinting = on; break;
    case 'z': adcMax = 0; faultCount = 0; break;
    case '?':
      showMenu();
      showAccessoryMenu();                    // accessory test
    break;
    default:
      monitor.print("Unknown command: ");
      monitor.println(cmd);
    return;
  }
  showStatus();
  showAccessoryStatus();                      // accessory test
}


void loop() {
  char cmd;
  long value;
  bool hasValue;
  if (readSerialCommand(monitor, cmd, value, hasValue)) handleCommand(cmd, value, hasValue);

  sendLoco();
  updateFault();
  updateInrush();
  updateAdc();

  if (accessory) sendAccessory();             // accessory test: bypasses the scheduler
    else dps.update();
}
