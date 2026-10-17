// Support file for the DRV8874 PH/EN test
//
// Contains the serial command parser, the loco traffic generator, the nFault
// monitoring, the inrush handling and the current sense (ADC) handling.
// The accessory test is in accessory.h.
//
// This file uses the #defines of TST-8874.ino, and must be included after them.
#pragma once

//#define SwitchFormat IB     // ROCO (+3) or IB (+7)
#define SwitchFormat ROCO     // ROCO

#define ADC_CHANNEL (PIN_DCC_SENSE - 40)   // RP2350B: ADC0..ADC7 are GPIO40..GPIO47

DCCPacketScheduler dps;

// Test settings, may be changed via the serial monitor
bool      dccInverted = false;  // DCC signal (PH) inverted (v1): swaps the track polarity.
                                // The library has no getter, so we keep a copy.
bool      loco;               // Send loco commands
bool      adcPrinting;        // Print the current sense value periodically

uint16_t  locoAddress;
uint32_t  locoTimer;

// nFault
bool      faultActive;        // Current state of the nFault pin (active low)
uint16_t  faultCount;         // Number of nFault events since the last reset
uint32_t  faultStart;         // Start of the current nFault event (micros)

// Inrush (also used in the nFault ISR, therefore volatile)
volatile bool     faultReaction;      // Start an inrush on every nFault (n1/n0)
volatile uint16_t inrushMs = 10;      // Length of the inrush (i<ms>); the driver limits it
bool      inrushActive;       // Last known state of isInrushModeEnabled()
uint32_t  inrushStart;        // Start of the current inrush (micros)

// Current sense
uint16_t  adcValue;           // Last sample
uint16_t  adcMax;             // Highest sample since the last reset
uint32_t  adcTimer;


bool readSerialCommand(Stream &ser, char &outCmd, long &outValue, bool &outHasValue) {
  // Reads a command letter, optionally followed by a number (for example "p0" or "c1").
  // This allows non-blocking reading of the input
  // The "standard" Arduino int value = monitor.parseInt(); blocks the code for 1 second
  static char letter = 0;
  static long value = 0;
  static bool inNumber = false;
  while (ser.available()) {
    char c = ser.read();
    if (c == '\n' || c == '\r') {
      if (letter == 0) continue;    // ignore empty ENTER
      outCmd = letter;
      outValue = value;
      outHasValue = inNumber;
      letter = 0;
      value = 0;
      inNumber = false;
      return true;                  // complete input received
    }
    if (c == ' ' || c == '\t') continue;
    if (c >= '0' && c <= '9') {
      if (letter == 0) continue;    // a number without command letter: ignore
      inNumber = true;
      value = value * 10 + (c - '0');
    }
    else {
      // a new command letter. The previous (unterminated) input is dropped
      letter = c;
      value = 0;
      inNumber = false;
    }
  }
  return false;                     // incomplete input
}


void sendLoco() {
  if (!loco) return;
  if (millis() - locoTimer > 1000) {
    locoAddress++;
    if (locoAddress > 999) locoAddress = 1;
    dps.setSpeed(locoAddress, 4);
    locoTimer = millis();
  }
}


void updateFault(void) {
  // Polling of the nFault pin. Since nFault is asserted for at least tRETRY (2 ms),
  // polling from the main loop is fast enough and no debouncing is needed.
  // For each nFault event the start time and, once released, its length are printed.
  bool active = (digitalRead(PIN_DCC_FAULT) == LOW);
  if (active && !faultActive) {
    faultCount++;
    faultStart = micros();
    monitor.print("nFault at ");
    monitor.print(millis());
    monitor.println(" ms");
  }
  if (!active && faultActive) {
    monitor.print("nFault released after ");
    monitor.print((micros() - faultStart) / 1000.0, 1);
    monitor.println(" ms");
  }
  faultActive = active;
}


void onFault(void) {
  // nFault ISR (falling edge). The inrush is started immediately, so that it is
  // already running when the DRV8874 re-enables its outputs after tRETRY (2 ms).
  if (faultReaction) dccPacketEngine.startInrushMode(inrushMs);
}


void updateInrush(void) {
  // Prints the actual length of each inrush. The driver ends an inrush at the first
  // packet boundary after inrushMs, so the actual length is up to one packet longer.
  bool active = dccPacketEngine.isInrushModeEnabled();
  if (active && !inrushActive) inrushStart = micros();
  if (!active && inrushActive) {
    monitor.print("Inrush ended after ");
    monitor.print((micros() - inrushStart) / 1000.0, 1);
    monitor.println(" ms");
  }
  inrushActive = active;
}


void updateAdc(void) {
  // Sampled every pass through loop(), to catch the peaks. Note that samples taken
  // during a RailCom cutout or an inrush are low by definition; only adcMax is meaningful.
  adcValue = adc_read();
  if (adcValue > adcMax) adcMax = adcValue;
  if (!adcPrinting) return;
  if ((millis() - adcTimer) > 500) {
    adcTimer = millis();
    monitor.print("ADC: ");
    monitor.print(adcValue);
    monitor.print("  max: ");
    monitor.println(adcMax);
  }
}


void showMenu(void) {
  monitor.println();
  monitor.println("Commands (letter, optionally followed by a number):");
  monitor.println("  p1/p0  rail power on / off");
  monitor.println("  c1/c0  RailCom gap on / off");
  monitor.println("  v1/v0  DCC signal (PH) inverted / normal");
  monitor.println("  h1/h0  H-bridge nSLEEP high (awake) / low (sleep)");
  monitor.println("  i[ms]  start an inrush (i20 = 20 ms; also sets the length for n1)");
  monitor.println("  n1/n0  start an inrush on every nFault on / off");
  monitor.println("  l1/l0  loco commands on / off");
  monitor.println("  w1/w0  periodic printing of the current sense on / off");
  monitor.println("  z      reset the ADC maximum and the nFault counter");
  monitor.println("  ?      show this menu");
  monitor.println();
}


void showStatus(void) {
  monitor.print("Power: ");
  monitor.print(dps.getpower() == ON ? "on " : "off");
  monitor.print("  RailCom: ");
  monitor.print(dps.getrailcom() ? "on " : "off");
  monitor.print("  H-bridge: ");
  monitor.println(digitalRead(PIN_DCC_ACTIVE) ? "on " : "off");

  monitor.print("DCC signal (PH): ");
  monitor.println(dccInverted ? "inverted" : "normal");

  monitor.print("Loco: ");
  monitor.print(loco ? "on " : "off");
  monitor.print("  canAcceptPacket: ");
  monitor.println(dccPacketEngine.canAcceptPacket);

  monitor.print("nFault: ");
  monitor.print(faultActive ? "active" : "idle  ");
  monitor.print("  count: ");
  monitor.print(faultCount);
  monitor.print("  ADC max: ");
  monitor.println(adcMax);

  monitor.print("Inrush: ");
  monitor.print(dccPacketEngine.isInrushModeEnabled() ? "on " : "off");
  monitor.print("  length: ");
  monitor.print(inrushMs);
  monitor.print(" ms  on nFault: ");
  monitor.println(faultReaction ? "on " : "off");
}
