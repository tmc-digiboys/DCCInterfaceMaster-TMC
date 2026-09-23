// Support file for the inrush protection test
//
// Contains the serial command parser, the DCC traffic generators, the inrush
// burst timer and the (still rudimentary) nFault and ADC handling.
//
// This file uses the #defines of TEST-inrush.ino, and must be included after them.
#pragma once

//#define SwitchFormat IB     // ROCO (+3) or IB (+7)
#define SwitchFormat ROCO     // ROCO

#define ADC_CHANNEL (PIN_DCC_SENSE - 40)   // RP2350B: ADC0..ADC7 are GPIO40..GPIO47

DCCPacketScheduler dps;

// Test settings, may be changed via the serial monitor
uint32_t  burstMs = 5;        // Duration of a single inrush burst, in ms
uint32_t  gapMs   = 20;       // Pause between repeated inrush bursts, in ms

// What is running?
bool      accessory;          // Send accessory commands
bool      loco;               // Send loco commands
bool      faultReaction;      // Start an inrush burst whenever nFault becomes active
bool      adcPrinting;        // Print the current sense value periodically

uint16_t  accAddress;
uint16_t  locoAddress;
uint32_t  accTimer;
uint32_t  locoTimer;

// Inrush burst generator
enum BurstState {BurstIdle, BurstOn, BurstGap};
BurstState burstState = BurstIdle;
uint8_t   burstsLeft;
uint32_t  burstTimer;

// nFault
bool      faultActive;        // Current state of the nFault pin (active low)
uint16_t  faultCount;         // Number of nFault events since the last reset
uint32_t  faultTime;          // Moment of the last nFault event

// Current sense
uint16_t  adcValue;           // Last sample
uint16_t  adcMax;             // Highest sample since the last reset
uint32_t  adcTimer;


bool readSerialCommand(Stream &ser, char &outCmd, long &outValue, bool &outHasValue) {
  // Reads a command letter, optionally followed by a number (for example "i8" or "p0").
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


void sendAccessory() {
  if (!accessory) return;
  if (millis() - accTimer > 200) {
    accAddress++;
    if (accAddress > 999) accAddress = 0;
    dps.setBasicAccessoryPos(accAddress, 1, true);
    accTimer = millis();
  }
};


void sendLoco() {
  if (!loco) return;
  if (millis() - locoTimer > 1000) {
    locoAddress++;
    if (locoAddress > 999) locoAddress = 1;
    dps.setSpeed(locoAddress, 4);
    locoTimer = millis();
  }
};


void startBursts(uint8_t count) {
  // Start one or more inrush bursts of burstMs each, separated by gapMs.
  // Note: a single inrush "packet" lasts INRUSH_REPEATS * 20 us (1 ms by default) and
  // is handed to the DMA as one atomic transfer. The end of a burst is therefore
  // rounded up to the next multiple of that packet length.
  if (count == 0) return;
  burstsLeft = count;
  burstTimer = millis();
  burstState = BurstOn;
  dccPacketEngine.enterInrushMode();
}


void updateBursts(void) {
  switch (burstState) {
    case BurstIdle:
    break;
    case BurstOn:
      if ((millis() - burstTimer) >= burstMs) {
        dccPacketEngine.leaveInrushMode();
        burstTimer = millis();
        burstsLeft--;
        if (burstsLeft == 0) burstState = BurstIdle;
          else burstState = BurstGap;
      }
    break;
    case BurstGap:
      if ((millis() - burstTimer) >= gapMs) {
        dccPacketEngine.enterInrushMode();
        burstTimer = millis();
        burstState = BurstOn;
      }
    break;
  }
}


void updateFault(void) {
  // Polling of the nFault pin. Since nFault is asserted for at least tRETRY (2 ms),
  // polling from the main loop is fast enough and no debouncing is needed (yet).
  bool active = (digitalRead(PIN_DCC_FAULT) == LOW);
  if (active && !faultActive) {
    faultCount++;
    faultTime = millis();
    monitor.print("nFault at ");
    monitor.println(faultTime);
    // TODO: this is where the real protection algorithm belongs: count the retries,
    // decide whether the current is rising or falling, and give up (power off) if needed.
    if (faultReaction && (burstState == BurstIdle)) startBursts(1);
  }
  faultActive = active;
}


void updateAdc(void) {
  // Sampled every pass through loop(), to catch the peaks. Note that samples taken
  // during a RailCom cutout or during the off phase of an inrush pulse are low by
  // definition; only adcMax is meaningfull.
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
  monitor.println("  i[ms]  single inrush burst (i8 = 8 ms burst)");
  monitor.println("  r[n]   n repeated inrush bursts (default 3)");
  monitor.println("  e / q  enter / leave inrush mode, without timer");
  monitor.println("  d<ms>  set the burst duration");
  monitor.println("  g<ms>  set the gap between repeated bursts");
  monitor.println("  p1/p0  rail power on / off");
  monitor.println("  s1/s0  service mode on / off");
  monitor.println("  c1/c0  RailCom gap on / off");
  monitor.println("  v1/v0  DCC signal inverted on / off");
  monitor.println("  m1/m0  inrush pattern on the monitor pin on / off");
  monitor.println("  o1/o0  monitor (RailSync) output on / off");
  monitor.println("  h1/h0  H-bridge enable pin high / low");
  monitor.println("  a1/a0  accessory commands on / off");
  monitor.println("  l1/l0  loco commands on / off");
  monitor.println("  n1/n0  react on nFault with an inrush burst on / off");
  monitor.println("  w1/w0  periodic printing of the current sense on / off");
  monitor.println("  z      reset the ADC maximum and the nFault counter");
  monitor.println("  ?      show this menu");
  monitor.println();
}


void showStatus(void) {
  monitor.print("Inrush: ");
  monitor.print(dccPacketEngine.isInrushModeEnabled() ? "on " : "off");
  monitor.print("  burst: ");
  monitor.print(burstMs);
  monitor.print(" ms  gap: ");
  monitor.print(gapMs);
  monitor.print(" ms  bursts left: ");
  monitor.println(burstsLeft);

  monitor.print("Power: ");
  monitor.print(dps.getpower() == ON ? "on " : "off");
  monitor.print("  RailCom: ");
  monitor.print(dps.getrailcom() ? "on " : "off");
  monitor.print("  SM: ");
  monitor.print(dccPacketEngine.isServiceModeEnabled() ? "on " : "off");
  monitor.print("  H-bridge: ");
  monitor.println(digitalRead(PIN_DCC_ACTIVE) ? "on " : "off");

  monitor.print("Accessory: ");
  monitor.print(accessory ? "on " : "off");
  monitor.print("  Loco: ");
  monitor.print(loco ? "on " : "off");
  monitor.print("  canAcceptPacket: ");
  monitor.println(dccPacketEngine.canAcceptPacket);

  monitor.print("nFault: ");
  monitor.print(faultActive ? "active" : "idle  ");
  monitor.print("  count: ");
  monitor.print(faultCount);
  monitor.print("  reaction: ");
  monitor.print(faultReaction ? "on " : "off");
  monitor.print("  ADC max: ");
  monitor.println(adcMax);
}
