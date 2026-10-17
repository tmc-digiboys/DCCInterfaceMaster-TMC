// Accessory test for the DRV8874 PH/EN test
//
// Sends basic accessory commands with ascending addresses: 1..ACC_LAST ON, then
// 1..ACC_LAST OFF, then again. Each command is sent exactly once, directly after the
// previous one. A DCC sniffer therefore shows every lost packet as a missing address.
//
// To achieve this, the packet scheduler is bypassed: as soon as the DCC hardware can
// accept a new packet, the next command is given directly to dccPacketEngine.send().
// While this test runs, loop() does not call dps.update(), so loco commands (l1) wait.
//
// This file uses monitor (TST-8874.ino), and must be included after it.
// To remove this test: delete this file and the lines marked "accessory test" in TST-8874.ino,
// and replace the if / else at the end of loop() by dps.update().
#pragma once

#define ACC_LAST 1024           // Highest accessory address of the test

bool      accessory;            // Accessory test on / off (a1/a0)
uint16_t  accAddress = 1;       // Address of the next command
bool      accState = true;      // Position of the next command: true = ON, false = OFF


void sendAccessory(void) {
  // Same packet as DCCPacketScheduler::setBasicAccessoryPos(), but without its repeats
  if (!dccPacketEngine.canAcceptPacket) return;
  DCCPacket p(accAddress);
  uint8_t data[1];
  data[0] = 0;
  if (accState) bitWrite(data[0], 0, 1);  // R: position
  bitWrite(data[0], 3, 1);                // D: activate
  p.addData(data, 1);
  p.setKind(basic_accessory_packet_kind);
  uint8_t dccPacket[MaxDccSize];
  uint8_t size = p.getBitstream(dccPacket);
  dccPacketEngine.send(dccPacket, size);
  // Next command: after the last address, start again with the other position
  accAddress++;
  if (accAddress > ACC_LAST) {
    accAddress = 1;
    accState = !accState;
  }
}


void showAccessoryMenu(void) {
  monitor.println("  a1/a0  accessory test on / off (1..1024 ON, then OFF; each sent once)");
}


void showAccessoryStatus(void) {
  monitor.print("Accessory test: ");
  monitor.print(accessory ? "on " : "off");
  monitor.print("  next: ");
  monitor.print(accAddress);
  monitor.println(accState ? " ON" : " OFF");
}
