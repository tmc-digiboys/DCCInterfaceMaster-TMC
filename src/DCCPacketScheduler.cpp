/*
* DCC Waveform Generator
*
* modified by Philipp Gahtow
* Copyright 2015-2022
* digitalmoba@arcor.de, http://pgahtow.de
*
*/

#include "DCCPacketScheduler_new.h"
#include "DCCHardware.h"
#include "DDCHardware_config.h"

#if defined(GLOBALRAILCOMREADER)
#include <avr/interrupt.h>
#endif

#if defined(__SAM3X8E__)	//Arduino DUE
#include <DueFlashStorage.h>
DueFlashStorage DueFlash;
#define FSTORAGE 	DueFlash
#define FSTORAGEMODE write

#elif defined(ESP32) //ESP32 only!
#include "z21nvs.h"
z21nvsClass EEPROMDCC;
#define FSTORAGE  EEPROMDCC
#define FSTORAGEMODE write

#elif defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
#define FSTORAGE  EEPROM
#define FSTORAGEMODE write

#else
// AVR based Boards follows
#include <EEPROM.h>
#define FSTORAGE 	EEPROM
#if defined(ESP8266)
#define FSTORAGEMODE write
#else
#define FSTORAGEMODE update
#endif
#endif

#if defined(ESP32)
extern portMUX_TYPE timerMux;
#endif


uint8_t ProgRepeat = 10;

uint16_t current_cv = 0;	//cv that we are working on
uint8_t current_cv_value = 0;	//value that is read
uint8_t current_cv_bit = 0xFF;	//bit that will be read - 0xFF = ready, nothing to read!
uint8_t cv_read_count = 0;		//count number of cv read

#define NON_PROG_OP 		0x00
#define WAIT_FOR_ACK		0x01
#define ACK_DETECTED		0x02
#define ACK_READ_SUCCESS 	0x10
#define ACK_READ_FAIL		0xFF
uint8_t current_ack_status = NON_PROG_OP;
 //current_ack_read = false;
unsigned long ack_start_time = 0;

#if defined(ESP32)
extern hw_timer_t * timer;
#endif

#if defined(GLOBALRAILCOMREADER)
//DCCPacketScheduler *DCCPacketScheduler::active_object = 0;	//Static handle object for interrupt
#endif

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////
//---------------------------------------------------------------------------------
DCCPacketScheduler::DCCPacketScheduler(void) : /*default_speed_steps(128),*/ /*last_packet_address(255),*/ packet_counter(1)
{
  e_stop_queue.setup(E_STOP_QUEUE_SIZE);	//just to send once for set repeat circle
//  high_priority_queue.setup(HIGH_PRIORITY_QUEUE_SIZE);
//  low_priority_queue.setup(LOW_PRIORITY_QUEUE_SIZE);
  repeat_queue.setup(REPEAT_QUEUE_SIZE);  //all repeat packets are inserted here, repeat first with priority!
//NEW
  periodic_refresh_queue.setup(PERIODIC_REFRESH_QUEUE_SIZE);	//collect all packtes there are permanent to repeat

  ops_programmming_queue.setup(PROG_QUEUE_SIZE); //for CV programming only
}

//---------------------------------------------------------------------------------
void DCCPacketScheduler::setup(uint8_t pin, uint8_t pin2, uint8_t steps, uint8_t format, uint8_t power) //for any post-constructor initialization
{
	loadEEPROMconfig();	//load the configuration

	dccPacketEngine.dccRailPin = pin;	    //set DCC Waveform pin
	dccPacketEngine.dccRailAuxPin = pin2;	//use as inverted DCC Waveform or Railcom gap pin
	dccPacketEngine.dccMonitorPin = 0xFF;	//disable per default

	dccPacketEngine.setupWaveformGenerator();	//Timer neu configurieren

	#if defined(__SAM3X8E__)
	TC_Start(DCC_ARM_TC_TIMER, DCC_ARM_TC_CHANNEL);
	#elif defined(ESP8266) 	//ESP8266
	timer1_enable(DCC_ESP_TIMER_DIV, DCC_ESP_TIMER_SET, DCC_ESP_TIMER_LOOP);
	#endif

	setpower(power, true);	//DCC signal on (active) and inform other over the new power state!

	slotFullNext = 0;
	if (format == LENZ) {
		TrntFormat = 0;
		DCCPacket::trntFormat          = ROCO;  // offset 3: client 1 -> lin 4 -> wrapat correct
		DCCPacket::nonLinearAddressing = true;
	} else {
		TrntFormat = 0;
		DCCPacket::trntFormat          = format; // ROCO=3 of IB=7
		DCCPacket::nonLinearAddressing = false;
	}
	DCCdefaultSteps = steps;

	ProgState = ProgStart;	//default Direct CV Zustand

	//Following RP 9.2.4, begin by putting 20 reset packets and 10 idle packets on the rails.
	//reset packet: address 0x00, data 0x00, XOR 0x00; S 9.2 line 75
	opsDecoderReset(RSTsRepeat);	//send first a Reset Packet

	//idle packet: address 0xFF, data 0x00, XOR 0xFF; S 9.2 line 90
	//Automatically send idle packet until first action!

	#if defined(GLOBALRAILCOMREADER)
		Serial3.begin(250000);
		POMCVAdr = 0xFFFF;
	#endif
}

//---------------------------------------------------------------------------------
//extra DCC signal for S88/LocoNet without Shutdown and Railcom
void DCCPacketScheduler::enable_additional_DCC_output(uint8_t pin)
{
	dccPacketEngine.dccMonitorPin = pin;		//set PIN for S88/LocoNet true DCC output
	dccPacketEngine.setupWaveformGenerator();	//Timer neu configurieren
}
//---------------------------------------------------------------------------------
void DCCPacketScheduler::disable_additional_DCC_output(void)
{
	enable_additional_DCC_output(0xFF); //disable - no PIN set
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::isInServiceMode(void) const {
    return dccPacketEngine.isServiceModeRepeating();
}

void DCCPacketScheduler::setAckReceived(void) {
    // External module signals a valid ACK — set status immediately SUCCESS
    if (current_ack_status == WAIT_FOR_ACK || current_ack_status == ACK_DETECTED) {
        current_ack_status = ACK_READ_SUCCESS;
    }
}

void DCCPacketScheduler::loadEEPROMconfig(void)
{
  // If an external configuration has been provided, use that instead of EEPROM
  if (_hasExtCfg) {
    uint8_t railcom     = (_extCfg.railcom       >= 0) ? (uint8_t)_extCfg.railcom      : 0x01;
    uint8_t progRdMode  = (_extCfg.progReadMode  >= 0) ? (uint8_t)_extCfg.progReadMode  : 0x03;
    uint8_t progRep     = (_extCfg.progRepeat    >= 7) ? (uint8_t)_extCfg.progRepeat    : OPS_MODE_PROGRAMMING_REPEAT;
    uint8_t rstSRep     = (_extCfg.rstSRepeat    >= 0) ? (uint8_t)_extCfg.rstSRepeat    : RESET_START_REPEAT;
    uint8_t rstCRep     = (_extCfg.rstCRepeat    >= 0) ? (uint8_t)_extCfg.rstCRepeat    : RESET_CONT_REPEAT;
    dccPacketEngine.setRailCom(railcom);
    ProgReadMode = progRdMode;
    ProgRepeat   = progRep;
    RSTsRepeat   = rstSRep;
    RSTcRepeat   = rstCRep;
    dccPacketEngine.setServiceModeMaxRepeats(ProgRepeat);
    return;
  }

  // Fallback: original EEPROM reading
  #if defined(ARDUINO_ARCH_RP2040)
    EEPROM.begin(256);                         // EEPROM size
  #endif

	if (FSTORAGE.read(EEPROMRailCom) > 1)
		FSTORAGE.FSTORAGEMODE(EEPROMRailCom,0x01);	//Default activ

	if (FSTORAGE.read(EEPROMProgReadMode) > 3)
		FSTORAGE.FSTORAGEMODE(EEPROMProgReadMode,0x03);	//Default "Beides"

	// if ((FSTORAGE.read(EEPROMProgRepeat) > 64) | (FSTORAGE.read(EEPROMRSTcRepeat) > 64)) {  // AP 2026
  if ((FSTORAGE.read(EEPROMProgRepeat) < 7) || (FSTORAGE.read(EEPROMProgRepeat) > 64)) {
		FSTORAGE.FSTORAGEMODE(EEPROMProgRepeat,OPS_MODE_PROGRAMMING_REPEAT);	//range 7-64
		FSTORAGE.FSTORAGEMODE(EEPROMRSTsRepeat,RESET_START_REPEAT);		//range 25-255
		FSTORAGE.FSTORAGEMODE(EEPROMRSTcRepeat,RESET_CONT_REPEAT);		//range 6-64
	}

	#if defined(ESP8266) || defined(ESP32) || defined(ARDUINO_ARCH_RP2040) //ESP8266, ESP32 or RP2040/2350
	FSTORAGE.commit();
	#endif

	dccPacketEngine.setRailCom(FSTORAGE.read(EEPROMRailCom));	// define if railcom cutout is active
	ProgReadMode = FSTORAGE.read(EEPROMProgReadMode);	        // Auslese-Modus: 0=Nichts, 1=Bit, 2=Byte, 3=Beides
	ProgRepeat = FSTORAGE.read(EEPROMProgRepeat);	            // Repaet for Packet Programming
	RSTsRepeat = FSTORAGE.read(EEPROMRSTsRepeat);	            // Repaet for Reset start Packet
	RSTcRepeat = FSTORAGE.read(EEPROMRSTcRepeat);	            // Repaet for Reset contingue Packet
  dccPacketEngine.setServiceModeMaxRepeats(ProgRepeat);     // Repeat for SM packets is done by the ISR itself
}

//---------------------------------------------------------------------------------
//set the power for the dcc signal
void DCCPacketScheduler::setpower(uint8_t state, bool notify)
{
	if (railpower != state) {
		railpower = state;	//save the state of the railpower
		if (state == OFF || state == SHORT) {
			//RailPower-Signal generate OFF
      dccPacketEngine.StopOutputSignal();
		}
		else {
			//generate RailPower-Signal
      dccPacketEngine.RunOutputSignal();
		}
		if (notifyRailpower && notify)
			notifyRailpower(railpower);
	}
}

//---------------------------------------------------------------------------------
//get the actual state of power
byte DCCPacketScheduler::getpower(void)
{
	return railpower;
}

//---------------------------------------------------------------------------------
//to de-/activate RailCom cutout rail output
void DCCPacketScheduler::setrailcom(bool rc)
{
	dccPacketEngine.setRailCom(rc);
}

//---------------------------------------------------------------------------------
//return the State of RailCom cutout output
bool DCCPacketScheduler::getrailcom(void)
{
  return dccPacketEngine.getRailCom();
}

//---------------------------------------------------------------------------------
/*bool DCCPacketScheduler::setSpeed(uint16_t address, uint8_t speed)
{
	//set Loco to speed with default settings!
  switch(DCCdefaultSteps)
  {
    case 14:
      return(setSpeed14(address, speed));
    case 28:
      return(setSpeed28(address, speed));
    case 128:
      return(setSpeed128(address, speed));
  }
  return false; //invalid number of steps specified.
}
*/
 // 2026/01/31 AP
bool DCCPacketScheduler::setSpeed(uint16_t address, uint8_t speed)
{
	//set Loco to speed with default settings!
  switch(DCCdefaultSteps)
  {
    case DCC14:
      return(setSpeed14(address, speed));
    case DCC28:
      return(setSpeed28(address, speed));
    case DCC128:
      return(setSpeed128(address, speed));
  }
  return false; //invalid number of steps specified.
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setSpeed14(uint16_t address, uint8_t speed)
{
	if (address == 0)	//check if Adr is ok?
		return false;

	byte slot = LokStsgetSlot(address);
	LokDataUpdate[slot].speed = speed;	//write Dir and Speed into register to SAVE
	if ((LokDataUpdate[slot].adr >> 14) != DCC14)  //0=>14steps, write speed steps into register
		LokDataUpdate[slot].adr = (LokDataUpdate[slot].adr & 0x3FFF) | (DCC14 << 14);

	// DCC 14-stap G-GGGG shuffle: bit4=G0(LSB), bits2-0=G3G2G1, G4=0
	// Input: bit7=richting, bits3-0=logische stap 0..15
	uint8_t v   = speed & 0x0F;
	uint8_t dir = (speed & 0x80) ? 0x20 : 0x00;
	uint8_t shuffled = ((v & 0x01) << 4)   // G0 -> bit4
	               | ((v >> 1) & 0x07);  // G3G2G1 -> bits2-0
	uint8_t speed_data_uint8_ts[] = {0x40};
	speed_data_uint8_ts[0] |= dir | shuffled;

    DCCPacket p(address);
    p.addData(speed_data_uint8_ts,1);

    p.setRepeat(SPEED_REPEAT);

    p.setKind(speed_packet_kind);

    //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
    //speed packets go to the high proirity queue
    //return(high_priority_queue.insertPacket(&p));
	if (railpower == ESTOP)	//donot send to rails now!
		return periodic_refresh_queue.insertPacket(&p);
	return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setSpeed28(uint16_t address, uint8_t speed)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  byte slot = LokStsgetSlot(address);
  LokDataUpdate[slot].speed = speed;		// speed & 0b01111111 + Dir;	//write into register to SAVE
  if ((LokDataUpdate[slot].adr >> 14) != DCC28)	//2=>28steps, write into register
	LokDataUpdate[slot].adr = (LokDataUpdate[slot].adr & 0x3FFF) | (DCC28 << 14);

  // DCC 28-stap G-GGGG shuffle: bit4=G0(LSB), bits3-0=G4G3G2G1
  // Input: bit7=richting, bits4-0=logische stap 0..31
  uint8_t v28  = speed & 0x1F;
  uint8_t dir28 = (speed & 0x80) ? 0x20 : 0x00;
  uint8_t shuffled28 = ((v28 & 0x01) << 4)   // G0 -> bit4
                     | ((v28 >> 1) & 0x0F);  // G4G3G2G1 -> bits3-0
  uint8_t speed_data_uint8_ts[] = {0x40};
  speed_data_uint8_ts[0] |= dir28 | shuffled28;

  DCCPacket p(address);
  p.addData(speed_data_uint8_ts,1);

  p.setRepeat(SPEED_REPEAT);

  p.setKind(speed_packet_kind);

  //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
  //speed packets go to the high proirity queue
  //return(high_priority_queue.insertPacket(&p));
  if (railpower == ESTOP)	//donot send to rails now!
	  return periodic_refresh_queue.insertPacket(&p);
  return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setSpeed128(uint16_t address, uint8_t speed)
{
	if (address == 0) {
	//	Serial.println("ERROR ADR0");
		return false;
	}
	byte slot = LokStsgetSlot(address);
	LokDataUpdate[slot].speed = speed;	//write Speed and Dir into register to SAVE
	if ((LokDataUpdate[slot].adr >> 14) != DCC128) //3=>128steps, write into register
		LokDataUpdate[slot].adr = (LokDataUpdate[slot].adr & 0x3FFF) | (DCC128 << 14);

	uint8_t speed_data_uint8_ts[] = { 0x3F, 0x00 };

//	if (speed == 1) //estop!
//		return eStop(address);//speed_data_uint8_ts[1] |= 0x01; //estop
	//else
		speed_data_uint8_ts[1] = speed; //no conversion necessary.

	//why do we get things like this?
	// 03 3F 16 15 3F (speed packet addressed to loco 03)
	// 03 3F 11 82 AF  (speed packet addressed to loco 03, speed hex 0x11);
	DCCPacket p(address);
	p.addData(speed_data_uint8_ts, 2);

	p.setRepeat(SPEED_REPEAT);

	p.setKind(speed_packet_kind);

	//speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
	//speed packets go to the high proirity queue

	//return(high_priority_queue.insertPacket(&p));
	if (railpower == ESTOP)	//donot send to rails now!
		return periodic_refresh_queue.insertPacket(&p);
	return repeat_queue.insertPacket(&p);
}

//--------------------------------------------------------------------------------------------
//Lokfunktion setzten
void DCCPacketScheduler::setLocoFunc(uint16_t address, uint8_t type, uint8_t fkt)
{			//type => 0 = AUS; 1 = EIN; 2 = UM; 3 = error
	bool fktbit = 0;	//neue zu �ndernde fkt bit
	if (type == 1)	//ein
		fktbit = 1;
	byte Slot = LokStsgetSlot(address);
	//zu �nderndes bit bestimmen und neu setzten:
	if (fkt <= 4) {
		byte func = LokDataUpdate[Slot].f0 & 0x1F;	//letztes Zustand der Funktionen 000 F0 F4..F1
		if (type == 2) { //um
			if (fkt == 0)
				fktbit = !(bitRead(func, 4));
			else fktbit = !(bitRead(func, fkt - 1));
		}
		if (fkt == 0)
			bitWrite(func, 4, fktbit);
		else bitWrite(func, fkt - 1, fktbit);
		//Daten senden:
		setFunctions0to4(address, func);	//func = 0 0 0 F0 F4 F3 F2 F1
	}
	else if ((fkt >= 5) && (fkt <= 8)) {
		byte funcG2 = LokDataUpdate[Slot].f1 & 0x0F;	//letztes Zustand der Funktionen 0000 F8..F5
		if (type == 2) //um
			fktbit = !(bitRead(funcG2, fkt - 5));
		bitWrite(funcG2, fkt - 5, fktbit);
		//Daten senden:
		setFunctions5to8(address, funcG2);	//funcG2 = 0 0 0 0 F8 F7 F6 F5
	}
	else if ((fkt >= 9) && (fkt <= 12)) {
		byte funcG3 = LokDataUpdate[Slot].f1 >> 4;	//letztes Zustand der Funktionen 0000 F12..F9
		if (type == 2) //um
			fktbit = !(bitRead(funcG3, fkt - 9));
		bitWrite(funcG3, fkt - 9, fktbit);
		//Daten senden:
		setFunctions9to12(address, funcG3); 	//funcG3 = 0 0 0 0 F12 F11 F10 F9
	}
	else if ((fkt >= 13) && (fkt <= 20)) {
		byte funcG4 = LokDataUpdate[Slot].f2;
		if (type == 2) //um
			fktbit = !(bitRead(funcG4, fkt - 13));
		bitWrite(funcG4, fkt - 13, fktbit);
		//Daten senden:
		setFunctions13to20(address, funcG4);	//funcG4 = F20 F19 F18 F17 F16 F15 F14 F13
	}
	else if ((fkt >= 21) && (fkt <= 28)) {
		byte funcG5 = LokDataUpdate[Slot].f3;
		if (type == 2) //um
			fktbit = !(bitRead(funcG5, fkt - 21));
		bitWrite(funcG5, fkt - 21, fktbit);
		//Daten senden:
		setFunctions21to28(address, funcG5);	//funcG5 = F28 F27 F26 F25 F24 F23 F22 F21
	}
	#if defined(EXTENDFUNCTION)
	else if ((fkt >= 29) && (fkt <= 36)) {
		byte func = LokDataUpdate[Slot].f0 >> 5;
		if (type == 2) //um
			fktbit = !(bitRead(func, fkt - 29));
		bitWrite(func, fkt - 29, fktbit);
		//Daten senden:
		setFunctions29to36(address, func);
	}
	else if ((fkt >= 37) && (fkt <= 44)) {
		byte func = 0; //LokDataUpdate[Slot].f5;
		if (type == 2) //um
			fktbit = !(bitRead(func, fkt - 37));
		bitWrite(func, fkt - 37, fktbit);
		//Daten senden:
		setFunctions37to44(address, func);
	}
	else if ((fkt >= 45) && (fkt <= 52)) {
		byte func = 0; //LokDataUpdate[Slot].f6;
		if (type == 2) //um
			fktbit = !(bitRead(func, fkt - 45));
		bitWrite(func, fkt - 45, fktbit);
		//Daten senden:
		setFunctions45to52(address, func);
	}
	else if ((fkt >= 53) && (fkt <= 60)) {
		byte func = 0; //LokDataUpdate[Slot].f7;
		if (type == 2) //um
			fktbit = !(bitRead(func, fkt - 53));
		bitWrite(func, fkt - 53, fktbit);
		//Daten senden:
		setFunctions53to60(address, func);
	}
	else if ((fkt >= 61) && (fkt <= 68)) {
		byte func = 0; //LokDataUpdate[Slot].f8;
		if (type == 2) //um
			fktbit = !(bitRead(func, fkt - 61));
		bitWrite(func, fkt - 61, fktbit);
		//Daten senden:
		setFunctions61to68(address, func);
	}
	#endif
}

//--------------------------------------------------------------------------------------------
//Lokfunktion Binary State setzten
void DCCPacketScheduler::setLocoFuncBinary(uint16_t address, uint8_t low, uint8_t high) {
	/*	Preamble | 0AAA-AAAA | AAAA-AAAA | 110xxxxx | FLLL LLLL | HHHH HHHH | Err.Det.B
		F Das oberste Bit F legt fest, ob der Bin�rzustand eingeschaltet oder ausgeschaltet ist.
		LLLLLLL Die niederwertigen sieben (!) Bits der Bin�rzustandsadresse.
		HHHHHHHH Die die h�herwertigen acht Bits der Bin�rzustandsadresse.
		DCC Bin�rzustandssteuerungsbefehle werden drei Mal am Gleis ausgegeben,
		und danach gem�� RCN-212 nicht mehr regelm��ig wiederholt! */

	if (high == 0) {	//Bin�rzustandsadressen < 128 ==> kurze Form

		if ((low & 0x7F) < 29)	//nur Bin�rzustandsadressen von 29 bis 32767
			return;

		//Bin�rzustandssteuerungsbefehl kurze Form: 1101-1101 DLLL-LLLL
		DCCPacket p(address);
		uint8_t data[] = { 0b11011101, low};
		p.addData(data, 2);
		p.setKind(function_packet_b_kind);
		p.setRepeat(FUNCTION_REPEAT);
		/*
		//save:
		#if defined(EXTENDFUNCTION)
		if (fkt < 69) {
			uint8_t bitPos = (fkt - 29) % 8;
			uint8_t num = (fkt - 29) / 8;
			switch (num) {
				case 0: bitWrite(f4, bitPos, fktbit); break; //F36 - F29
				case 1: bitWrite(f5, bitPos, fktbit); break; //F44 - F37
				case 2: bitWrite(f6, bitPos, fktbit); break; //F52 - F45
				case 3: bitWrite(f7, bitPos, fktbit); break; //F60 - F53
				case 4: bitWrite(f8, bitPos, fktbit); break; //F68 - F61
			}
		}
		#endif
		*/
		e_stop_queue.insertPacket(&p);
	}
	else { 	//bis max 32767
		//Bin�rzustandssteuerungsbefehl lange Form: 1100-0000 DLLL-LLLL HHHH-HHHH
		DCCPacket p(address);
		uint8_t data[] = { 0b11000000, low, high};
		p.addData(data, 3);
		p.setKind(function_packet_b_kind);
		p.setRepeat(FUNCTION_REPEAT);

		e_stop_queue.insertPacket(&p);
	}
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setFunctions0to4(uint16_t address, uint8_t functions)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  DCCPacket p(address);
  uint8_t data[] = { 0x80 };

  //Obnoxiously, the headlights (F0, AKA FL) are not controlled
  //by bit 0, but in DCC via bit 4. !
  data[0] |= functions & 0x1F;		//new - normal way of DCC! F0, F4, F3, F2, F1

  p.addData(data,1);
  p.setKind(function_packet_1_kind);
  p.setRepeat(FUNCTION_REPEAT);

  LokDataUpdate[LokStsgetSlot(address)].f0 = (functions & 0x1F) | (LokDataUpdate[LokStsgetSlot(address)].f0 & 0b11100000);	//write into register to SAVE

  return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setFunctions5to8(uint16_t address, uint8_t functions)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  DCCPacket p(address);
  uint8_t data[] = { 0xB0 };
  data[0] |= functions & 0x0F;

  p.addData(data,1);
  p.setKind(function_packet_2_kind);
  p.setRepeat(FUNCTION_REPEAT);

  LokDataUpdate[LokStsgetSlot(address)].f1 = (LokDataUpdate[LokStsgetSlot(address)].f1 | 0x0F) & (functions | 0xF0);	//write into register to SAVE

  return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setFunctions9to12(uint16_t address, uint8_t functions)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  DCCPacket p(address);
  uint8_t data[] = { 0xA0 };
  //least significant four functions (F5--F8)
  data[0] |= functions & 0x0F;

  p.addData(data,1);
  p.setKind(function_packet_3_kind);
  p.setRepeat(FUNCTION_REPEAT);

  LokDataUpdate[LokStsgetSlot(address)].f1 = (LokDataUpdate[LokStsgetSlot(address)].f1 | 0xF0) & ((functions << 4) | 0x0F);	//write into register to SAVE

  return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setFunctions13to20(uint16_t address, uint8_t functions)	//F20 F19 F18 F17 F16 F15 F14 F13
{
	//Funktionssteuerung F13-F20: 1101-1110 DDDD-DDDD
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0b11011110, functions }; 	//significant functions (F20--F13)
	p.addData(data, 2);
	p.setKind(function_packet_4_kind);
	p.setRepeat(FUNCTION_REPEAT);
	LokDataUpdate[LokStsgetSlot(address)].f2 = functions; //write into register to SAVE

	return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setFunctions21to28(uint16_t address, uint8_t functions)	//F28 F27 F26 F25 F24 F23 F22 F21
{
	//Funktionssteuerung F21-F28: 1101-1111 DDDD-DDDD
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0b11011111, functions};
	p.addData(data, 2);
	p.setKind(function_packet_5_kind);
	p.setRepeat(FUNCTION_REPEAT);
	LokDataUpdate[LokStsgetSlot(address)].f3 = functions; //write into register to SAVE

	return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setFunctions29to36(uint16_t address, uint8_t functions)	//F29-F36
{
	//Funktionssteuerung F29-F36: 1101-1000 DDDD-DDDD
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0b11011000, functions};
	p.addData(data, 2);
	p.setKind(function_packet_5_kind);
	p.setRepeat(FUNCTION_REPEAT);

	#if defined(EXTENDFUNCTION)
	LokDataUpdate[LokStsgetSlot(address)].f0 = (functions << 5) | (LokDataUpdate[LokStsgetSlot(address)].f0 & 0x1F); //write into register to SAVE
	#endif

	return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setFunctions37to44(uint16_t address, uint8_t functions)	//F37-F44
{
	//Funktionssteuerung F37-F44: 1101-1001 DDDD-DDDD
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0b11011001, functions};
	p.addData(data, 2);
	p.setKind(function_packet_5_kind);
	p.setRepeat(FUNCTION_REPEAT);

	#if defined(EXTENDFUNCTION)
	//LokDataUpdate[LokStsgetSlot(address)].f5 = functions; //write into register to SAVE
	#endif

	return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setFunctions45to52(uint16_t address, uint8_t functions)	//F45-52
{
	//Funktionssteuerung F45-F52: 1101-1010 DDDD-DDDD
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0b11011010, functions};
	p.addData(data, 2);
	p.setKind(function_packet_5_kind);
	p.setRepeat(FUNCTION_REPEAT);

	#if defined(EXTENDFUNCTION)
	//LokDataUpdate[LokStsgetSlot(address)].f6 = functions; //write into register to SAVE
	#endif

	return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setFunctions53to60(uint16_t address, uint8_t functions)	//F53-60
{
	//Funktionssteuerung F53-F60: 1101-1011 DDDD-DDDD
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0b11011011, functions};
	p.addData(data, 2);
	p.setKind(function_packet_5_kind);
	p.setRepeat(FUNCTION_REPEAT);

	#if defined(EXTENDFUNCTION)
	//LokDataUpdate[LokStsgetSlot(address)].f7 = functions; //write into register to SAVE
	#endif

	return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setFunctions61to68(uint16_t address, uint8_t functions)	//F61-68
{
	//Funktionssteuerung F61-F68: 1101-1100 DDDD-DDDD
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0b11011100, functions};
	p.addData(data, 2);
	p.setKind(function_packet_5_kind);
	p.setRepeat(FUNCTION_REPEAT);

	#if defined(EXTENDFUNCTION)
	//LokDataUpdate[LokStsgetSlot(address)].f8 = functions; //write into register to SAVE
	#endif

	return repeat_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
byte DCCPacketScheduler::getFunktion0to4(uint16_t address)	//gibt Funktionszustand - F0 F4 F3 F2 F1 zur�ck
{
	return LokDataUpdate[LokStsgetSlot(address)].f0 & 0x1F;
}
//---------------------------------------------------------------------------------
byte DCCPacketScheduler::getFunktion5to8(uint16_t address)	//gibt Funktionszustand - F8 F7 F6 F5 zur�ck
{
	return LokDataUpdate[LokStsgetSlot(address)].f1 & 0x0F;
}
//---------------------------------------------------------------------------------
byte DCCPacketScheduler::getFunktion9to12(uint16_t address)	//gibt Funktionszustand - F12 F11 F10 F9 zur�ck
{
	return LokDataUpdate[LokStsgetSlot(address)].f1 >> 4;
}
//---------------------------------------------------------------------------------
byte DCCPacketScheduler::getFunktion13to20(uint16_t address)	//gibt Funktionszustand F20 - F13 zur�ck
{
	return LokDataUpdate[LokStsgetSlot(address)].f2;
}
//---------------------------------------------------------------------------------
byte DCCPacketScheduler::getFunktion21to28(uint16_t address)	//gibt Funktionszustand F28 - F21 zur�ck
{
	return LokDataUpdate[LokStsgetSlot(address)].f3;
}
//---------------------------------------------------------------------------------
byte DCCPacketScheduler::getFunktion29to31(uint16_t address)	//gibt Funktionszustand F31 - F29 zur�ck
{
	return LokDataUpdate[LokStsgetSlot(address)].f0 >> 5;
}

//---------------------------------------------------------------------------------
bool DCCPacketScheduler::setBasicAccessoryPos(uint16_t address, bool state)
{
	return setBasicAccessoryPos(address, state, true);	//Ausgang aktivieren
}

//---------------------------------------------------------------------------------
//send an accessory message
bool DCCPacketScheduler::setBasicAccessoryPos(uint16_t address, bool state, bool activ)
{
    // address = outputAddress (client adres, 1-gebaseerd, 1..2048)
    // Alle adresconversie (lineair/niet-lineair) gebeurt in DCCPacket::getBitstream()
    if (address == 0 || address > 0x800)
        return false;

    DCCPacket p(address);  // outputAddress direct doorgeven aan getBitstream
    uint8_t data[1];
    data[0] = 0;
    if (state) bitWrite(data[0], 0, 1);   // R: richting/positie
    if (activ) bitWrite(data[0], 3, 1);   // D: activeer

    p.addData(data, 1);
    p.setKind(basic_accessory_packet_kind);
    p.setRepeat(OTHER_REPEAT);

    if (notifyTrnt)
		notifyTrnt(address, state, activ);

	bitWrite(BasicAccessory[address / 8], address % 8, state);	//pro SLOT immer 8 Zust�nde speichern!

	return e_stop_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
//return the state of an accessory
bool DCCPacketScheduler::getBasicAccessoryInfo(uint16_t address)
{
 	switch (TrntFormat) {
		case IB: address = address + IB; break;
	}

	return bitRead(BasicAccessory[address / 8], address % 8);	//Zustand aus Slot lesen
}

//---------------------------------------------------------------------------------
//send an extended accessory message
bool DCCPacketScheduler::setExtAccessoryPos(uint16_t address, uint8_t state)
{
	/*
	Extended Accessory decoder packet format:
	================================
	1111..11 0 1000-0001 0 0111-1011 0 xxxx-xxxx 0 EEEE-EEEE 1
      Preamble | 10AA-AAAA | 0aaa-0AA1 | DDDD-DDDD | Err.Det.B
  	*/
	if (address == 0 || address > 0x800)
		return false;

	// address = outputAddress (client adres, one-based)
	// Address conversion is in DCCPacket::getBitstream()
	DCCPacket p(address);
	uint8_t data[2];
	data[0] = 0x01;		// bit0=1 requires extended format (0000-0AA1, AA via getBitstream)
	data[1] = state;	// DDDD-DDDD

	p.addData(data, 2);
	p.setKind(extended_accessory_packet_kind);
	p.setRepeat(OTHER_REPEAT);

	if (notifyExtTrnt)
		notifyExtTrnt(address, state);

	return e_stop_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
//Special Function for programming, switch and estop:
//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
//write CV byte value
bool DCCPacketScheduler::opsProgDirectCV(uint16_t CV, uint8_t CV_data)
{
	//check if CV# is between 0 - 1023
	if (CV > 1023) {
		if (notifyCVNack)
			notifyCVNack(CV);
		return false;
	}

	if (railpower != SERVICE) {	//time to wait for the relais!
		setpower(SERVICE, true);
	}

	ProgState = ProgStart;
	ProgMode = ProgModeWriteByte;
	current_cv = CV;
	current_cv_value = CV_data;
	return true;
}
//##################################################################################
//intern Function!
void DCCPacketScheduler::opsWriteCV(uint16_t CV, uint8_t CV_data)
{
	//for CV#1 is the Adress 0
	//Long-preamble   0  0111CCAA  0  AAAAAAAA  0  DDDDDDDD  0  EEEEEEEE  1
	//CC=10 Bit Manipulation
	//CC=01 Verify byte
	//CC=11 Write byte 	<--
	DCCPacket p(((CV >> 8) & 0b11) | 0b01111100);
	uint8_t data[] = { 0x00 , 0x00};
	data[0] = CV & 0xFF;
	data[1] = CV_data;
	p.addData(data, 2);
	p.setKind(ops_mode_programming_kind);	//always use short Adress Mode!
	p.setRepeat(1); //auto repeat inside ISR! (ProgRepeat)

	ops_programmming_queue.insertPacket(&p);	//send on the rails
}

//---------------------------------------------------------------------------------
//verify CV value extern Function:
bool DCCPacketScheduler::opsVerifyDirectCV(uint16_t CV, uint8_t CV_data)
{
	//check if CV# is between 0 - 1023
	if (CV > 1023) {
		if (notifyCVNack)
			notifyCVNack(CV);
		return false;
	}

	if (railpower != SERVICE) {	//time to wait for the relais!
		setpower(SERVICE, true);
	}

	ProgState = ProgStart;
	ProgMode = ProgModeByteVerify;
	current_cv = CV;
	current_cv_value = CV_data;
	return true;
}
//##################################################################################
//intern Function!
void DCCPacketScheduler::opsVerifyCV(uint16_t CV, uint8_t CV_data)
{
	//for CV#1 is the Adress 0
	//Long-preamble   0  0111CCAA  0  AAAAAAAA  0  DDDDDDDD  0  EEEEEEEE  1
	//CC=10 Bit Manipulation
	//CC=01 Verify byte		<--
	//CC=11 Write byte
	DCCPacket p(((CV >> 8) & 0b11) | 0b01110100);
	uint8_t data[2];
	data[0]	= CV & 0xFF;
	data[1] = CV_data;
	p.addData(data, 2);
	p.setKind(ops_mode_programming_kind);	//always use short Adress Mode!
	p.setRepeat(1); //auto repeat inside ISR! (ProgRepeat)

	ops_programmming_queue.insertPacket(&p);	//send on the rails
}
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
//read a CV in bit-Mode
bool DCCPacketScheduler::opsReadDirectCV(uint16_t CV)
{
	//check if CV# is between 0 - 1023
	if (CV > 1023) {
		if (notifyCVNack)
			notifyCVNack(CV);
		return false;
	}

	if (railpower != SERVICE) {	//time to wait for the relais!
		setpower(SERVICE, true);
	}
	ProgState = ProgStart;
	if (ProgReadMode == 2)
		ProgMode = ProgModeByte;
	else ProgMode = ProgModeBit;
	current_cv = CV;
	return true;
}
//##################################################################################
//intern Function!
void DCCPacketScheduler::opsReadCV(uint16_t CV, uint8_t bitToRead, bool bitState)
{
	//for CV#1 is the Adress 0
	//long-preamble   0  0111CCAA  0  AAAAAAAA  0  111KDBBB  0  EEEEEEEE  1
	//CC=10 Bit Manipulation	<--
	//CC=01 Verify byte
	//CC=11 Write byte
	//BBB represents the bit position
	//D contains the value of the bit to be verified or written
	//K=1 signifies a "Write Bit" operation and K=0 signifies a "Bit Verify"
	DCCPacket p(((CV >> 8) & 0b11) | 0b01111000);
	uint8_t data[] = { 0x00 , 0x00};
	data[0] = CV & 0xFF;
	data[1] = 0b11100000 | (bitToRead & 0x07) | (bitState << 3);	//verify Bit is "bitSet"? ("1" or "0")
	p.addData(data, 2);
	p.setKind(ops_mode_programming_kind);	//always use short Adress Mode!
	p.setRepeat(1);	//auto repeat inside ISR! (ProgRepeat)

	ops_programmming_queue.insertPacket(&p);	//send on the rails
}
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
//POM - write CV byte value
bool DCCPacketScheduler::opsProgramCV(uint16_t address, uint16_t CV, uint8_t CV_data)
{
	//format of packet:
	// {preamble} 0 [ AAAAAAAA ] 0 111011VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (write)
	// {preamble} 0 [ AAAAAAAA ] 0 111001VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (verify/read)
	// {preamble} 0 [ AAAAAAAA ] 0 111010VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (bit manipulation)
	// only concerned with "write" form here!!!

	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);

	// split the CV address up among data uint8_ts 0 and 1

  // Changed, to avoid compiler warnings
	//uint8_t data[] = { ((CV >> 8) & 0b11) | 0b11101100, CV & 0xFF, CV_data };
  uint8_t data[] = {
    static_cast<uint8_t>(((CV >> 8) & 0b11) | 0b11101100),
    static_cast<uint8_t>(CV & 0xFF),
    static_cast<uint8_t>(CV_data)
  };

	p.addData(data, 3);
	p.setKind(pom_mode_programming_kind);
	p.setRepeat(ProgRepeat);

	//return low_priority_queue.insertPacket(&p);	//Standard

	return ops_programmming_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
//POM - write CV in bit-Mode
bool DCCPacketScheduler::opsPOMwriteBit(uint16_t address, uint16_t CV, uint8_t Bit_data)
{
	//format of packet:
	// {preamble} 0 [ AAAAAAAA ] 0 111011VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (write)
	// {preamble} 0 [ AAAAAAAA ] 0 111001VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (verify/read)
	// {preamble} 0 [ AAAAAAAA ] 0 111010VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (bit manipulation)
	// only concerned with "write" form here!!!

	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	// split the CV address up among data uint8_ts 0 and 1
  // Modified, to avoid compiler warnings
	//uint8_t data[] = { ((CV >> 8) & 0b11) | 0b11101000, CV & 0xFF, Bit_data & 0x0F};
  uint8_t data[] = {
    static_cast<uint8_t>(((CV >> 8) & 0b11) | 0b11101000),
    static_cast<uint8_t>(CV & 0xFF),
    static_cast<uint8_t>(Bit_data & 0x0F)
  };


	p.addData(data, 3);
	p.setKind(pom_mode_programming_kind);
	p.setRepeat(ProgRepeat);

	//return low_priority_queue.insertPacket(&p);	//Standard

	return ops_programmming_queue.insertPacket(&p);
	//return e_stop_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
//POM - read CV value
bool DCCPacketScheduler::opsPOMreadCV(uint16_t address, uint16_t CV)
{
	//format of packet:
	// {preamble} 0 [ AAAAAAAA ] 0 111001VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (verify/read)

	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	// split the CV address up among data uint8_ts 0 and 1
  // Modified, to avoid compiler warnings
	// uint8_t data[] = { ((CV >> 8) & 0b11) | 0b11100100, CV & 0xFF, 0x00 };
  uint8_t data[] = {
    static_cast<uint8_t>(((CV >> 8) & 0b11) | 0b11100100),
    static_cast<uint8_t>(CV & 0xFF),
    static_cast<uint8_t>(0x00)
  };

	p.addData(data, 3);
	p.setKind(pom_mode_programming_kind);
	p.setRepeat(ProgRepeat);

	#if defined(GLOBALRAILCOMREADER)
	POMCVAdr = CV;
	#endif

	return ops_programmming_queue.insertPacket(&p);
	//return e_stop_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
//broadcast Decoder ResetPacket
bool DCCPacketScheduler::opsDecoderReset(uint8_t repeat)	//default RSTcRepeat
{
	// {long preamble} 0 00000000 0	00000000 0 EEEEEEEE	1
	// 1111111111111111111111111 0 00000000 0 00000000 0 00000000 1
	DCCPacket p(0);	//Adr = 0
	uint8_t data[] = { 0x00 };
	p.addData(data, 1);
	p.setKind(ops_mode_programming_kind); //always use short Adress Mode!
	p.setRepeat(repeat);
	return ops_programmming_queue.insertPacket(&p);
}

//---------------------------------------------------------------------------------
//broadcast e-stop command
void DCCPacketScheduler::eStop(void)
{
    // 111111111111 0 00000000 0 01DC0001 0 EEEEEEEE 1
	// C = by default contain one additional speed bit
	// D = direction ("1" the locomotive should	move in	the	forward	direction)

    DCCPacket e_stop_packet(0); //address 0
    uint8_t data[] = {0x61}; //01100001
    e_stop_packet.addData(data,1);
    e_stop_packet.setKind(e_stop_packet_kind);
	e_stop_packet.setRepeat(E_STOP_REPEAT);
    e_stop_queue.insertPacket(&e_stop_packet);

	setpower(ESTOP, true);

    return;
}

/*
//---------------------------------------------------------------------------------
//e-stop a specail loco
bool DCCPacketScheduler::eStop(uint16_t address)
{
    // 111111111111 0	0AAAAAAA 0 01001001 0 EEEEEEEE 1
    // or
    // 111111111111 0	0AAAAAAA 0 01000001 0 EEEEEEEE 1
    DCCPacket e_stop_packet(address);
    uint8_t data[] = {0x41}; //01000001
    e_stop_packet.addData(data,1);
//    e_stop_packet.setKind(e_stop_packet_kind);
	e_stop_packet.setRepeat(E_STOP_REPEAT);
//    e_stop_queue.insertPacket(&e_stop_packet);
    //now, clear this packet's address from all other queues
    //high_priority_queue.forget(address);	//no locos
	//low_priority_queue.forget(address);
	//repeat_queue.forget(address);
	//periodic_refresh_queue.forget(address);
	e_stop_packet.setKind(speed_packet_kind);
	repeat_queue.insertPacket(&e_stop_packet);
}
*/

//---------------------------------------------------------------------------------
//to be called periodically within loop()
//checks queues, puts whatever's pending on the rails via global current_packet
void DCCPacketScheduler::update(void) {
	//CV read packet is on Prog.Track:
  if (dccPacketEngine.isServiceModeRepeating()) {
		if (notifyCurrentSense) {	//get the Base rail current
			uint16_t current_load_now = notifyCurrentSense();
      if (dccPacketEngine.isFirstServiceModePacket()) {
        // First SM packet - get base current voltage:
				if (current_ack_status != WAIT_FOR_ACK) {
					LASTVAmpSence = current_load_now;  //store the last value
					current_ack_status = WAIT_FOR_ACK;
					#if defined(PROG_DEBUG)
					Serial.print(current_load_now);
					Serial.print(":");
					#endif
				}
			}
			else {
				//current load detect:
				if ((current_load_now > (LASTVAmpSence + ACK_SENSE_VALUE))) {
					if (current_ack_status == WAIT_FOR_ACK) {
						current_ack_status = ACK_DETECTED;
						ack_start_time = micros();
							#if defined(PROG_DEBUG)
							Serial.print(current_load_now);
							Serial.print(";");
							#endif

					}
				}
				else {
					if (current_ack_status == ACK_DETECTED) {
						if ( ((micros() - ack_start_time) / 1000) >= ACK_SENSE_MIN) {
							if ( ((micros() - ack_start_time) / 1000) <= ACK_SENSE_MAX) {	//sec.
								current_ack_status = ACK_READ_SUCCESS;
							}
							else current_ack_status = ACK_READ_FAIL;
						}
						else current_ack_status = WAIT_FOR_ACK;
					#if defined(PROG_DEBUG)
						Serial.print((micros() - ack_start_time) / 1000);
					#endif
					}
				}
			}
		} //ENDE notify function
	} //ENDE Service-Mode operation

	//Get next packet:
  // TODO: Old => remove!
  // if (dccPacketEngine.isWaiting)          //if the ISR needs a packet:
  if (dccPacketEngine.canAcceptPacket)       // if the DCC Hardware is able to receive a new packet
	{
		DCCPacket p;

		if (ops_programmming_queue.notEmpty()) {	//first Check if ops Service Mode (programming)?
			ops_programmming_queue.readPacket(&p);
		}
		else {
			//--------------------------- Service Mode ------------------------------------------------------------------------------------
			if (railpower == SERVICE) {
					switch (ProgState) {
						case ProgStart: {
							//choose the Mode!
							switch (ProgMode) {
								case ProgModeBit:
									ProgState = ProgBitRead;
									current_cv_value = 0;	//cv value that we read
								break;
								case ProgModeByte:
									ProgState = ProgVerifyCV;
									current_cv_value = 0;	//cv value that we start to read
								break;
								case ProgModeByteVerify:
									ProgState = ProgVerifyCV;	//check cv value
								break;
								case ProgModeWriteByte:
									ProgState = ProgWriteByte;
								break;
							}
							current_cv_bit = 0;		//bit in cv value we are working on
							cv_read_count = 0;		//counter for try to read data (repeat)
							LASTVAmpSence = 0;		//reset
							COUNTVAmpSence = 0;		//reset normal mA level
              dccPacketEngine.enterServiceMode();
							//Send Start Reset Packets:
							opsDecoderReset(RSTsRepeat);	//send first a Reset Start Packet
							ops_programmming_queue.readPacket(&p);
							ack_start_time = micros();
						break; }
						case ProgACKRead: {
							#if defined(PROG_DEBUG)
								if (current_ack_status == ACK_READ_SUCCESS)	//ACK from decoder
									Serial.print("A");
								else Serial.print("x");
								if (COUNTVAmpSence < 10)
									Serial.print("0");
								Serial.print(COUNTVAmpSence);
								Serial.print(" ");
							#endif

							COUNTVAmpSence = 0;	//reset
							LASTVAmpSence = 0;		//reset last read mA level

							switch (ProgMode) {
								case ProgModeBit:
									//Check Bit Status
									if (current_ack_status == ACK_READ_SUCCESS)  //CV read....?
										bitWrite(current_cv_value,current_cv_bit,1);	//ACK, so bit is 'one'!
									else bitWrite(current_cv_value,current_cv_bit,0);	//no ACK => 'zero'!
									current_cv_bit++;	//get next bit
									if (current_cv_bit <= 7)
										ProgState = ProgBitRead;
									else {
										#if defined(PROG_DEBUG)
											Serial.print(current_cv_value);
											Serial.print(" b");
											Serial.print(current_cv_value, BIN);
											Serial.print("; ");
										#endif
										ProgState = ProgVerifyCV;
										ProgMode = ProgModeBitVerify;
									}
									break;
								case ProgModeBitVerify: {
									#if defined(PROG_DEBUG)
									if (current_ack_status == ACK_READ_SUCCESS)
										Serial.println();
									#endif
									current_cv_bit = 0;		//reset
									ProgMode = ProgModeBit;
									//Check CV Value
									if (current_ack_status == ACK_READ_SUCCESS)
										ProgState = ProgSuccess;
									else {
										//Read again...
										ProgState = ProgBitRead;
										cv_read_count++;		//count times we try to read this cv!
										if (current_cv_value > 0) {		//there was min one ACK
											if (cv_read_count == CV_BIT_MAX_TRY_READ) {	//check if we should try again?
												if (ProgReadMode == 3) {	//try both Mode
													#if defined(PROG_DEBUG)
													Serial.println("Try Byte-Mode");
													#endif
													//------change Mode!!!----------
													ProgState = ProgStart;
													ProgMode = ProgModeByte;
												}
												else ProgState = ProgFail;	//byte verify fails!
											}
											#if defined(PROG_DEBUG)
											else {
												Serial.print("wrong ");
												Serial.print(cv_read_count);
												Serial.println(" again!!!");
											}
											#endif
										}
										else ProgState = ProgFail;	//no ACK while reading - "keine Lok gefunden!"
										current_cv_value = 0;	//reset
									}
									break; }
								case ProgModeByteVerify:
									if (current_ack_status == ACK_READ_SUCCESS)
										ProgState = ProgSuccess;
									else ProgState = ProgFail;
									break;
								case ProgModeByte:
									//Check Byte Status
									if (current_ack_status == ACK_READ_SUCCESS)
										ProgState = ProgSuccess;
									else {
										#if defined(PROG_DEBUG)
											Serial.print(current_cv_value);
											Serial.print(" b");
											Serial.println(current_cv_value, BIN);
										#endif
										current_cv_value++;	//check the next value
										ProgState = ProgVerifyCV;
										if (current_cv_value == 0) {	//we are at the end?
											cv_read_count++;		//count times we try to read this cv!
											ProgState = ProgStart;
											if (cv_read_count == CV_BYTE_MAX_TRY_READ) //check if we should stop to try?
												ProgState = ProgFail;
										}
									}
									break;
							}
							if ((ProgState == ProgSuccess) || (ProgState == ProgFail))
								opsDecoderReset(RSTsRepeat);	//send Reset start Packet -> wait if we get a next Service Mode packet!
							else opsDecoderReset(RSTcRepeat);	//send Reset continue Packet
							ops_programmming_queue.readPacket(&p);
							current_ack_status = NON_PROG_OP;		//reset ACK information
						break; }
						case ProgBitRead: {
							//Read CV in Bit-Mode:
							ProgState = ProgACKRead;
							opsReadCV(current_cv, current_cv_bit);	//ask for the next bit!
							ops_programmming_queue.readPacket(&p);
						break; }
						case ProgVerifyCV: {
							//Check CV Value
							ProgState = ProgACKRead;
							opsVerifyCV(current_cv,current_cv_value); //verify bit read
							ops_programmming_queue.readPacket(&p);
						break; }
						case ProgWriteByte: {
							//switch to Verify Mode:
							ProgState = ProgVerifyCV;
							ProgMode = ProgModeByteVerify;
							//Write Byte into CV:
							opsWriteCV(current_cv,current_cv_value);
							ops_programmming_queue.readPacket(&p);
						break; }
						case ProgEnde: {
							//switch to "normal" Mode!
							setpower(ON, true);		//force to leave Service Mode!
              dccPacketEngine.leaveServiceMode();
							return;	//no new packet here!
						break; }
					}
					//-----ACK FINISH-----
					if (ProgState == ProgSuccess) {	//lesen erfolgreich!
						//CV lesen erfolgreich
						if (notifyCVVerify)		//Verify the Value to device!
							notifyCVVerify(current_cv,current_cv_value);
						ProgState = ProgEnde;
					}
					if (ProgState == ProgFail) {	//lesen fehlgeschlagen!
						//Error:
						if (notifyCVNack)
							notifyCVNack(current_cv);
						ProgState = ProgEnde;
					}
			}	//ENDE Service-Mode
			//--------------------------- Normal Packet Mode ------------------------------------------------------------------------------------
				else {
          dccPacketEngine.leaveServiceMode();
					if (e_stop_queue.notEmpty() && (packet_counter % ONCE_REFRESH_INTERVAL)) {	//if there's an e_stop packet, send it now!
						e_stop_queue.readPacket(&p); //nothing more to do. e_stop_queue is a repeat_queue, so automatically repeats where necessary.
					}
					else {
						if (repeat_queue.notEmpty()) {	//for each packet to send it fast, before it stay long waiting in periodic!
							repeat_queue.readPacket(&p);
							periodic_refresh_queue.insertPacket(&p);
						}
						else {
							//	if (periodic_refresh_queue.notEmpty()) // && periodic_refresh_queue.notRepeat(last_packet_address))
							if (railpower != ESTOP)
								periodic_refresh_queue.readPacket(&p);
							else {
								//try again:
								if (e_stop_queue.notEmpty())
									e_stop_queue.readPacket(&p);
								else return; //no data to send!
							}
						}
						/*if (p.getKind() == speed_packet_kind && railpower == ESTOP) {
							e_stop_queue.readPacket(&p);
						}*/
					}
				}
		}
		++packet_counter;	//to not repeat only one queue!
		//last_packet_address = p.getAddress(); //remember the address to compare with the next packet
		//current_packet_size = p.getBitstream(current_packet); //feed to the starting ISR.

    // Old code - TODO: should be removed
    // dccPacketEngine.dataSize = p.getBitstream(dccPacketEngine.data); //feed to the starting ISR.
    // dccPacketEngine.isWaiting = false;

    uint8_t dccPacket[MaxDccSize];               // Buffer to be filled by getBitstream
    uint8_t size = p.getBitstream(dccPacket);    // Fills the DCC packet with data and return its size
    dccPacketEngine.send(dccPacket, size);       // Gives the DCC packet to the DCC hardware
	}
}

bool DCCPacketScheduler::getRailComStatus (void) {
	return dccPacketEngine.railComGap();
}


//--------------------------------------------------------------------------------------------
//aktuellen Zustand aller Funktionen und Speed der Lok
void DCCPacketScheduler::getLocoData(uint16_t adr, uint8_t data[])
{
	//uint8_t Steps, uint8_t Speed, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3		==> F0 bis F31
	byte Slot = LokStsgetSlot(adr);
	data[0] = LokDataUpdate[Slot].adr >> 14; 	//Steps
	data[1] = LokDataUpdate[Slot].speed;
	data[2] = LokDataUpdate[Slot].f0;	//F31 F30 F29 F0 - F4 F3 F2 F1
	data[3] = LokDataUpdate[Slot].f1;	//F12 - F5
	data[4] = LokDataUpdate[Slot].f2;	//F20 - F13
	data[5] = LokDataUpdate[Slot].f3;	//F28 - F21
}

//--------------------------------------------------------------------------------------------
//Gibt aktuelle Fahrtrichtung der Angefragen Lok zur�ck
byte DCCPacketScheduler::getLocoDir(uint16_t adr)
{
	return bitRead(LokDataUpdate[LokStsgetSlot(adr)].speed, 7);
}
//--------------------------------------------------------------------------------------------
//Gibt aktuelle Geschwindigkeit der Angefragen Lok zur�ck
byte DCCPacketScheduler::getLocoSpeed(uint16_t adr)
{
	return LokDataUpdate[LokStsgetSlot(adr)].speed & 0x7F;
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
byte DCCPacketScheduler::LokStsgetSlot(uint16_t adr)		//gibt Slot f�r Adresse zur�ck / erzeugt neuen Slot (0..126)
{
	byte Slot;
	for (Slot = 0; Slot < SlotMax; Slot++) {
		if ((LokDataUpdate[Slot].adr & 0x3FFF) == adr)
			return Slot;	//Lok gefunden, Slot ausgeben
		if ((LokDataUpdate[Slot].adr & 0x3FFF) == 0) {
			//Empty? neuer freier Slot - keine weitern Lok's!
			LokStsSetNew(Slot, adr);	//Eintragen
			return Slot;
		}
	}
	//kein Slot mehr vorhanden!
	//start am Anfang mit dem �berschreiben vorhandender Slots
	Slot = slotFullNext;
	LokStsSetNew(Slot, adr);	//clear Slot!
	slotFullNext++;
	if (slotFullNext >= SlotMax)
		slotFullNext = 0;
	return Slot;
}

//--------------------------------------------------------------------------------------------
void DCCPacketScheduler::LokStsSetNew(byte Slot, uint16_t adr)	//Neue Lok eintragen mit Adresse
{
	LokDataUpdate[Slot].adr = adr | (DCCdefaultSteps << 14); //0x4000; //0xC000;	// c = '3' => 128 Fahrstufen
	LokDataUpdate[Slot].speed = 0x80;	//default direction is forward
	LokDataUpdate[Slot].f0 = 0x00;
	LokDataUpdate[Slot].f1 = 0x00;
	LokDataUpdate[Slot].f2 = 0x00;
	LokDataUpdate[Slot].f3 = 0x00;

	//generate first drive information:
	#if defined(InitLocoDirect)
	setSpeed(LokDataUpdate[Slot].adr, LokDataUpdate[Slot].speed);
	#endif
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
#if defined(GLOBALRAILCOMREADER)
//Interrupt routine for reading via Serial
ISR(USART3_RX_vect) {
	DCCPacketScheduler::handle_RX_interrupt();	 //weiterreichen an die Funktion
}

//--------------------------------------------------------------------------------------------
// Interrupt handling for receive Data
// static
inline void DCCPacketScheduler::handle_RX_interrupt()
{
  if (active_object)
  {
	active_object->RailComReceive();	//Byte lesen
  }
}

//--------------------------------------------------------------------------------------------
//Reading RailCom Data:
void DCCPacketScheduler::RailComReceive(void) {
	uint8_t c = RailComDecodeInData(UDR3);
	if (RailComID > 0x3F) {
		if ((c >> 2) < 3) //app:pom, app:adr_low, app:adr_high -> read only 12 Bit!
			RailComID = c;
	}
	else if (RailComData == 0xFF)
		RailComData = c;
	/*
	if (c < 0x40) {    //g�ltige Railcom Message empfangen
		//Serial.print(c, BIN);
		//Serial.print(":");
	}
	*/
}

//Decoding RailCom Received Data
uint8_t DCCPacketScheduler::RailComDecodeInData(uint8_t in) {
  switch (in) {
    case 0b10101100: return 0x00;
    case 0b10110010: return 0x10;
    case 0b01010110: return 0x20;
    case 0b11000110: return 0x30;
    case 0b10101010: return 0x01;
    case 0b10110100: return 0x11;
    case 0b01001110: return 0x21;
    case 0b11001100: return 0x31;
    case 0b10101001: return 0x02;
    case 0b10111000: return 0x12;
    case 0b01001101: return 0x22;
    case 0b01111000: return 0x32;
    case 0b10100101: return 0x03;
    case 0b01110100: return 0x13;
    case 0b01001011: return 0x23;
    case 0b00010111: return 0x33;
    case 0b10100011: return 0x04;
    case 0b01110010: return 0x14;
    case 0b01000111: return 0x24;
    case 0b00011011: return 0x34;
    case 0b10100110: return 0x05;
    case 0b01101100: return 0x15;
    case 0b01110001: return 0x25;
    case 0b00011101: return 0x35;
    case 0b10011100: return 0x06;
    case 0b01101010: return 0x16;
    case 0b11101000: return 0x26;
    case 0b00011110: return 0x36;
    case 0b10011010: return 0x07;
    case 0b01101001: return 0x17;
    case 0b11100100: return 0x27;
    case 0b00101110: return 0x37;
    case 0b10011001: return 0x08;
    case 0b01100101: return 0x18;
    case 0b11100010: return 0x28;
    case 0b00110110: return 0x38;
    case 0b10010101: return 0x09;
    case 0b01100011: return 0x19;
    case 0b11010001: return 0x29;
    case 0b00111010: return 0x39;
    case 0b10010011: return 0x0A;
    case 0b01100110: return 0x1A;
    case 0b11001001: return 0x2A;
    case 0b00100111: return 0x3A;
    case 0b10010110: return 0x0B;
    case 0b01011100: return 0x1B;
    case 0b11000101: return 0x2B;
    case 0b00101011: return 0x3B;
    case 0b10001110: return 0x0C;
    case 0b01011010: return 0x1C;
    case 0b11011000: return 0x2C;
    case 0b00101101: return 0x3C;
    case 0b10001101: return 0x0D;
    case 0b01011001: return 0x1D;
    case 0b11010100: return 0x2D;
    case 0b00110101: return 0x3D;
    case 0b10001011: return 0x0E;
    case 0b01010101: return 0x1E;
    case 0b11010010: return 0x2E;
    case 0b00111001: return 0x3E;
    case 0b10110001: return 0x0F;
    case 0b01010011: return 0x1F;
    case 0b11001010: return 0x2F;
    case 0b00110011: return 0x3F;
    case 0x0F: return 0xF0;   //ACK
  }
  return 0xFF;
}
#endif
