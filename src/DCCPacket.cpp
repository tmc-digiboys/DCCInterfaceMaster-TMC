#include "DCCPacket.h"

bool     DCCPacket::nonLinearAddressing = false;
uint16_t DCCPacket::trntFormat          = 3;     // default ROCO

DCCPacket::DCCPacket(uint16_t new_address) //: address(new_address), kind(idle_packet_kind), size_repeat(0x40)
{
/*
  address_kind = DCC_SHORT_ADDRESS;			   //short or long Address needs to use (1 - 127)
  if (new_address > MAX_DCC_SHORT_ADDRESS) //use long Address
	address_kind = DCC_LONG_ADDRESS;
*/
  address = new_address;	//if address param is empty it's set per default to 0xFF!
  data[0] = 0x00;         //default idle packet with address 0xFF and data 0x00
  data[1] = 0x00;
  data[2] = 0x00;
  size_repeat = 0x40;
  kind = idle_packet_kind;
}

//uint8_t DCCPacket::getBitstream(uint8_t rawbytes[]) //returns size of array.
uint8_t DCCPacket::getBitstream(volatile uint8_t rawbytes[]) //returns size of array.
{
	int total_size = 1; //minimum size

	if (kind & MULTIFUNCTION_PACKET_KIND_MASK) {
		if (kind == idle_packet_kind) //idle packets work a bit differently:
		// since the "address" field is 0xFF, the logic below will produce C0 FF 00 3F instead of FF 00 FF
		{
			rawbytes[0] = 0xFF; //Leerlauf oder auch Idle Paket (255)
		}
		else if ((address > MAX_DCC_SHORT_ADDRESS) && (kind != ops_mode_programming_kind)) //This is a 14-bit address (128 - 10239)
		{
			rawbytes[0] = (uint8_t)((address >> 8) | 0xC0);  //(192-231)
			if (rawbytes[0] > 0xE7)		//231=0xE7
				rawbytes[0] = 0xE7;
			rawbytes[1] = (uint8_t)(address & 0xFF);
			++total_size;
		} else //we have an 7-bit address
		{
			rawbytes[0] = (uint8_t)(address & 0x7F);	// (1-127)
		}

		uint8_t i;
		for (i = 0; i < getSize(); ++i, ++total_size) {
			rawbytes[total_size] = data[i];
		}

		uint8_t XOR = 0;
		for (i = 0; i < total_size; ++i) {
			XOR ^= rawbytes[i];
		}
		rawbytes[total_size] = XOR;

		return total_size + 1;
	} else if (kind & ACCESSORY_PACKET_KIND_MASK) {
		if ((kind == basic_accessory_packet_kind) || (kind == extended_accessory_packet_kind)) {
			// Basic Accessory Packet looks like this:
			// {preamble} 0 10AAAAAA 0 1AAACAAD 0 EEEEEEEE 1
			// {preamble} 0 10AAAAAA 0 0AAA0AA1 0 DDDDDDDD 0 EEEEEEEE 1	(extACC)
			// or this:
			// {preamble} 0 10AAAAAA 0 1AAACDDD 0 (1110CCVV 0 VVVVVVVV 0 DDDDDDDD) 0 EEEEEEEE 1 (if programming)

			rawbytes[0] = 0x80;

			// address is the output address (client adres, one-based).
      // Common address calculation for basic and extended accessory decoders.
      // RCN-213: lin = address + TrntFormat, where address is one-based.
      // Example: user address 1 -> lin = 1 + 3 = 4 -> decoder address 1, TT = 0.
			uint16_t lin = address + DCCPacket::trntFormat;
			uint8_t  TT, LSB, MSB_bits;
			uint16_t outAddr = lin;

			if (nonLinearAddressing) {
        // Lenz non-linear addressing:
        // The linear and non-linear address sequences normally advance
        // together, except that the first four addresses of each 256-address
        // block (starting with block 1) are mapped to the preceding block.
        // This makes physical addresses 0–3 usable, although they are not
        // reached by the linear mapping because user address 1 starts at
        // physical address 4.
        //
        // Verified against measured reference values:
        // user addresses 253–256 -> physical addresses 0–3
        // user addresses 509–512 -> physical addresses 256–259
        // with the normal +3 offset immediately outside these ranges.

				uint16_t block = lin / 256;
				uint16_t pos   = lin % 256;
				if (pos < 4 && block >= 1) outAddr = lin - 256;
			}
			TT = outAddr & 0x03;
			uint16_t decAdr = outAddr >> 2;
			LSB = decAdr & 0x3F;
			MSB_bits = (decAdr >> 6) & 0x07;

			rawbytes[0] |= LSB;

			if (kind == extended_accessory_packet_kind) {
        // Extended accessory packet: 0aaa-0AA1
        // Bit 7 = 0, inverted MSB address bits, TT, bit 0 = 1.
				rawbytes[1] = ((~MSB_bits & 0x07) << 4) | (TT << 1) | 0x01;
			} else {
        // Basic accessory packet: 1aaa-CTTP
        // Bit 7 = 1, inverted MSB address bits, D, TT, and R.
				rawbytes[1] = 0x80 | ((~MSB_bits & 0x07) << 4) | (TT << 1) | (data[0] & 0x09);
			}

			//now, add any programming bytes (skipping first data byte, of course)
			uint8_t i;
			uint8_t total_size = 2;
			for (i = 1; i < getSize(); ++i, ++total_size) {
				rawbytes[total_size] = data[i];
			}

			//and, finally, the XOR
			uint8_t XOR = 0;
			for (i = 0; i < total_size; ++i) {
				XOR ^= rawbytes[i];
			}
			rawbytes[total_size] = XOR;

			return total_size + 1;
		}
	}
	return 0; //ERROR! SHOULD NEVER REACH HERE! do something useful,
  // like transform it into an idle packet or something! TODO
}

void DCCPacket::addData(uint8_t new_data[], uint8_t new_size) //insert freeform data.
{
  for(int i = 0; i < new_size; ++i)
    data[i] = new_data[i];
  size_repeat = (size_repeat & 0x3F) | (new_size<<6);
}
