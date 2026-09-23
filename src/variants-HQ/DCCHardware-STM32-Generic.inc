//************************************************************************************************
//
// File:            DCCHardware-STM_Generic.inc
// Purpose:         DCC Waveform Generator for STM32F4 processors
// Copyright 2026:  R.A. van Hoeijen
// History:         2026-003-04/ R.A> van Hoeijen - First version
//
// remark: in order to operate on an STM32H7 processor the main ino File
//         should have the following code statement SCB_DisableDCache();  // disable D-cache
//         as first line in the main setup . Also the STM32 H7 does not have eeprom on board so reading
//         or erasing it will definitly cause a crash, so assign in the scheduler in eeprom the values
//         as fixed default values. There is a ST defined stub for it but that does not work.
//
//  STM32F4 and STM32H7 series processor DMA/Timer based DCC generator ( Overview)
//
// This module defines a DCC Hardware Layer based upon DMA/Timer handling
// for STM32F4/DCCHardware_stm32h7xx based processors. This result in accurate usec timings for
// DCC streams and minimal processor loads. The interface routines are defined within this file and
// some general routines which are common for the processors. Hardware specific initialization is done
// in seperate files which ar include at the end.
//
// To generate the DCC stream, two buffers are used together with A DMA
// channel to provide a timer with the next compare value of a halfbit time
// for transmission on the DCC output pin.
//
// In the Interrupt Service Routine from the DMA channel the buffers shall be
// switched and coupled to the dma channel . When the first buffer is handled by DMA
// the second buffer will be filled with new values.
// A combination of DMA channel and a Timer in toggle compare mode is used  to minimize
// processor load. The DMA Channel feeds the timer compare register CCR1 , when the timer needs a new
// compare value it signals the DMA channel which in turn transfers the next value.
// In order to realize a a coherent DCC stream the DMA buffers contain the next half bit compare value.
// this means that a new value is the current compare value added with the  half bittime.
// the Timer is a 16 bit counter and by using the next compare value, the value can continously added because
// the overflow is taken into account.
//
// Synchronization with the application:
// -------------------------------------
// The public interface is synchronous and flag-based. The application calls
// newPacket() only when canAcceptPacket is true. Internally, user data is
// first copied into a staging buffer and later transferred to one of the two
// DMA buffers. This prevents the application from modifying memory that DMA
// is still using.
//
// GPIO pins
//--------------------------------------
// Due to the nature of gpio pin assignement within the STM32F4xx nucleo boards the Pins
// are fixed. This is a design decision  within the code.
// We have choosen to use 16 bit Timer3 and are in toggle mode using timer3-channel1. This means that
// the DCC stream pin is fixed to PA6. For the dccRailAuxPin we have choosen PA5.

//************************************************************************************************

//#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include "DCCHardware.h"
#include "HardwareTimer.h"
//******************************************************************************
// Part 1: Pins en output signals
//******************************************************************************
// Direct register access for pin changes
// Create a structure for each output signal. This structure provides
// direct access to the DxCore output registers, using a bitmask.
struct DccOut_t {
  volatile uint32_t *outset;   // BSRR register (set)
  volatile uint32_t *outclr;   // BSRR register (reset via <<16)
  volatile uint32_t *outtgl;   // ODR register (toggle)
  uint32_t bit;                // pin bitmask
  bool enable;
};


// Instantiate three objects; one for each output signal
static DccOut_t dccAuxPin;   // Create an instance for the dccAuxPin
static DccOut_t dccrailPin;
static DccOut_t DebugPin_1;
static DccOut_t DebugPin_2;

// Initialize a DCC output signal (pin) for direct register access
// Parameters:
//   - pinObj: DccOut_t object holding port registers and bit mask
//   - arduinoPin: Arduino pin number; set to 0xFF if unused
// For valid pins, this function:
//   - Sets the pin as OUTPUT
//   - Maps the Arduino pin to its PORTx registers
//   - Stores pointers to BSSR, ODR registers
//   - Stores the pin bit mask in the structure
//   - Disables toggling initially (enable = false)
// If the pin is 0xFF (not used), all pointers are set to nullptr and enable
// is false
inline void initDccPin(DccOut_t &pinObj, uint8_t arduinoPin)
{
  if (arduinoPin != 0xFF) {

    pinMode(arduinoPin, OUTPUT);

    PinName p = digitalPinToPinName(arduinoPin);
    GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(p));

    pinObj.outset = &(port->BSRR);
    pinObj.outclr = &(port->BSRR);
    pinObj.outtgl = &(port->ODR);

    pinObj.bit = STM_LL_GPIO_PIN(p);
    pinObj.enable = false;

    *pinObj.outclr = pinObj.bit << 16;   // start LOW

  } else {

    pinObj.outset = nullptr;
    pinObj.outclr = nullptr;
    pinObj.outtgl = nullptr;
    pinObj.bit = 0;
    pinObj.enable = false;
  }
}
inline void initDccPinAF(DccOut_t &pinObj, uint8_t arduinoPin, uint8_t af)
{
  if (arduinoPin == 0xFF) {
    pinObj.enable = false;
    return;
  }

  PinName p = digitalPinToPinName(arduinoPin);
  GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(p));

  uint32_t pin = STM_LL_GPIO_PIN(p);
  uint32_t index = __builtin_ctz(pin);

  // alternate function mode
  port->MODER &= ~(3U << (index * 2));
  port->MODER |=  (2U << (index * 2));

  if (index < 8) {
    port->AFR[0] &= ~(0xF << (index * 4));
    port->AFR[0] |=  (af << (index * 4));
  } else {
    port->AFR[1] &= ~(0xF << ((index - 8) * 4));
    port->AFR[1] |=  (af << ((index - 8) * 4));
  }

  pinObj.outset = &(port->BSRR);
  pinObj.outclr = &(port->BSRR);
  pinObj.outtgl = &(port->ODR);

  pinObj.bit = pin;
  pinObj.enable = true;

  *pinObj.outclr = pin << 16;   // start LOW
}
//******************************************************************************************************
//******************************************************************************************************
// 2. Defines
//******************************************************************************************************
/* =========================================================
 * DCC timing (µs)
 * ========================================================= */
#define TICKS_PER_US        1           // TIM @ 1 MHz
#define DCC_HALF_1_US       58
#define DCC_HALF_0_US       100
#define DCC_HALF_BIT_US     29
#define RAILCOMCUTOUT       480
#define PREAMBLE_BITS       16
#define MAX_DCC_LENGTH_BITS 144
#define RAILCOM_CUTOUT_LENGTH 4	  	// see RCN-217 (Ausgabe 24.11.2025)
#define RAILCOMPREDELAY  82
#define AUXRAILCOMCUTOUT    478


// #defines to set, clear and toggle the AUX output signals
// Although only some will be used, for completeness all options are given.
#define DCCAUX_HGH          {*dccAuxPin.outset = dccAuxPin.bit;   }
#define DCCAUX_LOW          {*dccAuxPin.outclr = dccAuxPin.bit << 16;}
#define DCCAUX_TGL          {*dccAuxPin.outtgl ^= dccAuxPin.bit;}
// #defines for debug pins
#define DEBUG_1_HGH         {*DebugPin_1.outset = DebugPin_1.bit;}
#define DEBUG_1_LOW         {*DebugPin_1.outset = DebugPin_1.bit << 16;}
#define DEBUG_1_TGL         {*DebugPin_1.outset ^= DebugPin_1.bit;}
#define DEBUG_2_HGH         {*DebugPin_2.outset = DebugPin_2.bit;}
#define DEBUG_2_LOW         {*DebugPin_2.outset = DebugPin_2.bit << 16;}
#define DEBUG_2_TGL         {*DebugPin_2.outset ^= DebugPin_2.bit;}
//******************************************************************************************************
// 3. Declaration and initialisation (constructor) of external object
//******************************************************************************************************
// Constructor definition
DccPacketEngine::DccPacketEngine(){
  	                          // Pin for a continuous DCC Signal, ignoring the RailCom gap / Railpower
  canAcceptPacket = true;                         // ISR is ready to receive the first packet from the user update() code
}

// dccPacketEngine is the object that controls packet transmission for different hardware
DccPacketEngine dccPacketEngine;


//******************************************************************************************************
// 4. Declaration and initialisation of internal data structures
//******************************************************************************************************
static volatile uint8_t idle_pkt[3] = { 0xFF, 0x00, 0xFF };
static volatile uint8_t reset_pkt[3] = { 0x00, 0x00, 0x00 };
HardwareTimer * RailComCutoutTimer;
extern "C" void RailComCutoutIRQ(void);
/* =========================================================
 * DMA buffer layout
 * ========================================================= */
typedef struct {
	uint8_t length;
	uint16_t data[MAX_DCC_LENGTH_BITS];
}MessageBuffer_Type;


// Define the structure for all variables used by the ISR
// The inital values for preambleLength is taken from RCN-211 (Ausgabe 23.07.2023)
// The inital values for preambleLengthSM is taken from RCN-216 (Ausgabe 24.11.2025)
struct DCC_ISR_State {
  volatile uint8_t nextPacket[MaxDccSize];
  volatile uint8_t nextPacketSize;
  volatile uint8_t currentPacket[MaxDccSize];    // Current packet being sent
  volatile uint8_t currentPacketSize;            // Number of bytes in current packet
  volatile uint8_t defaultPacket[3];             // Pre-generated default IDLE packet (Reset for SM)
  volatile uint8_t defaultPacketSize;            // Always 3 bytes
  volatile bool smActive;                        // SM active (long preamble, resets, SM flow allowed)
  volatile bool isServiceModePacket;             // We have to send Service Mode Packets
  volatile uint8_t smRepeatsRemaining;           // Number of retransmissions left for this SM packet
  volatile uint8_t smMaxRepeats;	               // Number of SM retransmissions after first send
  volatile bool railComCutoutInProgress;	       // In the gap, this flag is guaranteed to be true
  volatile bool railCom;                         // Should we generate a RailCom gap?
  volatile bool railcompredelay = false;         // extra delay time flag to indicate dma timing compensate is active
  volatile bool railComAllowedOnAux;
  volatile bool railComActiveOnAux;
  volatile bool railPower;	                     // Controls if the railsignal is on/off
  volatile uint8_t preambleLength = 17;          // Length of the preamble in normal operation
  volatile uint8_t preambleLengthSM = 20;        // Length of the preamble in Service Mode
  volatile uint8_t active_preamble_length =preambleLength; // Currently active preamble length
  volatile uint8_t Buf_in_Use = 1;                // indicator which message buffer is occupied in dma transfer
  volatile uint16_t *dcc_buffer;                  // pointer to message buffer when filled with new data
  volatile uint16_t PreviousTime;                 // last time for compare value in dma transfer used to make absolute compare times for bits
  volatile uint8_t nr_of_bytes = 0;               // number of elements in the meaage buffer to be tranferred by dma
  volatile bool is_initialized = false;
  volatile bool dccSignalInverted =  false;
  volatile bool AuxActiveLevel = 0;            // active level of auxpin 1 is high
  volatile MessageBuffer_Type DMAMesg_Buf1;          // message buffer used in dmas tranfer
  volatile MessageBuffer_Type DMAMesg_Buf2;
};

          // message buffer used in dma tranfer
static DCC_ISR_State dccISR;                     // Create an instance of the struct for the ISR

static void initIsr() {
  // Note that we do not (re)set railCom, railPower, preambleLength and preambleLengthSM,
  // since the user may already have set these values.

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
  // define default package
  dccISR.defaultPacket[0] = 0xFF;
  dccISR.defaultPacket[1] = 0x00;
  dccISR.defaultPacket[2] = 0xFF;
  dccISR.defaultPacketSize = 3;
  dccISR.PreviousTime =  100;
  dccISR.nr_of_bytes = 0;
  dccISR.railcompredelay= false;
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
  __disable_irq();
  for (uint8_t i = 0; i < size; i++) dccISR.nextPacket[i] = data[i];
  dccISR.nextPacketSize = size;
  dccPacketEngine.canAcceptPacket = false;      // ISR must first copy Packet
  __enable_irq();
}

//******************************************************************************
// 6. Service Mode interface methods
//******************************************************************************
void DccPacketEngine::enterServiceMode(void) {
  if (dccISR.smActive) return;
  // Enable Service Mode behaviour (long preamble, reset packets, SM flow).
  dccISR.smActive = true;
  // Pre-generate the reset packet (to make the ISR faster)
  __disable_irq();
  dccISR.defaultPacket[0] = 0;
  dccISR.defaultPacket[1] = 0;
  dccISR.defaultPacket[2] = 0;
  __enable_irq();
}

void DccPacketEngine::leaveServiceMode(void) {
  // Leave Service Mode immediately. Ongoing SM retransmissions are cancelled.
  if (!dccISR.smActive) return;
  dccISR.smActive = false;
  dccISR.smRepeatsRemaining = 0;
  // Pre-generate the idle packet (to make the ISR faster)
  __disable_irq();
  dccISR.defaultPacket[0] = 0xFF;
  dccISR.defaultPacket[1] = 0;
  dccISR.defaultPacket[2] = 0xFF;
  __enable_irq();
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

// The following two functions are not used by the Z21PG variants
void DccPacketEngine::setAuxActiveLevel(bool activeHigh) {
  //if (dccISR.AuxActiveLevel == activeHigh) return;
  if (activeHigh)
  {
    DCCAUX_HGH;
  }
  else
  {
    DCCAUX_LOW;
  }
  dccISR.AuxActiveLevel = activeHigh;
 }

// Enable / disable generation of a RailCom gap pulse on the Aux pin
void DccPacketEngine::setRailComGapInAux(bool useForRcCutout) {
  dccISR.railComAllowedOnAux = useForRcCutout;
  dccISR.railComActiveOnAux = dccISR.railComAllowedOnAux;
}

// Not used by this variant
void DccPacketEngine::useMonitorPinforInrush(bool value) {(void)value;}

//******************************************************************************
// 9. Start / Stop interface methods
//******************************************************************************
void DccPacketEngine::StopOutputSignal() {
  // This routine may be called when a short circuit is detected or when the user
  // presses the emergency stop button. In that case the normal
  // must be forced LOW, so no DCC power is present on
  // the rails.
	if (!dccISR.railPower) return;
  dccISR.railPower = false;
  if (dccISR.AuxActiveLevel)
  {
     DCCAUX_LOW;
  }
  else
  {
     DCCAUX_HGH;
  }

}

void DccPacketEngine::RunOutputSignal() {
  // Enabling power is delicate, because we must ensure that the
  // DCC signals always remain low

if (dccISR.railPower) return;
  dccISR.railPower = true;
  if (dccISR.AuxActiveLevel)
  {
    DCCAUX_HGH;
  }
  else
  {
    DCCAUX_LOW;
  }
}





void DccPacketEngine::setDccSignalInverted(bool inverted) {
  // Inverts the polarity of the DCC signal. No impact on AUX signal
  if (dccISR.dccSignalInverted == inverted) return;
  dccISR.dccSignalInverted = inverted;
  if (dccISR.is_initialized) {
    canAcceptPacket = false;
    if (dccISR.dccSignalInverted)
    {
      TIM3->CCER |= TIM_CCER_CC1P;   // invert output polarity
    }
    else
    {
      TIM3->CCER &= ~TIM_CCER_CC1P;
    }

  }
}

/* =========================================================
 * DCC emit helpers (absolute CCR times)
 * ========================================================= */

 //
 // This incline function takes care of writing the absolute bit times
 // in the transmit buffer
static inline void Write_Buffer(uint16_t us)
{
    // message handles absolute time so the pervious time should be added
    // to buffer in order to satisfy the trigger values
    dccISR.PreviousTime  += us;

    //
    // dcc_buffer points to one of the two dma channel buffers
    // when one is currently in process of transmittion nexct buffer
    // will be filled
    *dccISR.dcc_buffer = (uint16_t) dccISR.PreviousTime;

    // increment buufer pointer for next value
    dccISR.dcc_buffer++;
    //
    // keep tranck of the number of values in
    // the buffer for transfer purposes
    dccISR.nr_of_bytes++;
}

// write the two halfs for a 1 bit in the selected message buffer
static inline void emit_1(void)
{
   Write_Buffer(DCC_HALF_1_US);
   Write_Buffer(DCC_HALF_1_US);
}
// write the tow hald bits for a zero bit in the assiigned message buffer
static inline void emit_0(void)
{
    Write_Buffer(DCC_HALF_0_US);
    Write_Buffer(DCC_HALF_0_US);
}

// write the rail com gap with the half-half 1 bit in message buffer
// and the remaining time for the real gap
static inline void emit_RailcomCutout(void)
{
  //
  // railcom has a 29 usec pulse before sending a low signal
  // So add thie 29 usec value
	Write_Buffer( DCC_HALF_BIT_US);

  // Railconm cutout value include half 29 usec value to keep low signal
	Write_Buffer( RAILCOMCUTOUT);
}
/* =========================================================
 * Fill buffer with  packet
 * ========================================================= */
static void fill_Buffer(volatile uint8_t *pkt,
                      uint8_t size)
{

     // reset # of elements to get new number for packet
	   dccISR.nr_of_bytes = 0;

     // check which message buffer is vurrently in use
     // to determine which buffer to fill and allocate pointer
	   if ( dccISR.Buf_in_Use == 1)
	   {
		   dccISR.dcc_buffer = dccISR.DMAMesg_Buf2.data;
	   }
	   else
	   {
		   dccISR.dcc_buffer = dccISR.DMAMesg_Buf1.data;
	   }

    // check if service mode is enable because that changes preamble length
	   if ( dccPacketEngine.isServiceModeEnabled()){
		   dccISR.active_preamble_length = dccISR.preambleLengthSM;
	   }

	   else
     {

       // check if railcom is activated and in this place also
       // there is no service mode message. preamble will be
       // according railcom or normal
       // railcom gap is considered to be the first part of the new message
		   if (dccISR.railCom){
			   dccISR.active_preamble_length =dccISR.preambleLength - RAILCOM_CUTOUT_LENGTH;

    	   emit_RailcomCutout();
		   }
		   else{
			  dccISR.active_preamble_length =dccISR.preambleLength;
		   }

	   }
     // place correct preamble in buffer to start message eventualy after railcom gap
       for (int i = 0; i < dccISR.active_preamble_length; i++)
           emit_1();

        // place the bits of the new message in the dma messgae buffer
       for (int b = 0; b < size; b++) {
           emit_0();
           for (int i = 7; i >= 0; i--)
               (pkt[b] & (1U << i)) ? emit_1() : emit_0();
      }

       emit_1(); // end bit

       // place number of elements by correct dma message buffer

       if (dccISR.Buf_in_Use ==1)
       {
    	   dccISR.DMAMesg_Buf2.length=dccISR.nr_of_bytes;
       }
       else
       {
    	   dccISR.DMAMesg_Buf1.length= dccISR.nr_of_bytes;
       }

}


/* ===========================================================================
 * Timer ISR for handling accurate railcomgap timing
 * =========================================================================== */

extern "C" void RailComCutoutIRQ(void)
{

   if  (dccISR.railcompredelay)
   {
     // predelay is expired real railcom gap should be activated on external pin
     if (dccISR.AuxActiveLevel)
     {
        DCCAUX_LOW;
     }
     else
     {
        DCCAUX_HGH;
     }

     RailComCutoutTimer->setOverflow( AUXRAILCOMCUTOUT, TICK_FORMAT);

     // reset predelay flag
     dccISR.railcompredelay = false;
     RailComCutoutTimer->refresh();   // force ARR update
     TIM4->CNT = 0;
     // Start timer
     RailComCutoutTimer->resume();
     // signal railcom gap is in progress
     dccISR.railComCutoutInProgress = true;

   }
   else
   {
    // reset gpio pin to low lever and stop timer

    if (dccISR.AuxActiveLevel)
     {
         DCCAUX_HGH;
     }
     else
     {
       DCCAUX_LOW;
     }
    RailComCutoutTimer->pause();
    dccISR.railComCutoutInProgress = false;
   }

}

/* =========================================================================
 * DMA Handling ISR
 * Consist of two parts :
 * 1. wil switch the message buffers to garantee a continous dcc stream with
 *    railcom if enabled and enable dma asap to allow for dma transfers
 *
 * 2. will fill the released buffer with a new packet if available or
 *    with depending on if service mode is enable a braodcast reset packet or
 *    an idle packet
 * ========================================================================== */
extern "C" void DMA1_Stream4_IRQHandler(void)
{


    //
    // First part of ISR is to change the buffers assigned to the dma channels
    // in order to let the dma channel feed the dcc signal continously.
    // As sson as possible the dma is again enabled.
    // Second Part of ISR is to prepare the next message in the buffer

    if (DMA1->HISR & DMA_HISR_TCIF4)
    {
        // 1️⃣ Disable stream
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        while (DMA1_Stream4->CR & DMA_SxCR_EN);
    	 // Clear alle relevante flags
    	DMA1->HIFCR =
                  DMA_HIFCR_CTCIF4 |
                  DMA_HIFCR_CHTIF4 |
                  DMA_HIFCR_CTEIF4 |
                  DMA_HIFCR_CDMEIF4 |
                  DMA_HIFCR_CFEIF4;

     // two message buffers are used if one is servicing the dma channel
     // the scond one will be filled again.
     // A check is done which buffer is in use so the new buffer for dma is
     // set in the dma registers with is number of transactions

      if (dccISR.Buf_in_Use == 2)
      {
          // set new buffer in dma registers
          DMA1_Stream4->M0AR = (uint32_t)dccISR.DMAMesg_Buf1.data;
          // set new value for number of transfers
          DMA1_Stream4->NDTR = dccISR.DMAMesg_Buf1.length;
          // set flag buffer 1 is in use
          dccISR.Buf_in_Use = 1;

      }
      else
      {
          // next buffer is buffer 2 ready for transfer
          DMA1_Stream4->M0AR = (uint32_t)dccISR.DMAMesg_Buf2.data;

          // set new number of transfers
          DMA1_Stream4->NDTR = dccISR.DMAMesg_Buf2.length;
          // buffer in use is buffer 2
          dccISR.Buf_in_Use = 2;


      }
      // check if a Railcom signale has to be provided on the external pin
      // Will not be done in service mode or power is off
      if ( dccISR.railCom && !dccPacketEngine.isServiceModeEnabled() && dccISR.railPower && dccISR.railComActiveOnAux){
        // set pre delay value for compenstating ongoing dma
        // transfer and after this time expires really start railcom gap
        // delay is measured to be 50 usec with logic state analyzer
        RailComCutoutTimer->setOverflow(RAILCOMPREDELAY,  TICK_FORMAT);
         dccISR.railcompredelay = true;
        // Start timer
        RailComCutoutTimer->resume();
        dccISR.railComCutoutInProgress = true;

      }

    	// restart dma channel
    	DMA1_Stream4->CR |= DMA_SxCR_EN;

    	// We can proces a new packet provided we don't have to repeat earlier SM packets
    	if (!dccPacketEngine.isServiceModeRepeating())
      {
    	    // Did the application code already gave us a new packet?
    	    // In that case dccPacketEngine.canAcceptPacket is false
    	    if (!dccPacketEngine.canAcceptPacket)
          {
    	    // use next user packet
    	     	  fill_Buffer(dccISR.nextPacket,dccISR.nextPacketSize);
    	       	for (uint8_t i = 0; i < dccISR.nextPacketSize; i++)
    	                  dccISR.currentPacket[i] = dccISR.nextPacket[i];
    	        dccISR.currentPacketSize = dccISR.nextPacketSize;
    	        // Setup SM retransmission if required
    	        if (dccISR.isServiceModePacket)
    	             dccISR.smRepeatsRemaining = dccISR.smMaxRepeats;
    	    }
          else
          {
    	        // No new packet: load default packet
    	        fill_Buffer(dccISR.defaultPacket, 3 );

    	    }
    	    dccPacketEngine.canAcceptPacket = true; // ready for next packet
      }
      else
      {
    	    // still repeating SM packet, decrement counter
          if (dccISR.smRepeatsRemaining > 0)
          {
             dccISR.smRepeatsRemaining = dccISR.smRepeatsRemaining - 1;
             fill_Buffer(dccISR.currentPacket, dccISR.currentPacketSize);
          }
      }

    }


}
#if defined(STM32F4xx)
    #include "DCCHardware-STM32F4xx.inc"
  #elif defined(STM32H7xx)
    #include "DCCHardware-STM32H7xx.inc"
  #endif
