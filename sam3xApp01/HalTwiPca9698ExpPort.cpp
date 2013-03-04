/*
 * HalTwiPca9698ExpPort.cpp
 * Copyright (c)2012 Azonde, All Rights Reserverd.
 
 * Created: 10/25/2012 5:45:04 PM
 *  Author: neil
 *
 *  Description -
 *
 *  
 */ 
#include "sam.h"
#include "Arduino.h"
#include "az23project.h"
#include "sysDebug.h"
#define cOffsetDbgMask MASK_INDX_SystemMngXxx
#include "HalTwiPca9698.h"
#include "rstc.h"


/* Mode
 * B7 6 5 - 0
 * B4 0 SMBA - doesn't respond to Alert Response Address. Uses INT- line
 * B3 0 IOAC - oesn;t respond to GPIO All Call
 * B2 0
 * B1 0 OCH outputs change on STOP cmd
 * B0 0 OEPOL - OE active low
 * Default 0x02 - needs updating 
 */
#define cPca9698_Mode 0x00

/* Interrupt Mask for Bank0
 * D00,D01,D03,D05 are interrupts active
 */

typedef enum {
//Mask definitions for Bank0 ~ Outputs and only port with Inputs 
   epi0ExpInt1_m=0x01,
   epi0ExpInt2_m=0x02,
   epi0CrdSwSt_m=0x04,
   ep0PwrUsb6En=0x10,
   epi0ExtAwakeSwSt_m=0x20,
   epi0Mask=(epi0ExtAwakeSwSt_m|epi0CrdSwSt_m|epi0ExpInt2_m|epi0ExpInt1_m),
   ep0Rs232InEnN=0x40,
   ep0Rs232FoffEnN=0x80,
} ePortInputs_bank0;

/* Output configuration for outputs
 * D7 1 OUt4 Bank4 - all Totem
 * D6 1 Out3 Bank3 - all Totem
 * D5 1 Out2 Bank2 - all Totem
 * D4 1 Out1 Bank1 - all Totem
 * D3 1 Out067 Bank0 - Totem
 * D2 1 Out045 Bank0 - Totem -D05 input no effect
 * D1 1 Out023 Bank0 - Totem - input no effect
 * D0 1 Out001 Bank0 - Totem  - actually input no effect
 * Default to 0xff - so no change
 */
#define cPca9698OutConf 0xFF


extern "C" {  

#define cPca9698Regs  5

const uint8_t Pca9698SetPortsInactive[cPca9698Regs] = {
	epi0Mask|ep0Rs232FoffEnN, //Bank0  '1' not used
	0x00, //Bank1 outputs
	0xE0, //2 - POwer led Act 1
	0xFF, //3
	0x3E //4
};
const uint8_t Pca9698ConfigReg[cPca9698Regs]= {
 epi0Mask,   //Bank0 input '1' for D00,D01,D03,D05
            0x00, 0x00, 0x00, 0x00 };

const uint8_t Pca9698ConfigAllInput[cPca9698Regs]= {
  0x3f,   // Bank0 All inputs except RS232
			0xff, //Bank1
			0xef,//&(~epb2EnPwrLed_m)), //Bank2 except EnPwrLed
			0xff, //Bank3
			0xff //Bank4
			};

const uint8_t pca9698interruptMask[] = {
	epi0Mask,   //Bank0 input '1' for D00,D01,D03,D05 to enable interrupts
	        0x00,0x00,0x00,0x00 	};
			
const uint8_t pca9698ledstMask[] = {
			0x00,   //Bank0 no leds
			0x00, //Bank1 no leds
			0xf0, //Bank2 all leds B4 turns off power led
			0xff, //Bank3 all leds
			0x3e, //Bank4 00111110b//Bank4
	};
//The led masks definitions are on 4 bytes/32bits covering bank2,3,4, (not bank0 or bank1)
#define cHtpLedOffset 2
//#define cHtpLedMask    0x0003EFFF0L
#define cHtpLedMask    0x0003FFFF0L
#define cHtpLedActMask 0x0003EFFE0L
#define cHtpLedNotMask 0x000010000L
#define cHtpLedFirstMask      0x10
#define cHtpLedNumber           21

//const uint8_t pca9698ploarity[] = {
//	0x00,0x00,0x00,0x00,0x00 };			
/* const Pca9698_t portDirection[]= {
// config Polarity  0xFFconf
//Bank 0 mixed use input and output
   {epi0Mask,   //Bank0 input '1' for D00,D01,D03,D05
           0x00,//Bank0 interrupts caused by level change
                     0x00},//Bank0
   {0x00,   0x00,    0x00},//Bank1 all outputs
   {0x00,   0x00,    0x00},//Bank2 all outputs
   {0x00,   0x00,    0x00},//Bank3 all outputs
   {0x00,   0x00,    0x00},//Bank4 all outputs
};
*/

} //Extern 'C'

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
HalExpansionPortClass::HalExpansionPortClass(void) {

  }//::HalExpansionPortClass

  

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
result_t HalExpansionPortClass::portHwRefresh() {
//#define us_start_address 0 //Bank 0
result_t retResult;
/* PCA internal address length */


 // if (FAIL==HplTwi1_Request(0) ) {
//	  //dbgOut(eDbgAll_??,'A',"HalTwiPca write fail");
//	  return FAIL;
//	  }
  
  retResult = writePcaPorts(cPca9698cmd_OpReg,portStateOutput,ehpPortNumber);
  //HplTwi1_Release(0);
  return retResult;
}	


/**********************************************
 * \brief 
 *
 *
 * \return none
 */
result_t HalExpansionPortClass::initilizePcaPorts() {
	 //uint8_t portStateOutput[ehpPortBuffSize];
	 result_t retResult=SUCCESS;
	ledBitWalk_bit = cHtpLedFirstMask;
	ledTest_bool = 0;//TODO False?	 

	//set direction of Pca9698 hardware ports bits
	retResult = writePcaPorts(cPca9698cmd_CfgReg,(uint8_t *)Pca9698ConfigReg,ehpPortNumber);
	//Todo: FUT if (FAIL == retResult) call error handler
	
	//Initialize the Pca9698 internal registers - the master source is portStateOutput[] 
	memcpy(portStateOutput,Pca9698SetPortsInactive,cPca9698Regs);
	retResult = writePcaPorts(cPca9698cmd_OpReg,(uint8_t *)Pca9698SetPortsInactive,ehpPortNumber);
	
	//Todo: FUT Add interrupt settings

	return retResult;
}

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
void HalExpansionPortClass::goLowPowerMode() {
	 //uint8_t portStateOutput[ehpPortBuffSize];
	 result_t retResult=SUCCESS;
 
	//HalExpansionPort.bitClr(epbtRs232FoffEnN);
	//HalExpansionPort.portHwRefresh();
	//set direction of Pca9698 hardware ports bits
	retResult = writePcaPorts(cPca9698cmd_CfgReg,(uint8_t *)Pca9698ConfigAllInput,ehpPortNumber);
	//retResult = writePcaPorts(cPca9698cmd_CfgReg,(uint8_t *)Pca9698ConfigReg,ehpPortNumber);
	//Todo: FUT if (FAIL == retResult) call error handler
	
    //rstc_reset_extern(RSTC);//Also Reset peripherals reset - TwiPort, EthernetMod, Xbp,MtCell, Arduino
	delay(1);
  disableVirtualPin(HwSdaLTwi1);
  disableVirtualPin(HwSclLTwi1); //Program the controller to dedicate TWD/TCK

	return;
}
/**********************************************
 * \brief 
 *
 *
 * \return none
 */
result_t HalExpansionPortClass::writePcaPorts(uint32_t iRegister, uint8_t *pBuffer, uint32_t bufLength) {

uint32_t bufCnt=bufLength;
uint32_t breakOut_cnt=0;
result_t retResult=SUCCESS;

	TWI_StartWrite(HW_ADDR_TWI_PCA9698,HW_PCA9698_TwiAddr, iRegister,HW_PCA9698_InternalAddrLen,*pBuffer);
	pBuffer++;
	bufCnt--;

	while(bufCnt >0) {
		//if no NACK and Byte sent, then transmit next one
		if (TWI_SR_NACK & TWI_GetStatus(HW_ADDR_TWI_PCA9698)) {
						  dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca Unexpected NACK ",bufCnt);
			retResult=FAIL;
			break;
		}
		if (!TWI_ByteSent(HW_ADDR_TWI_PCA9698)) {
		   if (90000 < ++breakOut_cnt) {
			  dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca TWI_SR_TXRDY exceeded ",bufCnt);
			  retResult=FAIL;
			  break; //Out while
			}
			continue;				
		}
		breakOut_cnt = 0;
		TWI_WriteByte(HW_ADDR_TWI_PCA9698,*pBuffer);
		pBuffer++;
		bufCnt--;
	}
	
TWI_Stop(HW_ADDR_TWI_PCA9698);
while (!TWI_TransferComplete(HW_ADDR_TWI_PCA9698)) {
	if (90000 < ++breakOut_cnt) {
	   dbgOut(eDbgAll_errEvt,'B',"HalTwiPca TWI_SR_TXCOMP exceeded ");
	   break; //Out while
	}
}
 
 return retResult;
 }//portInit
#define PORTS_DEBUG
 // ***************************************************************
 inline void HalExpansionPortClass::portSet(eHalPhysicalPorts portNumber,uint8_t bitmask) {
#ifdef PORTS_DEBUG
if (ehpPortBuffSize< portNumber){
	dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca portSet invalid port ",portNumber);
	return;
}
#endif
	 portStateOutput[portNumber] |= bitmask;
 }
  // ***************************************************************
  inline void HalExpansionPortClass::portClr(eHalPhysicalPorts portNumber,uint8_t bitmask) {
#ifdef PORTS_DEBUG
if (ehpPortBuffSize< portNumber){
	dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca portClr invalid port ",portNumber);
	return;
}
#endif
	  portStateOutput[portNumber] &= bitmask;
  }
  // ***************************************************************
  inline void HalExpansionPortClass::portToggle(eHalPhysicalPorts portNumber,uint8_t bitmask) {
#ifdef PORTS_DEBUG
  	 if (ehpPortBuffSize< portNumber){
		 dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca portToggle invalid port ",portNumber);
		 return;
	 } 
#endif

	  portStateOutput[portNumber] ^= bitmask;
  }
// ***************************************************************
 void HalExpansionPortClass::bit_Set(ePca9698Bits gpio ) {
	  unsigned byte = gpio / 8;
	  unsigned bit = gpio % 8;
#ifdef PORTS_DEBUG
if (epbtTooFar <= gpio){
	dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca bitSet invalid ",gpio);
	return;
}
#endif
	  portStateOutput[byte] |= (1 << bit);
  }
// ***************************************************************
void HalExpansionPortClass::bitClr(ePca9698Bits gpio )  {
	unsigned byte = gpio / 8;
	unsigned bit = gpio % 8;

#ifdef PORTS_DEBUG
if (epbtTooFar <= gpio){
	dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca bitClr invalid ",gpio);
	return;
}
#endif
	portStateOutput[byte] &= ~(1 << bit);
 }

// ***************************************************************
void HalExpansionPortClass::bitToggle(ePca9698Bits gpio )  {
	unsigned byte = gpio / 8;
	unsigned bit = gpio % 8;

#ifdef PORTS_DEBUG
if (epbtTooFar <= gpio){
	dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca bitToggle invalid ",gpio);
	return;
}
#endif
	portStateOutput[byte] ^= (1 << bit);
}
   // ***************************************************************
   inline void HalExpansionPortClass::ledsSet(uint32_t bitmask) {
	 if (!(cHtpLedNotMask & bitmask)){
		 dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca ledsSet not a Led ",bitmask);
		 return;
	 }
	   ((uint32_t &) portStateOutput[1]) |= bitmask;
   }
   // ***************************************************************
   inline void HalExpansionPortClass::ledsClr(uint32_t bitmask) {
	 if (!(cHtpLedNotMask & bitmask)){
		 dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca ledsClr not a Led ",bitmask);
		 return;
	 }
	   ((uint32_t &) portStateOutput[1]) &= bitmask;
   }
   // ***************************************************************
 void HalExpansionPortClass::ledsToggle(uint32_t bitmask) {
	 if (!(cHtpLedNotMask & bitmask)){
	   dbgOut1(eDbgAll_errEvt,'B',"HalTwiPca ledsToggle not a Led ",bitmask);
	   return;
	 }			   

	   ((uint32_t &) portStateOutput[1]) ^= bitmask;
   }

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
   void HalExpansionPortClass::ledBitWalk() {

    uint32_t *p_temp;
	p_temp  = ((uint32_t *) &portStateOutput[cHtpLedOffset]);//Start of leds

    
	ledBitWalk_bit <<= 1;
	if (cHtpLedNotMask & ledBitWalk_bit) {
		//Take it over the one bit not a led
		ledBitWalk_bit <<= 1;
	}

	if  (0 == ( cHtpLedMask & ledBitWalk_bit)) {
		//No led bits are set
		ledBitWalk_bit = cHtpLedFirstMask;
	}
	*p_temp = (ledBitWalk_bit ^ cHtpLedActMask);
	portHwRefresh();
     }

// ***************************************************************
void HalExpansionPortClass::ledTestStart() {
	ledTest_bool = true;
}
// ***************************************************************
void HalExpansionPortClass::ledTestStop() {
	ledTest_bool = false;
}
// ***************************************************************
void HalExpansionPortClass::timerTick() {

	if (false != ledTest_bool) {
//			Serial.println("-doit");
		ledBitWalk();
	}		
	
 }//timerTick
  

	
	
extern "C" {  


#if 0


  
http://code.google.com/searchframe#4Wo8TiCwikI/include/&q=pca9698.h%20package:u-boot-mini6410%5C.googlecode%5C.com
#ifndef __PCA9698_H_
#define __PCA9698_H_

int pca9698_request(unsigned gpio, const char *label);
void pca9698_free(unsigned gpio);
int pca9698_direction_input(u8 addr, unsigned gpio);
int pca9698_direction_output(u8 addr, unsigned gpio, int value);
int pca9698_get_value(u8 addr, unsigned gpio);
int pca9698_set_value(u8 addr, unsigned gpio, int value);

#endif /* __PCA9698_H_ */
http://code.google.com/p/u-boot-mini6410/source/browse/drivers/gpio/pca9698.c?r=9f8e407f82a4ed4eb8bbcbb2b067903188cba842
/*****************************************************************************
 * possible idea
 * Driver for NXP's pca9698 40 bit I2C gpio expander
 */

#include <common.h>
#include <i2c.h>
#include <asm/errno.h>
#include <pca9698.h>

/*
 * The pca9698 registers
 */

#define PCA9698_REG_INPUT               0x00
#define PCA9698_REG_OUTPUT              0x08
#define PCA9698_REG_POLARITY            0x10
#define PCA9698_REG_CONFIG              0x18

#define PCA9698_BUFFER_SIZE             5
#define PCA9698_GPIO_COUNT              40

static int pca9698_read40(u8 addr, u8 offset, u8 *buffer)
{
        u8 command = offset | 0x80;  /* autoincrement */

        return i2c_read(addr, command, 1, buffer, PCA9698_BUFFER_SIZE);
}

static int pca9698_write40(u8 addr, u8 offset, u8 *buffer)
{
        u8 command = offset | 0x80;  /* autoincrement */

        return i2c_write(addr, command, 1, buffer, PCA9698_BUFFER_SIZE);
}

static void pca9698_set_bit(unsigned gpio, u8 *buffer, unsigned value)
{
        unsigned byte = gpio / 8;
        unsigned bit = gpio % 8;

        if (value)
                buffer[byte] |= (1 << bit);
        else
                buffer[byte] &= ~(1 << bit);
}

int pca9698_request(unsigned gpio, const char *label)
{
        if (gpio >= PCA9698_GPIO_COUNT)
                return -EINVAL;

        return 0;
}

void pca9698_free(unsigned gpio)
{
}

int pca9698_direction_input(u8 addr, unsigned gpio)
{
        u8 data[PCA9698_BUFFER_SIZE];
        int res;

        res = pca9698_read40(addr, PCA9698_REG_CONFIG, data);
        if (res)
                return res;

        pca9698_set_bit(gpio, data, 1);

        return pca9698_write40(addr, PCA9698_REG_CONFIG, data);
}

int pca9698_direction_output(u8 addr, unsigned gpio, int value)
{
        u8 data[PCA9698_BUFFER_SIZE];
        int res;

        res = pca9698_set_value(addr, gpio, value);
        if (res)
                return res;

        res = pca9698_read40(addr, PCA9698_REG_CONFIG, data);
        if (res)
                return res;

        pca9698_set_bit(gpio, data, 0);

        return pca9698_write40(addr, PCA9698_REG_CONFIG, data);
}

int pca9698_get_value(u8 addr, unsigned gpio)
{
        unsigned config_byte = gpio / 8;
        unsigned config_bit = gpio % 8;
        unsigned value;
        u8 data[PCA9698_BUFFER_SIZE];
        int res;

        res = pca9698_read40(addr, PCA9698_REG_INPUT, data);
        if (res)
                return -1;

        value = data[config_byte] & (1 << config_bit);

        return !!value;
}

int pca9698_set_value(u8 addr, unsigned gpio, int value)
{
        u8 data[PCA9698_BUFFER_SIZE];
        int res;

        res = pca9698_read40(addr, PCA9698_REG_OUTPUT, data);
        if (res)
                return res;

        pca9698_set_bit(gpio, data, value);

        return pca9698_write40(addr, PCA9698_REG_OUTPUT, data);
}


#ifndef _I2C_H_
#define _I2C_H_

/*
 * WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
 *
 * The implementation MUST NOT use static or global variables if the
 * I2C routines are used to read SDRAM configuration information
 * because this is done before the memories are initialized. Limited
 * use of stack-based variables are OK (the initial stack size is
 * limited).
 *
 * WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
 */

/*
 * Configuration items.
 */
#define I2C_RXTX_LEN	128	/* maximum tx/rx buffer length */

#ifdef	CONFIG_I2C_MULTI_BUS
#define	MAX_I2C_BUS			2
#define	I2C_MULTI_BUS			1
#else
#define	MAX_I2C_BUS			1
#define	I2C_MULTI_BUS			0
#endif

#if !defined(CONFIG_SYS_MAX_I2C_BUS)
#define CONFIG_SYS_MAX_I2C_BUS		MAX_I2C_BUS
#endif

/* define the I2C bus number for RTC and DTT if not already done */
#if !defined(CONFIG_SYS_RTC_BUS_NUM)
#define CONFIG_SYS_RTC_BUS_NUM		0
#endif
#if !defined(CONFIG_SYS_DTT_BUS_NUM)
#define CONFIG_SYS_DTT_BUS_NUM		0
#endif
#if !defined(CONFIG_SYS_SPD_BUS_NUM)
#define CONFIG_SYS_SPD_BUS_NUM		0
#endif

#ifndef I2C_SOFT_DECLARATIONS
# if defined(CONFIG_MPC8260)
#  define I2C_SOFT_DECLARATIONS volatile ioport_t *iop = ioport_addr((immap_t *)CONFIG_SYS_IMMR, I2C_PORT);
# elif defined(CONFIG_8xx)
#  define I2C_SOFT_DECLARATIONS	volatile immap_t *immr = (immap_t *)CONFIG_SYS_IMMR;

# elif (defined(CONFIG_AT91RM9200) || \
	defined(CONFIG_AT91SAM9260) ||  defined(CONFIG_AT91SAM9261) || \
	defined(CONFIG_AT91SAM9263)) && !defined(CONFIG_AT91_LEGACY)
#  define I2C_SOFT_DECLARATIONS	at91_pio_t *pio	= (at91_pio_t *) ATMEL_BASE_PIOA;
# else
#  define I2C_SOFT_DECLARATIONS
# endif
#endif

#ifdef CONFIG_8xx
/* Set default value for the I2C bus speed on 8xx. In the
 * future, we'll define these in all 8xx board config files.
 */
#ifndef	CONFIG_SYS_I2C_SPEED
#define	CONFIG_SYS_I2C_SPEED	50000
#endif
#endif

/*
 * Many boards/controllers/drivers don't support an I2C slave interface so
 * provide a default slave address for them for use in common code.  A real
 * value for CONFIG_SYS_I2C_SLAVE should be defined for any board which does
 * support a slave interface.
 */
#ifndef	CONFIG_SYS_I2C_SLAVE
#define	CONFIG_SYS_I2C_SLAVE	0xfe
#endif

/*
 * Initialization, must be called once on start up, may be called
 * repeatedly to change the speed and slave addresses.
 */
void i2c_init(int speed, int slaveaddr);
void i2c_init_board(void);
#ifdef CONFIG_SYS_I2C_BOARD_LATE_INIT
void i2c_board_late_init(void);
#endif

#if defined(CONFIG_I2C_MUX)

typedef struct _mux {
	uchar	chip;
	uchar	channel;
	char	*name;
	struct _mux	*next;
} I2C_MUX;

typedef struct _mux_device {
	int	busid;
	I2C_MUX	*mux;	/* List of muxes, to reach the device */
	struct _mux_device	*next;
} I2C_MUX_DEVICE;

I2C_MUX_DEVICE	*i2c_mux_search_device(int id);
I2C_MUX_DEVICE *i2c_mux_ident_muxstring (uchar *buf);
int i2x_mux_select_mux(int bus);
int i2c_mux_ident_muxstring_f (uchar *buf);
#endif

/*
 * Probe the given I2C chip address.  Returns 0 if a chip responded,
 * not 0 on failure.
 */
int i2c_probe(uchar chip);

/*
 * Read/Write interface:
 *   chip:    I2C chip address, range 0..127
 *   addr:    Memory (register) address within the chip
 *   alen:    Number of bytes to use for addr (typically 1, 2 for larger
 *              memories, 0 for register type devices with only one
 *              register)
 *   buffer:  Where to read/write the data
 *   len:     How many bytes to read/write
 *
 *   Returns: 0 on success, not 0 on failure
 */
int i2c_read(uchar chip, uint addr, int alen, uchar *buffer, int len);
int i2c_write(uchar chip, uint addr, int alen, uchar *buffer, int len);

/*
 * Utility routines to read/write registers.
 */
static inline u8 i2c_reg_read(u8 addr, u8 reg)
{
	u8 buf;

#ifdef CONFIG_8xx
	/* MPC8xx needs this.  Maybe one day we can get rid of it. */
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif

#ifdef DEBUG
	printf("%s: addr=0x%02x, reg=0x%02x\n", __func__, addr, reg);
#endif

	i2c_read(addr, reg, 1, &buf, 1);

	return buf;
}

static inline void i2c_reg_write(u8 addr, u8 reg, u8 val)
{
#ifdef CONFIG_8xx
	/* MPC8xx needs this.  Maybe one day we can get rid of it. */
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif

#ifdef DEBUG
	printf("%s: addr=0x%02x, reg=0x%02x, val=0x%02x\n",
	       __func__, addr, reg, val);
#endif

	i2c_write(addr, reg, 1, &val, 1);
}

/*
 * Functions for setting the current I2C bus and its speed
 */

/*
 * i2c_set_bus_num:
 *
 *  Change the active I2C bus.  Subsequent read/write calls will
 *  go to this one.
 *
 *	bus - bus index, zero based
 *
 *	Returns: 0 on success, not 0 on failure
 *
 */
int i2c_set_bus_num(unsigned int bus);

/*
 * i2c_get_bus_num:
 *
 *  Returns index of currently active I2C bus.  Zero-based.
 */

unsigned int i2c_get_bus_num(void);

/*
 * i2c_set_bus_speed:
 *
 *  Change the speed of the active I2C bus
 *
 *	speed - bus speed in Hz
 *
 *	Returns: 0 on success, not 0 on failure
 *
 */
int i2c_set_bus_speed(unsigned int);

/*
 * i2c_get_bus_speed:
 *
 *  Returns speed of currently active I2C bus in Hz
 */

unsigned int i2c_get_bus_speed(void);

/* NOTE: These two functions MUST be always_inline to avoid code growth! */
static inline unsigned int I2C_GET_BUS(void) __attribute__((always_inline));
static inline unsigned int I2C_GET_BUS(void)
{
	return I2C_MULTI_BUS ? i2c_get_bus_num() : 0;
}

static inline void I2C_SET_BUS(unsigned int bus) __attribute__((always_inline));
static inline void I2C_SET_BUS(unsigned int bus)
{
	if (I2C_MULTI_BUS)
		i2c_set_bus_num(bus);
}

#endif	/* _I2C_H_ */

#endif //0


}//extern "C" 


///http://www.emdebian.org/~zumbi/mx53/u-boot-fsl/drivers/i2c/
//http://www.emdebian.org/~zumbi/mx53/u-boot-fsl/drivers/i2c/pca9564_i2c.c

//http://ngw100-guyvo.blogspot.com/2008/04/hell-of-pca9698.html