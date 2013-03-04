/* HalExpansionPort.h HalExpansionPort library
 * Copyright (c) 2012 Neil Hancock
 *  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * Class HalExpansionPort to manage scheduling of multipile leds.

 * 
 * To set up the device
 * - configure ports -
 *    Output and logic level
 *    Input and interrupt
 *  There are a nymber of owners of the ports
 * 
 * */
#ifndef HalExpansionPort_h
#define HalExpansionPort_h

typedef enum {
  //Interrupt inputs 
  ehnExpInt1, //D00
  ehnExpInt2, //D01
  ehnCrdSwSt, //D03 Low going
  ehnExtAwakeSwSt, //DO5
} eHalPortInterrupts;

typedef enum {
	//Ports offset into the PCA9698
  ehpCmd=0,
  ehpPort0=0,
  ehpPort1=1,
  ehpPort2=2,
  ehpPort3=3,
  ehpPort4=4,
  ehpPortNumber=5,
  ehpPortBuffSize=ehpPortNumber,
  } eHalPhysicalPorts;
typedef enum {
	eplbPwrAnlg1En
	}ePcaLedBits;
typedef enum {
	epb1PwrAnlg1En_m=0x01,
	epb1PwrAnlg2En_m=0x02,
	epb1PwrAnlg3En_m=0x04,
	epb1PwrExtPoeEn_m=0x08,
	epb1UsbrEn_m=0x10,
	epb1UsbrC0_m=0x20,
	epb1UsbrC1_m=0x40,
	epb1UsbrC2_m=0x80	
	}ePcaBank1;
typedef enum {
	epb2PwrUsbFm1En_m=0x01,
	epb2PwrUsbm5En_m=0x02,
	epb2PwrUsb4En_m=0x04,
	epb2PwrExtRsEn_m=0x08,
	epb2EnPwrLed_m=0x10,
	epb2LedPwrGrn_m=0x20,
	epb2Usb2Grn_m=0x40,
	epb2Usb2Red_m=0x80
}ePcaBank2;
typedef enum {
	epbtExpInt1=0x00,
	epbtExpInt2=0x01,
	//not used
	epbtCrdSwSt  =0x03,
	epbtPwrUsb6En=0x04,
    epbtExtAwakeSwStN=0x05,
	epbtRs232InEnN   =0x06,
	epbtRs232FoffEnN =0x07,
	//Bank1
	epbtPwrAnlg1En= 0x08,
	epbtPwrAnlg2En= 0x09,
	epbtPwrAnlg3En= 0x0a,
	epbtPwrExtPoeEn=0x0b,
	epbtUsbrEn=     0x0c,
	//epbtUsbrC0=0x0d,
	//epbtUsbrC1=0x0e,
	//epbtUsbrC2=0x0f
	
	//Bank2
   epbtPwrUsbFm1En=0x10,
   epbtPwrUsb5En,
   epbtPwrUsb4En,
   epbtPwrExtRsEn,
   epbtEnPwrLed,
   epbtLedPwrGrn,
   epbtUsb2grn,
   epbtUsb2red,	
	//Bank3
	
	//Bank4	
		epbtTooFar=0x48,
}ePca9698Bits;
typedef enum {
	//PCA9698 commands - 1st byte written
	epcaCmdWriteOutPorts    =0x08, //base 5 registers
	epcaCmdWritePolarityReg =0x10,//base 5 registers - invert input
	epcaCmdWriteConfigReg   =0x18, //base 5 registers - direction
	epcaCmdWriteMaskInterrupt =0x20, //base 5 registers - interrupt enables
	epcaCmdWriteOutConf =0x28, //1 regsiter
	epcaCmdWriteAllbnk =0x29, //1 register
	epcaCmdWriteMode =0x2a,
	epcaCmdAutoIncrBa =0x80 // Auto incre bank - 8bytes
} ePcaCmds;

#define cPca9698cmd_AI_bit 0x80
#define cPca9698cmd_IpReg   (0x00|cPca9698cmd_AI_bit)
#define cPca9698cmd_OpReg   (0x08|cPca9698cmd_AI_bit)
#define cPca9698cmd_CfgReg  (0x18|cPca9698cmd_AI_bit)
#define cPac9698cmd_MaskInt (0x20|cPca9698cmd_AI_bit)

//PCA9698 Port Expander on TWI1
#define HW_PCA9698_TwiAddr    0x20        //!< Pca9698 TWI slave bus address - raw not shifted
#define HW_PCA9698_InternalAddrLen 0x01  //This is of the command register
#define HW_ID_TWI_PCA9698    ID_TWI1
#define HW_ADDR_TWI_PCA9698  TWI1 // TWI Base Address in SAM3X memory map for
#define HW_TWI1_SPEED 400000

class HalExpansionPortClass  {
  public:
    HalExpansionPortClass(void);
    void portInit(void); //Initialization
    void portInterrupt(void); // Interrupt on inputs
	result_t portHwRefresh(void);
	result_t initilizePcaPorts(void);
    void portSet(eHalPhysicalPorts portNumber,uint8_t bitmask);//Activate according to bit mask
    void portClr(eHalPhysicalPorts portNumber,uint8_t bitmask);//Clear according to bit mask
    void portToggle(eHalPhysicalPorts portNumber,uint8_t bitmask);
    void ledsSet(uint32_t bitmask);//Activate according to bit mask
	void ledsClr(uint32_t bitmask);//clear according to bit mask
    void ledsToggle(uint32_t bitmask);//Activate according to bit mask
	void bit_Set(ePca9698Bits gpio );
	void bitClr(ePca9698Bits gpio );
    void bitToggle(ePca9698Bits gpio );
	void goLowPowerMode();
    void ledTestStart();
	void ledTestStop();
	void timerTick();
//    uint8_t portread(uint8_t portNumber,uint8_t register);
//    portManage();
//	bool reqUpdatePort(ports);//System Input
  //register for interrupts

  private:
  	//result_t TwiWrite(uint32_t iaddress,uint8_t length,	uint8_t *pBuffer);
  result_t writePcaPorts(uint32_t iRegister,uint8_t *pBuffer,uint32_t number);
      void ledBitWalk();
    uint8_t portStateOutput[ehpPortBuffSize];
    uint8_t portStateInput[ehpPortBuffSize]; //First space reserved
    uint32_t ledBitWalk_bit;
	bool ledTest_bool; //True if test in progress
};
#ifndef AZ23_PRJ_FILE
extern HalExpansionPortClass HalExpansionPort; 
#endif //AZ23_PRJ_FILE
#endif//HalExpansionPort_h