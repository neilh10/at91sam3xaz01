/*									tab:4
 *
 * Date last modified:  $Id: HPLTWI.h,v 1.1 2003/10/31 22:38:27 idgay Exp $
 */

// define TWI device status codes. 


enum { //Twi Addr
/* Analog A B007a board 
 */
//#define cTwiAddrAdg728A 0x98
// ADG 729 has 4 addrs 1000 1xx0-
//  0x88(U7) 0x8A(U9) 0x8C 0x8E(U7)

//U8b SxA-DA -data i/p mux
//    Any of S1x short IA i/p for calibration
//U8c SxB-DB -vref select
//    S1B=1.75V S2B=1.03V S3B=0.88V S4B=0.15V 
cTwiAddrAdg729A=0x8E,

//U9b SxA-DA IA to AdcIn+
//    S1A=IA S2A=GND S3A=GND S4A=IA_VRefb 
//U9c SxB-DB Thermistor Frontx to AdcIn+
//    S1B=TF1 S2B=TF2 S3B=TF3 S4B=ThermLoc
cTwiAddrAdg729B=0x8A,

//U7b SxA-DA Thermistor Top to AdcIn+
//    S1A=TT3 S2A=TT2 S3A=TT1 S4A=ThermistorAir 
//U7c SxB-DB AdcIn-
//    S1B=IA_Vrefb S2B=IA_Vref S3B=IA_out(same as B-S1) S4B=Gnd
cTwiAddrAdg729C=0x88,

cTwiAddrMcp3421A=0xD0,
cTwiAddrCAT24C04=0xA0,
cTwiAddrGeneralCall=0x00,
cTwiCmdGeneralReset=0x06,

//proto 2 addrs **** remove
//  twiAddrPcf8574_0=0x40, //I/O bit expander
//  twiAddrAd1561_0=0x58,  //variable resistor proto1
  twiAddrAdg728_0=0x98,  // 8-1 mux U1 'vo-'
  twiAddrAdg728_1=0x9A,  // 8-1 mux U2 'vo+'
  twiAddrAdg728_2=0x9C,  // 8-1 mux U80 ADC_MUX i/p
  twiAddrAdg729_0=0x88,  // 2x4-1 mux U5 p2 A(lsb)=Gain B(msb)=Vref
  twiAddrAdg729_1=0x8A,  // 2x4-1 btwn mux U8 p2. A(lsb)='+ve' B(msb)='-ve' of IA
  twiAddrEnd
} eTwiAddr;

/* Muxing Setup - See diagrams
 * For IA o/op
 *  ADG729C-B Select IA_VrefB to AdcIn-
 *  ADG729A-B Select IA reference - typically 0.15V 
 *  ADG729B-A Select IA_UT to AdcIn+
 * For IA "zero"
 *  ADG729C-B Select IA_OUT
 *  ADG729A-B Select IA reference - typically 0.15V
 *  ADG729A-A select CAL
 *  ADG729B-A select IA_OUT
 *  
 * For Temperature
 *  ADG729C-B Select GND
 *  ADG729C-A or ADG729B-B select one of temperature
 *  
 */
typedef enum { //ADG729A U8- B010r1
  //Calibration - typically don't select
  A_S1A_CALPRE=0x01, //Pre connect - cap across i/p
  A_S1A_CALALL=0x0F, //Connect all
  //Select for on of 4 different VRef Voltages
  A_S1B_VR_1V75=0x10,
  A_S2B_VR_1V03=0x20,
  A_S3B_VR_0V88=0x40,
  A_S4B_VR_0V15=0x80,
//  A_STD_REF=A_S1B_VR_1V75,
  A_STD_REF=0,
  A_zero=0
} eADG729A;

typedef enum { //ADG729B U9- B010r1
  //Input to +ve of Adc or AdcIn+
  //Select for on of following or none
  B_S1A_IA_OUT=0x01,
  B_S2A_GND=0x02,
  //B_S3A_GND=0x04,
  B_S4A_VrefB= 0x08,
  //Input to AdcIn+ one of following or none
  B_S1B_ThrmFrnt1=0x10,
  B_S2B_ThrmFrnt2=0x20,
  B_S3B_ThrmFrnt3=0x40,
  B_S4B_ThrmLoc=  0x80,
  B_zero=0
} eADG729B;

typedef enum { //ADG729C U7 - B010r1
  //Input to AdcIn+ select one of following or none
  C_S1A_ThrmTop1=0x01,
  C_S2A_ThrmTop2=0x02,
  C_S3A_ThrmTop3=0x04,
  C_S4A_ThrmAir=0x08,
  C_Amsk=0x0F,
  //Input to -ve or AdcIn- select one of following or none
  C_S1B_VrefB= 0x10,//Std Choice
  C_S2B_VrefU= 0x20,
  C_S3B_IA_OUT=0x40, //ADC Zero choice
  C_S4B_GND=   0x80,
  //Feed AGND to RefBuffer and AdcIn-
  //The following requires MUX A-B to be all open
  C_S24B_GND=(C_S2B_VrefU|C_S4B_GND), //GND to i/p of buffer
  C_zero
} eADG729C;

typedef struct sHalTwiRec {
    uint8_t opcode; //Simple action - one of eHALTWI_OPCODE
    uint8_t data; //to be written or stored
    } tHalTwiRec;

typedef struct {
    int8_t step; //Not for caller. Place in the chain.
  //  char addr; // I2C addr to be read/written
   int8_t number; //Number of active records including address
   //int8_t owner; //Future? HALTWI_OWNER
//    uint16_t activity; //bit definition W=0,R=1. Type of activity for bytes 
    } tHalTwiHeader;

#define TWI_16_RECS 32
typedef struct  {
    tHalTwiHeader h;
    tHalTwiRec rec[TWI_16_RECS]; //typically 8 or 16

    } tTwiFrame16;

 enum {
    HALTWI_ACT_NULL  = 0,
    HALTWI_ACT_ADDR  = 1,
//    HALTWI_ACT_RDADDR  = 1,
    HALTWI_ACT_WRITE = 2,
    HALTWI_ACT_READ  = 3,
    HALTWI_ACT_DELAY_10MS =4, //Introduce a delay of 'x' MS before next TWI action
    HALTWI_ACT_END
    } eHALTWI_ACT_OPCODE;

#define cHtb_maxSize 6
//#define cHtb_EepromSize  0x10
//#define cHtb_bufInit 0x55
typedef struct
{
	uint8_t slave_adr;				//Slave address and W/R byte
	uint8_t size;						//Number of bytes to send or 
											//receive 	
//  unsigned char *data_ptr;				//Pointer to the bytes to send
  uint8_t octet[cHtb_maxSize];				//Pointer to the bytes to send
  
}Hil_twiBuf_t;		//tx_type							//or pointerto RX buffer 
 
 //end
