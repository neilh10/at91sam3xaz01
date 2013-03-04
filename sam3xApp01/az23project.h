/* az23project.h
  Copyright (c) 2012 Azonde.  All right reserved.

Arduino Mapping 1.5
R/TXD   UART  Serial
R/TXD0  USART0 Serial2
R/TXD1  USART1 Serial3
R/TXD2 (arduino PIO_PB20X1_AD13  PIO_PB21B_SPI0_NPCS2 Az23: ExpSlots Serial
R/TXD3  [Arduino:USART2?] Serial4

arduino1.5: Possible dual allocation with PinArd52 & 86
*/

#ifndef az23project_h
#define az23project_h
//#include <stdint.h>

//From sam/boards/sam3x_ek
/*! Board oscillator settings */
#define BOARD_FREQ_SLCK_XTAL            (32768U)
#define BOARD_FREQ_SLCK_BYPASS          (32768U)
#define BOARD_FREQ_MAINCK_XTAL          (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS        (12000000U)

/*! Master clock frequency */
#define BOARD_MCK                       CHIP_FREQ_CPU_MAX

/** board main clock xtal statup time */
#define BOARD_OSC_STARTUP_US   15625


 typedef enum {
ArdVpm00,ArdVpm01,ArdVpm02,ArdVpm03,ArdVpm04,ArdVpm05,ArdVpm06,ArdVpm07,ArdVpm08,ArdVpm09,
ArdVpm10,ArdVpm11,ArdVpm12,ArdVpm13,ArdVpm14,ArdVpm15,ArdVpm16,ArdVpm17,ArdVpm18,ArdVpm19,
ArdVpm20,ArdVpm21,ArdVpm22,ArdVpm23,ArdVpm24,ArdVpm25,ArdVpm26,ArdVpm27,ArdVpm28,ArdVpm29,
ArdVpm30,ArdVpm31,ArdVpm32,ArdVpm33,ArdVpm34,ArdVpm35,ArdVpm36,ArdVpm37,ArdVpm38,ArdVpm39,
ArdVpm40,ArdVpm41,ArdVpm42,ArdVpm43,ArdVpm44,ArdVpm45,ArdVpm46,ArdVpm47,ArdVpm48,ArdVpm49,
ArdVpm50,ArdVpm51,ArdVpm52,ArdVpm53,ArdVpm54,ArdVpm55,ArdVpm56,ArdVpm57,ArdVpm58,ArdVpm59,
ArdVpm60,ArdVpm61,ArdVpm62,ArdVpm63,ArdVpm64,ArdVpm65,ArdVpm66,ArdVpm67,ArdVpm68,ArdVpm69,
ArdVpm70,ArdVpm71,ArdVpm72,ArdVpm73,ArdVpm74,ArdVpm75,ArdVpm76,ArdVpm77,ArdVpm78,ArdVpm79,
ArdVpm80,ArdVpm81,ArdVpm82,ArdVpm83,ArdVpm84,ArdVpm85,ArdVpm86,ArdVpm87,//ArdVpm88,ArdVpm89,
//Total so far 88 items
ArdVpmEnd,
AzVpm01=ArdVpmEnd,
AzVpm02,AzVpm03,AzVpm04,AzVpm05,AzVpm06,AzVpm07,AzVpm08,AzVpm09,
AzVpmEnd //Total97 =88+9 - update variants\arduino_due_x\variant.h:PINS_COUNT
} enumArdVirtualPinMap;

/* use ePca9698Bits
typedef  enum {//Twi Based U8 HwExpansion - reference into table azHePinDescriptions
    //D00-07
    HeExpInt1, 
    HeExpInt2,
    HeNotUsed,
    HeCrdSwSt,
    HePwrUsb6En, //Output
    HeExtAwakeSwSt_,
    HeRs232InEn_,//Output
    HeRs232FoffEn_,//Output
    //D10-D17
    HePwrAnlg1En,//Output
    HePwrAnlg2En,//Output
    HePwrAnlg3En,//Output
    HePwrExpPoeEn,

    HeUsbrEn,
    HeUsbrC0,
    HeUsbrC1,
    HeUsbrC2,
    
    //D20-D27
    HePwrUsbFm1En,
    HePwrUsb5En,
    HePwrUsb4En,
    HePwrExtRsEn,
    HeEnPwrLed,
    HeLedPwrGrn,
    HeUsb2grn,
    HeUsb2red,
    
    //D30-37
    HeLedSsRed,
    HeLedssGrn,
    HeLedTxGrn,
    HeLedRxRed,

    HeLedUsb3red,
    HeLedUsb3grn,
    HeLedUsbOtgRed,
    HeLedUsbOtgGren,

    //D40-47
    HePwrUsb7En,
    HeLedBlueSync,
    HeLedBatBred,
    HeLedBatBgrn,
    
    HeLedBatAred,
    HeLedBatAGrn,
    HeExtRelay1En,
    HeExtRelay2En,
        HeEn
        
} enumHardwareExpansionPorts ;//Az23 use ePca9698Bits*/
  
#ifdef __cplusplus
extern "C"{
#endif // __cplusplus


//#include "wiring_constants.h"
// The Az23 port mappings

//NotUsedPIO_Pa0 ArdVpm69
#define HwRS232InvldSt_ ArdVpm68//PIO_PA1/WKUP0 allocated Az23 to PIO_PA1 from PIO_PA1A_CANRX0
#define HwExtAnlg1a     ArdVpm61 //PIO_PA2
#define HwExtAnlg2a     ArdVpm60 //PIO_Pa3
#define HwArdS15A5PA4   ArdVpm59 //PIO_Pa4
#define HwExtTouchSt_   AzVpm01  //PIO_Pa5
#define HwArdS14A4PA6   ArdVpm58 //PIO_Pa6
#define HwPrRadioSt1    ArdVpm31 //PIO_Pa7
   //ProcDinRxd   ArdVpm00 PIO_PA8A_URXD UART Arduino Serial
   //ProcDoutTxd  ArdVpm01 PIO_PA9A_UTXD UART Arduino Serial
   //ArdS20Rx0Pa10 ArdVpm19 USART0 Serial2
   //ArdS21Tx0Pa11 ArdVpm18 USART0 Serial2
#define PrRadioToProcDout ArdVpm16 //PIO_PA12A_RXD1 Arduino Serial3
#define PrRadioFmProcDin  ArdVpm17 //PIO_PA13A_TXD1 Arduino Serial3
#define HwPrRadioToRts    ArdVpm23 //PIO_PA14
#define HwPrRadioFmCts    ArdVpm24 //PIO_PA15
#define HwArdS10A0Pa16    ArdVpm54 //PIO_PA16 
  //SDAA ArdVpm70 PIO_PA17A_TWD0
  //SCLA ArdVpm71 PIO_PA18A_TWCK0
#define HwPwrMtEn      ArdVpm42 //PIO_PA19
#define HwPrRadioToDtr ArdVpm43 //PIO_PA20
    //NotUsed_PIO_Pa21 ArdVpm73
#define HwArdS13A3Pa22 ArdVpm57 //PIO_PA22
#define HwArdS12A2Pa23 ArdVpm56 //PIO_PA23
#define HwArdS11A1Pa24 ArdVpm55 //PIO_PA24
//Spi0_MISO  ArdVpm74 PIO_PA25A_SPIO_MISO
//Spi0_MOSI  ArdVpm75 PIO_PA26A_SPIO_MOSI
//Spi0_SCLK  ArdVpm76 PIO_PA27A_SPIO_SPCK
//Spi0_A0Pa27 ArdVpm77 PIO_PA28A_SPIO_NPCS0
//Spi0_A1Pa29 ArdVpm87 PIO_PA29A_SPIO_NPCS1


//PIO_PBxx
//NotUsedPb0
//NotUsedPb1
//NotUsedPb2
//NotUsedPb3
//NotUsedPb4
//NotUsedPb5
//NotUsedPb6
//NotUsedPb7
//NotUsedPb8
//NotUsedPb9
#define HwPwr5VEn AzVpm02  //PB10 part of all ArdVpm85
#define HwUsbmID  AzVpm03  //PB11 part of all ArdVpm85
#define HwSdaLTwi1 ArdVpm20//SdaL  //PIO_PB12A_TWD1        ArdVpm20 TWI1 
#define HwSclLTwi1 ArdVpm21 //SclL  //PIO_PB13A_TWCK1       ArdVpm21 TWI1
//NotUsedPIO_PB14_ExpCtsProcIn  ArdVpm53
#define HwExpDac0  ArdVpm66 //PIO_PB15
#define HwExtIp2a  ArdVpm67 //PIO_PB16 reallocated Az23 to PIO_PB16A_TCLK5 from PIO_PB16X1_DAC1
#define HwThermAdcB10Pb17 ArdVpm62  //PIO_PB17
#define HwRbatAdcB11Pb18  ArdVpm63 //PIO_PB18
#define HwMuxAdcB12Pb19   ArdVpm64 //PIO_PB19
     //ExpFmProcDin  ArdVpm65 PIO_PB20 UsedAz23 as Serial5?
     //ExpToProcDout ArdVpm52/PIO_PB21 ArdVpm86/PIO_PB21B_SPIO_NPCS2 UsedAz23 as Serial5?
#define HwExpRtsCsProcOut AzVpm04 //PIO_PB22 
#define HwSpioNPCS3       ArdVpm78 //SPI0_A2Pb23  PIO_PB23B_SPI0_NPCS3
#define HwExpClkProcOut   AzVpm05 //PIO_PB24
#define HwArdS22Pb25 ArdVpm02 //PIO_PB25B_TIOA0
#define HwExtIp1a    ArdVpm22 //PIO_Pb26 
#define HwExtAnlg3a  ArdVpm13 //PIOPb27  Az23 was Arduino PIN_LED_13 Amber
//PB28-32 JTAG

//PIO_PCxx
   //NotUsedPc0_Erase
#define HwLedCpuRed_ ArdVpm33 //PIO_PC1 UsedAz23
#define HwLedCpuGrn_ ArdVpm34 //PIO_PC2
#define HwArdS213Pc3 ArdVpm35 //PIO_PC3
#define HwExpwiEn   ArdVpm36 //PIO_PC4
#define HwVbstEn     ArdVpm37 //PIO_PC5
#define HwVbst12VEn  ArdVpm38  //PIO_PC6
#define HwPwr3v3En   ArdVpm39 //PIO_PC7
   //NotUsedPIO_PC8  ArdVpm40
#define HwExpPwrOn   ArdVpm41 //PIO_PC9
#define HwExpEn1     AzVpm06//PIO_PC10
#define HwExpEn2     AzVpm07//PIO_PC11
   //NotUsedPIO_PC12 ArdVpm51
#define HwRbatTestEn ArdVpm50  //PIO_PC13
#define HwPrRadioEn  ArdVpm49 //PIO_PC14
#define HwSdiOutEn   ArdVpm48 //PIO_PC15
#define HwSdiInEn    ArdVpm47 //PIO_PC16
#define HwPwrIn30VswShdn ArdVpm46 //PIO_PC17
#define HwPwrIn30VLdoEnN  ArdVpm45 //PIO_PC18
#define HwPwrIn6VaEnN     ArdVpm44 //PIO_PC19
#define HwPwrIn6VbEnN AzVpm08 //PIO_PC20
#define HwArdS29Pc21 ArdVpm09 //PIO_PC24B_PWML4 - ArduinoShield
#define HwArdS28Pc22 ArdVpm08 //PIO_PC23B_PWML5 - ArduinoShield
#define HwArdS27Pc23 ArdVpm07 //PIO_PC24B_PWML6 - ArduinoShield
#define HwArdS26Pc24 ArdVpm06 //PIO_PC24B_PWML7 - ArduinoShield
#define HwArdS25Pc25 ArdVpm05 //PIO_PC25B_TIOA6 - ArduinoSheild
#define HwWire1OutEn ArdVpm04 //PIO_PC26B_TIOB6?? -UsedAsAz23
#define HwExtIp3a    AzVpm09  //PIO_PC27
#define HwArdS23Pc28 ArdVpm03 //PIO_PCS28B_TIOA7
#define HwWire1InEn  ArdVpm10 //PIO_PC29B_TIOB7 - ??
    //HwNotUsedPc30  ArdVpm72 //Ax23 used PIO_PC30 from LEDs 

//PIO_PDxx
#define HwAnlgMuxA0  ArdVpm25 //PIO_PD0
#define HwAnlgMuxA1  ArdVpm26 //PIO_PD1
#define HwAnlgMuxA2  ArdVpm27 //PIO_PD2
#define HwAnlgMuxEn_ ArdVpm28 //PIO_PD3
      //ProcDoutTx3  ArdVpm14 //PIO_PD4B_TXD3 USART2 Arduino Serial4
      //ProcDinRx3   ArdVpm15 //PIO_PD5B_RXD3 USART2 Arduino Serial4 
#define HwRs422OutEn ArdVpm29 //PIO_PD6
#define HwArdS211Pd7 ArdVpm11 PIO_PD7B_TIOA8 - ArduinoShield
#define HwArdS212Pd8 ArdVpm12 PIO_PD8B_TIOB8 - ArduinoShield
#define HwExtIp4a    ArdVpm30 //PIO_PD9
#define HwRs422Shdn  ArdVpm32 //PIO_PD10

//extern void setup( void ) ;
//extern void loop( void ) ;

#define initVirtualPin(VirtualPin) \
	PIO_Configure( \
	g_APinDescription[VirtualPin].pPort,\
	g_APinDescription[VirtualPin].ulPinType,\
	g_APinDescription[VirtualPin].ulPin,\
	g_APinDescription[VirtualPin].ulPinConfiguration)\

#define disableVirtualPin(VirtualPin) \
PIO_SetInput( \
g_APinDescription[VirtualPin].pPort,\
g_APinDescription[VirtualPin].ulPin,\
g_APinDescription[VirtualPin].ulPinConfiguration)


//#define digitalPinToPort(P)        ( g_APinDescription[P]->pPort )

	//ETCChannel ulTCChannel ;
typedef enum {
	//OutputStd,OutputInv, InputSt, InputStIntN,InputStIn
	eHeptNull
	} eHwExpPioType;
/* Hardware Expansion Port U8 */
typedef struct _eHwExpPinDescription
{
	uint8_t Port ;
	uint8_t Pin ;
	//uint32_t ulPeripheralId ;
	eHwExpPioType PinType ;
	//uint32_t ulPinConfiguration ;
	//uint32_t ulPinAttribute ;
	//EAnalogChannel ulAnalogChannel ; /* Analog pin in the Arduino context (label on the board) */
	//EAnalogChannel ulADCChannelNumber ; /* ADC Channel number in the SAM device */
	//EPWMChannel ulPWMChannel ;
} eHwExpPinDescription ;

void HilInitAzPorts(void);
void HilInitPeripherals(void);
#ifdef __cplusplus
} // extern "C"


//#include "HardwareSerial.h"
//#include "wiring_pulse.h"

#endif // __cplusplus

 /***********************************
Maintenance section

  **/
typedef enum { //result_t - njh was the other way round, check this works
	SUCCESS = 0,
	FAIL = 1,
 //SUCCESS_DBG_NO_PROMPT=0x10
    } eResult;
//typedef uint8_t result_t;
//uint8_t rcombine(uint8_t r1, uint8_t r2); // keep 1.1alpha1 happy
//typedef uint8_t result_t __attribute__((combine(rcombine)));
typedef eResult result_t;
typedef enum {
    MNT_HW_FAIL=1, // Hard failure
    MNT_HW_RECOVERED=2, //Suspected Hardware failure, recovered
    MNT_SW_FAIL,        // Software fail, not recoverable
    MNT_SW_RECOVERED,  //Software fail recovered
    MNT_REPORTS_END
    }eMaintenanceReport;

typedef enum {
    /* Device ID enumerations. Should be kept sequential and not too long as used to look up an
    array of values
    */
    DEV_TWI_DEVICE_NULL=0, //No device, left for debugging purposes
    DEV_TWI_GENERAL, //All of the TWI devices. Use debug to further isolate
    DEV_TWI_ADG728_0, 
    DEV_TWI_ADG728_1, 
    DEV_TWI_ADG729_0,
    DEV_TWI_ADG729_1,
   DEV_ADC1,
   DEV_ENUMERATION_END
    }eDeviceEnumeration;
typedef enum {
    /* Subsytem ID enumerations. Should be kept sequential and not too long as used to look up an
    array of values     */
  SUBS_ERR_ENUM_ERROR,//0 Error calling this interface
  SUBS_ERR_ENUM_CNT,  //1 number of errors counted
  SUBS_MSK_ENUM_SENSORSCALIBRATE,//2
  SUBS_MSK_ENUM_HPLADC,          //3
  SUBS_MSK_ENUM_SENSORSMNG,      //4
  SUBS_MSK_ENUM_SENSORSMNGCOND,  //5
  SUBS_MSK_ENUM_DATAMNG,  //6
  SUBS_MSK_ENUM_DATASTOREMNG,//7
  SUBS_MSK_ENUM_SENSORSADCHW,    //8
  SUBS_MSK_ENUM_SENSORSADCHWMSB, //9
  SUBS_ENUMERATION_END
}eSubsEnumeration;

// Include board variant
//Set of unions to access uint8_t in uint32_t
typedef struct {
    uint8_t l; //LSB
    uint8_t m; //MSB
    } tTwoByte; 
typedef struct {
    uint8_t b1; //LSB
    uint8_t b2; //
    uint8_t b3; //
    uint8_t b4; //MSB
    } tFourUint8;
typedef struct {
    uint8_t b1; //LSB
    uint8_t b2; //
    uint8_t b3; //
    uint8_t b4; //
    uint8_t b5; //
    uint8_t b6; //
    uint8_t b7; //
    uint8_t b8; //MSB
    } tEightUint8;
typedef union {
    tTwoByte b; //byte
    uint16_t w; //Word       
  } uByteWrd; 
typedef struct {
    uByteWrd l; //LSB
    uByteWrd m; //MSB
    } tTwoUint16;

typedef union {
//tFourByte b; //byte
    tTwoUint16 w; //word
    uint32_t l;   //long
    } uByteWrdLng;
typedef struct {
    uByteWrdLng l; //LSB
    uByteWrdLng m; //MSB
    }__attribute__ ((packed)) tTwoUint32;

//Signed unions
typedef struct {
    int8_t l; //LSB
    int8_t m; //MSB
} __attribute__ ((packed)) tTwoInt8;

typedef union {
    tTwoInt8 b; //byte
    int16_t w; //Word       
  } __attribute__ ((packed)) uInt8Int16;

typedef struct {
    uInt8Int16 l; //LSB
    uInt8Int16 m; //MSB
} __attribute__ ((packed)) tTwoInt16;
typedef union {
//tFourByte b; //byte
    tTwoInt16 w; //word
    int32_t l;   //long
    } __attribute__ ((packed)) uInt32Int16;

//Set of unions to access uint16_t in uint64_t/tos_time
typedef struct {
  uint16_t l; //LSB
  uint16_t m; //MSB
} __attribute__ ((packed)) w32_t;

typedef union {
  tFourUint8 u8; //
  uint32_t w32;   //long
} __attribute__ ((packed)) w32_4u8_u;
typedef union {
  w32_t two_u16; //word
  uint32_t w32;   //long
} __attribute__ ((packed)) w32Two_u16_u;
typedef struct {
  w32Two_u16_u  l; //u32 LSB
  w32Two_u16_u  m; //u32 MSB
} __attribute__ ((packed)) w64_t;

typedef union {
  uint8_t     b[8];
  tEightUint8 u8; //
  uint64_t w64;   //long
} __attribute__ ((packed)) w64_8u8_u;
typedef union {
  w64_t   two_u32;   //two u32
  uint64_t w64;  //long
} __attribute__ ((packed)) w64Two_u32_u;

//End unions
typedef struct tEvtRec {
    uByteWrd rec;
} tEvtRec;
#define EVT_LOG_SIZE 12
typedef struct tEvtLog {
  tEvtRec Rec[EVT_LOG_SIZE]; //Stored evts
  tEvtRec * pNxt; //ptr Next location;
  uint8_t num; //number of evts
} tEvtLog;
#define initEvtLog(Log) \
    atomic {\
      memset(&Log,0,sizeof(tEvtLog));\
      Log.pNxt = HplTwiEvtLog.Rec;\
      Log.num = 0;\
    };
#define aEvtLog_incpEvtRec(pEvtLog,pEvtRec)\
    pEvtRec++;\
    if (pEvtRec >= &pEvtLog->Rec[EVT_LOG_SIZE]) {\
      pEvtRec = &pEvtLog->Rec[0];\
    }
#define aEvtLog_incEvtRec(EvtLog)\
    EvtLog.pNxt++;\
    if (EvtLog.pNxt >= &EvtLog.Rec[EVT_LOG_SIZE]) {\
      EvtLog.pNxt = &EvtLog.Rec[0];\
    }


#define HPLADC_EVT_LOG_SIZE 32
typedef struct tHplAdcEvtRec {
  uint8_t port; //phy port
  uint16_t data; //lsb data
} tHplAdcEvtRec;

typedef struct tHplAdcEvtLog {
  tHplAdcEvtRec rec[HPLADC_EVT_LOG_SIZE]; //Stored evts
  tHplAdcEvtRec * pNxt; //ptr Next location;    
} tHplAdcEvtLog;

#define SENSORSADCHW_EVT_LOG_SIZE 64
typedef struct tSensorsAdcHwEvtLog {
  tEvtRec tupl[SENSORSADCHW_EVT_LOG_SIZE]; //Stored evts
  tEvtRec * pNxt; //ptr Next location;    
} tSensorsAdcHwEvtLog;

#define SENSORSMNG_EVT_LOG_SIZE 16
typedef struct tSensorsMngEvtLog {
  tEvtRec tupl[SENSORSMNG_EVT_LOG_SIZE]; //Stored evts
  tEvtRec * pNxt; //ptr Next location;    
} tSensorsMngEvtLog;

typedef struct {
  uint8_t devReports[DEV_ENUMERATION_END];
  uint8_t subsReports[SUBS_ENUMERATION_END]; //SUBSystem ID
} tMaintenanceReports;

typedef enum { //Gain settings for AD623/AD5161-100K
    eGsX16=0xF0, //X16 gain
    eGsX32=0xF8,
    eGsX64=0xFC,
    eGsX64_ACTUAL=0x47, //measured at 71 (decimal)
    eGsX128=0xFF,
    eGsEnd
  }eGainSetting;
  
typedef struct { // Buffer for results
    //Raw adc stored as a fraction of 2.048V +/- 0.05% or 1.024mV
    //That is significant bits and 
    uint8_t adcDataConfig; //eAdcDataConfig 016bit->3-16bit, Gain1-8  4-7 18bit Gain1-8
    uInt32Int16 rawFrac; // Data store as raw fractional component, MSB is always B15
    //    uByteWrdLng rawVal; //rawVal.w.l  or  rawVal.l
} tHal_adcMcpConfig;

#define SENSORS_ADC_CMD_BUFSIZE 9 //Should be SENSORS_ADC_HW_INSTEND
typedef struct { //AdcCmdBuffer
  void  *pLogAdcBuf;//tSensorsLogAdc
  uint8_t cmd; //linear Buffer
  uint8_t callersModuleId;
}tHal_adcCmdBuf;

typedef struct tSnsrAdcHwStatus {
  //SensorMngM module level
  //SensorsAdcHwM.cmd() Module not re-entrant and protected by
  bool sBusy; //semaphore Busy = TRUE/FALSE applies to this subsystem
    //norace as only simple reads, and writes protected, and char
  uint8_t conversionCnt;//Current conversion cnt. Range: 0 to (SENSORS_LOG_ADC_SIZE-1)
  uint8_t sensorAdcCmd; //Current command - eSensorsAdcHwCmd
  uint8_t cmdState;     //store one of eSnsrCmdState
  uint8_t cmdStateTwi;  //store one of eSnsrCmdStateTwi
  uint8_t activePort;   //active Port performed ADC
  uint8_t iaCntrl;   //-proto2 Instrument Amp control setting. eIaCntrlSettings
  uint8_t iaGain;    //Proto2 Instrument Amp gain setting - eGainSetting
  tHal_adcMcpConfig mcpConfig; //Proto3 ADC MCP3421 cntl settings
  tHal_adcCmdBuf buf[SENSORS_ADC_CMD_BUFSIZE];//Linear Stack 1 deep for all cmds
} tSnsrAdcHwStatus;
  //Seperate values for the MAX_NUM of 'logical data samples' the
  //system is capable of supporting
  // and ACT_NUM of data samples supported as a compile option
#define SENSOR_READINGS_MAX_NUM 16 //MAX_NUM logical
#define SENSOR_READINGS_ACT_NUM 9  //ACT_NUM of physical data samples supported in system


//#define SENSOR_READINGS_MAX_NUM 16  // was up to ver 1.1.
//#define SENSOR_ADC_OVERHEAD_CHNLS 4
//#define SENSOR_LOG_ADC_CHANNELS (SENSOR_READINGS_MAX_NUM+SENSOR_ADC_OVERHEAD_CHNLS)

typedef struct /* tSensorReadings */{
    // 9 bytes allias for tSr12BitAllMt02 - update any changes
  uint8_t msgType; //
  uint8_t length; //7 (not incl msgType&len) + Number of data elements x 2
  uint32_t time;
  uint8_t StatusSonde; //per eSr12ba_StatusSonde;
  uint16_t StatusSensors;
  uint16_t data[SENSOR_READINGS_ACT_NUM]; //Actual number of sensors implemented
} tSensorReadings;

typedef struct {
  uint8_t msgType; //
  uint8_t data[29/*cHalStdRecordLen*/];
} tHalStdRecord;
typedef struct/*ttHal_logger*/ { // Buffer for results
//Not more than cFfRecordSize
// 32bytes = 2 bytes overhead + 30 bytes payload
//            tSensorReadings   9bytes overhead + 10 readings@16bits +  1spare
  uint8_t destAckFlags; //eDrDestRecords - One bit per destination. Negative logic for FlashFile
  uint8_t chksum; //Checksum over "record
  tSensorReadings record;
  //Union tHalStdRecord;
} tHalLogger;


//Definition of number records written to flash
typedef struct {
  int16_t nxtRecordNum; //current Physical record addr where data can be written
  int16_t ackRecordNum; // highest consecutive record number that has been acked
  int32_t recordsWritten; //Total number written ?? what happens when it wraps
} tLoggerReport;

typedef enum  {
 eHrHibernationNone=   0x0000,//Active - not in hibernation
 eHrHibernationPwrLow= 0xa55a,//Low Power state - only timer function
 eHrHibernationPwrIdle=0x5aa5 //Idle Power State - minimial sys
} eHrHibstate;

typedef enum  {
  ehsmConsole =0x01,
  ehsmRs232Local =0x02,
  ehsmRs232CheckStatus =0x04,
} eHibSourceMask;

typedef struct {
  int16_t hibState; //value eHrHibstate
  uint8_t hibSource; //eHibSourceMask
} tHibernateReport;

typedef enum { //tSnsrMngStatus.lst enums
  SENSORSMNG_SELF_TEST,
  SENSORSMNG_CALIBRATE,
  SENSORSMNG_SIZEOF, //of previouse indications
  SENSORSMNG_END //marker
  } eSensorsMng;

typedef struct tSnsrMngStatus {
  //SensorMngM module level
  uint8_t smsState; //eSensorsMngStates - current working state
//  uint8_t lst[SENSORSMNG_SIZEOF]; //eSensorsMng
} tSnsrMngStatus;

typedef enum { //tSnsrMngStatus.lst enums
  SDMS_GATEWAY_REQ=0,
  SDMS_WALLTIME_REQ=1, //Request Walltime message
  SDMS_PROV_REQ=2,
  SDMS_COLLECTING_DATA=3,
  SDMS_TRANSFER_READINGS,//=4
  SDMS_MANAGEMENT_END_REQ,//=5,
  //SDMS_MANAGEMENT_END_DONE,//=6,
  SDMS_NOP,  //Testing etc

  SDMS_SIZEOF, //of previouse indications
  SDMS_END //marker
  } eSensorsDataMngStates;

typedef struct tSnsrDataMngStatus {
  //SensorDataMngM module level
  uint8_t sdmState; //eSensorsDataMngStates - current working state
//  uint8_t lst[SENSORSMNG_SIZEOF];
} tSnsrDataMngStatus;

/*typedef struct {
  //Basic Time with a simple check to see if it is valid
    uint16_t timeChecksum; //Simple check
    tos_time_t sysTime_mS;  
} sysTimeTupl_t;*/

typedef struct {
/* Master Controlling relationship for GatwayTransfer v DataSampling */
    bool isTransferMultGateway_bool; // TRUE=Sample1:GatewayN FALSE=SampleN:Gateway1
    uint8_t Max_Cnt; //This is valid if !0, if 0 its 1:1
  } tPeriodicCycle;

#if 0
typedef struct {
/*Time is a managed update since it is across multipile bytes, and the
 * system could fail int he middle of an update.
 * A checksum is stored with time to be able to do a simple validation
 */
  sysTimeTupl_t live;
  sysTimeTupl_t bckup;
  
  /* True if a sysTime is valid
   * This is internally set when
   *   - time is set
   *   - on valid time is found, making this a warm boot
   * If the check on live is found to be good, then it is assumed this
   * is also good.
   */
  bool timeIsProvisioned; // A valid time has been recieved
  } secureTime_t;

typedef enum { //Source of where the time is provisioned from
  eTss_init=0,
  eTss_UTC,
  eTss_Craft,
  eTss_end
} eTss_TimeSetSoruce;

  typedef struct {
    uint8_t     timeSetSource; //one of eTss_TimeSetSoruce
    secureTime_t lvSecTime; //live updates here
    secureTime_t snapSecTime; //snap shot on coming up for diagnositcs
    tos_time_t lastRebootTime_mS;
    tos_time_t lastProvisionedTime_mS;
    uint16_t provisionedTimeCntr;
  } simpleTimeStatus_t;
  #endif //#if 0
typedef enum  {
//Instance of the ADC_HW for accessing SensorsAdcHwM
// should be UNIQUE_SENSORS_ADC_HW ?

//*** ALSO if adding to this update SENSOR_ADC_CMD_BUFSIZE ****
  SENSORS_ADC_HW_INST_MNG,
  SENSORS_ADC_HW_INST001,
  SENSORS_ADC_HW_INST_SELF_TEST,
  SENSORS_ADC_HW_INST_CALIBRATE,
  SENSORS_ADC_HW_INST_MNG_PRSR,
  SENSORS_ADC_HW_INST_MNG_COND,
  SENSORS_ADC_HW_INST_PwrMng1,
  SENSORS_ADC_HW_INST_PwrMng2,
  SENSORS_ADC_HW_INSTEND
} eSENSORS_ADC_HW_INST;
/*typedef enum  {
  TWI_PERPHL_SensorsAdcHwM,
  TWI_PERPHL_INST001,
  TWI_PERPHL_SensorsMngPwrM,
  TWI_PERPHL_INSTEND
} eTWI_PERPHL_INST; */

typedef enum  { //Supposed to be 1:1 - but possibly fine tuned for
//Initially defined from HPLoneWireM - but may need fine tuning
// clck is 3.686Mhz - and loop is based on counting 4 cycles ~ 1.085uS
// So the following should be adjusted .... and are likely to be off
  DELAY_6uS = 3,
  DELAY_10uS = 5,
  DELAY_8uS = 8, //Should this be 16uS?
  DELAY_20uS = 10, //Guess based on 10uS
  DELAY_55uS = 29,
  DELAY_60uS = 32,
  DELAY_64uS = 34,
  DELAY_70uS = 37,
  DELAY_100uS = 100, // guessed based in 1=1us
  DELAY_410uS = 216,
  DELAY_480uS = 252
} eTimingProc_uwait;

typedef enum  { //min period is ~7mS
  eDELAY__20TbmS=20,
  eDELAY__40TbmS=40,
  eDELAY_100TbmS=100,
  eDELAY_500TbmS=500,
  eDELAY__1TbS= 1024,
  eDELAY__2TbS= 2048,
  eDELAY__2_5TS= 2560,
  eDELAY__3_6TS= 3600,
  eDELAY__5TbS= 5000,
  eDELAY__5_6TS= 5600,
  eDELAY_10TbS=10000,
  eDELAY_20TbS=20000,
  //eDELAY_30TbS=30000,
  //eDELAY_40TbS=40000L,
  eDELAY_80TbS=80000L,
  eDELAY_90TbS=90000L,
  eDELAY_120TbS=120000L,
  //eDELAY_CellPhoneDbg1=eDELAY_20TbS,
  //eDELAY_CellPhoneStartup1=eDELAY_80TbS,
  
  eTimingTbmsEnd
} eTimingTbms;

typedef enum  {
//Defines the management operations that can be performed while
//condition a sensor data stream
  eCsaFilter=0x01, //Filter Data
  eCsaModulo=0x02, //Modulo the data
  eCsaAdjConstSlope =0x04, //Adjust the data by the const and slope values
} eCsa_CondSnsrActions;

typedef struct {
  uint16_t cnstOffset; //Initial offset
  //uint8_t constK; // slope Multplier Integer
  //uint8_t constKfrac; // slope Multplier frac
  double slopeMult; //slope Multiplier
} compensationPt_t;
#define sizeofCompensationPt sizeof(compensationPt_t)

typedef struct {
//defines the management and constant data used in
// "conditioning" a stream of data from a sensor.
  uint8_t actions; //Bits as per eCondSnsrActions
  bool flushFilter; //Flush Filter next pass
  uint8_t deltaRange; //+- range allowed, typically 4xError
  uint8_t sampleErrorSize; //
  compensationPt_t compPt;
  uint16_t maxValue; //maximium value expected
} condSnsrData_t;
#define sizeofCondSnsrData sizeof(condSnsrData_t)

typedef enum  {
//Significant subsystems initiation tasks
  eSu_SysTick=0x01,
  eSu_StartUp=0x00, //0x02,
  eSu_end
} eStartUp;

typedef enum   {//UART Comms Type
  eUctInit,
  eUctLoopback, //Loop radio uart data back on it self
  eUctDebugConnect //Connect Radio through to Debug Port
} eUartCommType;



typedef enum {
	TIME_TBMS_SEC_MULTIPLIER=1024,
	TIME_CONVRT_MSEC_SEC_SHIFT=10,
	TIME_MSEC_MASK=0x3FF
} eTimeManipulation;

//SAM3X Power Options - requires #include "pmc.h"
#define setSam3xBus128bits() efc_set_flash_access_mode(EFC, 0)
#define setSam3xBus64bits()  efc_set_flash_access_mode(EFC, EEFC_FMR_FAM)

//for hardware\arduino\sam\system\libsam\source\timetick.c
// also hardware\arduino\sam\libraries\Scheduler\Scheduler.cpp
//#define DelayWait(xxx) delay(xxx)
//void Wait( volatile uint32_t dwMs );
#endif // az23project_h