/*Sonde general includes after implementatio { section
 *See PersStorM for access
 */
#ifndef HALPersistentStore_H
#define HALPersistentStore_H

enum {EEPROM_INT_BASE_ADDRESS=0
  ,PEI_myNodeDescriptionSize=200
  ,UNINIT_VALUE=0xA5
  ,DBACCESS_FAILED_ID=0xFFFE
  ,NODE_FAILED_ID=0xFFFE
};

typedef struct {
uint8_t mn;  //Minor - lower address
uint8_t mj; //Major - MSB address
} ver_t;

typedef union {
ver_t ver;  //
uint16_t wrd; //
} version_u;

//*******************************
typedef enum {//Physical hardware of install board
//Specifies how 16 bits is divided into multifunction hibernationTime
//Top 3 bits are the specifier - defining the format of the other 13 bits
//  see hibernationTimeS() / gatewayCheck_Cnt()
  eHt_spMask=0xE0 //Mask for specifier
  ,eHt_spElapsed_Seconds=0x00 //seconds - 1-8191/2.27hrs granualirity 1sec
  //eHT_spRserved2 0x20
  //eHT_spRserved3 0x40
  //eHT_spRserved4 0x60
  //eHT_spRserved5 0x80
  ,eHt_spPeriodicSmplData_min=0xA0 //PeriodicCycles 31 cycles, + time in minutes
  ,eHt_spPeriodicGateway_min=0xC0  //PeriodicCycles 31 cycles, + time in minutes
  ,eHt_spPeriodicGateway=0xE0  //PeriodicCycles 31 cycles, + time 8-2040/granularity 8seconds
  //Periodic
  ,eHt_prSecondsMult=0x08 //uptoVer2.2 Multiplier for lower byte
  ,eHt_prSecondsMult_min=0x3C //Ver2.3+ 1minuteToSeconds Multiplier for lower byte
                            //Range 0-255minutes, 4hrs 15minutes
} eHt_HibernationTime;

//*******************************
typedef enum {//Physical hardware of install board
  //eMT_MX_XBee=01, //baud=
  eMT_MX_XbpS1=02,
  eMT_MX_XTend=03,
  //eMT_MX_XStream=04, // retired and no hw support
  eMT_MX_XbpDm900=04, //XBeePee DigiMesh 900Mhz
  eMT_MX_XbpXsc900=05, //Future:XB XSC 900Mhz 
  //Other potential DigiModules
  eMT_MtCellular=0x10,
 
  //eMT_COMMON, //For common mode actions
  eMT_end
} eModemType;
typedef enum {//Minor for eMT_MtCellular
  eMtm_MtCellUnconfig=0xff,
  eMtm_MtGprs1=0x00, // Gprs Rserved revisions: 11 12 13
  eMtm_MtCdma1=0x04,
  eMtm_MtUndef
} eMtm_ModemTypeMultitech;

typedef struct {
  uint8_t mdmType; //eModemType
  uint8_t mdmTypeMinor;
} modemDescriptor_t;

//*******************************
//*******************************
typedef struct {
  //Retry attempts
    //bit0:4 timeout in seconds 1-33seconds
    // bit5:7 retry attempts 0-3
  uint8_t timeout; // definition of timeout/retry window
  uint8_t silence;// (future)number of seconds to not listen after each retry/timeout
}HAL_pcRetry_t;//ProtocolControlRetry

typedef struct { //Protocol
  HAL_pcRetry_t defFirstWindow;// definition of 1st retry window
  HAL_pcRetry_t def2ndWindow; //2nd window definitions
  uint8_t sizeFirstWindow;// Definition of first window attempts 2-255
} HAL_protocolConfig_t;
enum {
  eProtoCnfg_timeoutMask=0x1F,
  eProtoCnfg_retryMask=0xE0,
  eProtoCnfg_retryShift=0x05,
    //XBP-digi 0x07 Retry#1 0 times (tot 1 time) + 7 secs delay between retrys
    //XBP 2.5G=0x65 Retry#1 3 times (tot 4 time) + 5 secs delay between retrys
  eProtoCnfg_defFirstWindowDefault=0x07,
//  eProtoCnfg_defFirstWindowSilenceDefault=0x4,//Future
    //XBP-digi 0x05 Retry#2 0 time (tot 1 times) + 5 secs delay between retrys
    //XBP 2.5G 0x25 Retry#2 1 time (tot 2 times) + 5 secs delay between retrys
  eProtoCnfg_def2ndWindowDefault=0x05,
  // Retry#1 and then use retry#2 after exceeded
  eProtoCnfg_sizeFirstWindowDefault=0x10,
  eProtoCnfg_sizeFirstWindowMin=0x02,
  
  eProtoCnfg_InvalidLower=0x00, //documentation, implemented in code
  eProtoCnfg_InvalidHigher=0xff, //documentation, implemented in code
  eProtoCnfg_end
      
} eProtocolConfig;

typedef struct {
  uint8_t timeRequestTsec; //Time between requests 1-255
  uint8_t timeRequestNum; //Total number of requests 1-255
} timeRequests_t;

typedef struct {
  timeRequests_t tr_1st; //Initial time requests
  timeRequests_t tr_2nd; // After 1st has expired these apply
  uint8_t timeHibernationTmin; //Hibernation time between requests 1-254minutes
} wtrStartUpConfig_t; //wall time req StartupReq

enum {
  etrscTimeReq_1st_Tsec=0x14,  //20 secs between retrys
  etrscTimeReq_1st_Num= 0x40,  //Total number -notsupported?
  etrscTimeReq_2nd_Tsec=0x0a,  // future
  etrscTimeReq_2nd_Num= 0x03, //future
  etrscTimeHib_Tmin=    0x0A, //future
  etrscEnd
} eWtrStartUpConfig;
//*******************************
enum {//DebugSubSysMasks definitions
  eDssmMngmntPrsntMsk =0x03, //2=ThisDataPresent & valid 0 - anything else invalid
  eDssmMngmntPrsntYes =0x02,
  eDssmManualSize=0x10, //16 decimal
} eDebugSubSysMasks;

typedef struct {
  uint8_t size;   //Total size of this array sizeof(debugSubSysMasks_t)
  uint8_t mngmnt; // eDssmMngmntXxx
  uint8_t startup; //if not (STARTUP_PROV_NO or 0xff) - system can start
//  uint8_t debugSubSysMasks[eDssmManualSize];
  uint8_t dssMasks[eDssmManualSize];
} debugSubSysMasks_t;
#define cDebugSubSysMaskHeader 3 //size and mngmnt

//***Sensor Descriptors****************************
/*enum {// definitions of different type of descriptors
eSdtNone=0xFF, //Default of EEPROM
//eStdVolt,
eSdtPrsr001PSG=0x20,
eSdtPrsr005PSG=0x24,
eSdtThermFront=0x80, //Vishay238164055103
eSdtThermAir  =0x81, //NCP18XH103J03RBseries20K
//eSdtThermTop=  0x81,//

eSdUninit=0xff
} eSehsrDescriptors;
*/
typedef struct { //Sensor Descriptor Calibr
  //TODO make into reserved storage - calibration offset types to
  //support complex types in the future. 
  //Calibration Offset+ (x+Null)Mult/Div for total range
  //Note: there may be scale processing of some sort as well.
  int16_t Null; //Null initial offset
  int16_t Offset; //Offset in final units
  int16_t Mult;//Multipler of the number
  int16_t Divisor;//Divisor of the number

} SdCalbr_t; 
#define sizeOfSdCalbr() sizeof(SdCalbr_t)

typedef struct {
// Type eSdType01 constOffset + Ax
// adjust DebugPersistentStoreM for any changes here
  uint8_t sensorIdType; // eSensorsPhyType or eSNST_SensorsPhyIdType that maps to eSensorsAdcHwCmd for time being - future: thermistor, pressure_01 pressure_05
  uint8_t sensorIdFam; // - Family atrribute if needed
  uint8_t provisioning; // for this sensor - TODO definition
  uint16_t fltrStepOffset;//Max fltrStepOffset before filtering is bypassed
  
  //Calibration offsets for two ranges
  // A expected natural range
  // B amplified range
  SdCalbr_t calbrA;
  SdCalbr_t calbrB;
} SdTyp01_t;
#define sizeOfSensorConfig() sizeof(SdTyp01_t)
#define sizeOfSensorConfigHdr() (sizeof(SdTyp01_t) - 2*sizeof(SdCalbr_t))
typedef SdTyp01_t analogDigital_t;

typedef struct {
  analogDigital_t anlDig;
} manufacturingConfig_t;


//This will eventually move to an EEPROM on the head sensor board
typedef struct {
  SdTyp01_t sensors[SENSOR_READINGS_MAX_NUM];
//  snsrDescTyp01_t sensor2;
//  snsrDescTyp01_t sensor3;
} hdSensorManConfig_t;
#define sizeOfSensorManConfig() sizeof(hdSensorManConfig_t)

typedef struct {
 /*Store the time when doing a known watchdog reset.
  *This allows us to come out of the reset with a reasonable idea of
  *what the time is.
  */
  uint8_t chkSum;
  w64Two_u32_u time;
//  tos_time_t time;
} timeEntry_t;
enum {
  eetsEntryMask=0x03,
  eetsEntry0valid=0x01,
  eetsEntry1valid=0x10,
  eetsEntryNone=0xFF,
  eetsBuffer0=0,
  eetsBuffer1=1,
  eetsBufferNoValidTime=0xFF,
} eeTimeStatus;
typedef struct {
//B0:B1 =01 entry[0] valid
//B0:B1 =10 entry[1] valid
//B0:B1 =11 or 00 neither valiud
//B2-7 Undefined 1111-11
  uint8_t status; 
  timeEntry_t entry[2];//one or other is valid depending on status
} timeDef_t;
////geographical identifier /////
typedef struct { 
uint32_t longitude;
uint32_t Latitude;  
//#define PEI_myNodeCoordsLong 4
 //#define PEI_myNodeCoordsLat  8
 //Customer description - up to two lines with 160 chars + HTML formating
 uint8_t myNodeDescription[PEI_myNodeDescriptionSize];
} geoId_t;

typedef struct {
//temperature - reporting in C or F, accuracy, calibrated
//water height - accuracy, calibrated
//pressure - accuracy, calibrated
} PeiEepromSensorBrd_t;
typedef enum {
  eBp_rbatProvisioned=0x01   //When 1 should treat Rbat as provisioned
  ,eBp_rbatUnProvisioned=0x02//When 1 should treat Rbat as unprovisioned, above overrides
  ,eBp_rbatCharging   =0x04 //When 1 should treat Rbat as charging
  ,eBp_unprovisioned=0x00 //default for not provisioned in EE
  ,eBp_EepromUnprovisioned=0xff
} eBp_BatteryProvisioning_t;

typedef enum {
eCm_manageChannel=0x01 //If 1 should manage this channel
  //If managing channel the do the following
  ,eCm_spare1=0x02
  ,eCm_labelMask=0x0C
  ,eCm_labelNone=0x00 //No labels sent
  ,eCm_label1GaugeNode=0x04 //Send Gauge-xxxxx as label
  ,eCm_label2Label1=0x04 //Send label standard
  ,eCm_label1Label1=0x04 //Send manConfig

  ,eCm_stateSendOosM=0x04 //If 1 should send a state on channel connection/disconnection
} eCm_ChannelManagement_t;

//******************************************
//Structure that lays out the formation of the 4Kbytes of EEPROM 
typedef struct {
  //uint16_t size - put this in at some point
  uint16_t thisDbId; //0=Minor(LSB) 1=Major (MSBVersion of this - possibly make typedef later

  uint16_t myNodeId; //2LSB 3MSB This may need to be audited/replaced with MaxStream NodeId
  modemDescriptor_t thisRfmdmId; //4=Lsb, 5=Msb - version of the attached Modem
  //-----------------------------------
  //Format per eHT_HibernatioTime
  // 5432 1098 7654 3210
  // sssd dddd dddd dddd s- specifier / d for data specific to specifier
  // 0
  uint16_t hibernationTime; //Second increments. 0 - internal default
  HAL_protocolConfig_t HAL_protocolConfig; //8=1st 9= a=2nd b= c=size
  wtrStartUpConfig_t wtrStartupConfig; //{0d= 0e=}{0f= 10=}11
  
  debugSubSysMasks_t debugSubSysMasks;
  manufacturingConfig_t reserved_notUsed_manConfig;//This is for per system config and isn't current used. Can't change size
  //Possible use for Board Hw S/n etc

  geoId_t geoId;//Geographical Identifier also data present in Module
 //PerNode customer ID
 
//Sensor provisioning - here or in sensor EEPROM
  hdSensorManConfig_t hdSensorManConfig;
  timeDef_t lastKnownTime;
  //range
  //possible new
  uint8_t batteryProvisioning_eBp; //eBp_BatteryProvisioning_t - rbatProvisioned:1 rbatCharging
  uint8_t channelManage_eCm; //eCm_channelManagement   labelSend:1

  uint8_t reserved[0x3E]; //reserved for misc future expansion, no specific purpose but as a buffer
  uint8_t end[1]; //end marker
} PeiEepromInternal_t;
#define cPeiEepromInternalSize (sizeof(PeiEepromInternal_t) )

//******************************************
// eep Fifo/buffer definitions
  //**The following doesn't seem to make a lot of sense**
#define cEepSensorReadingsSize (sizeof(tSensorReadings)/*-2*(SENSOR_READINGS_MAX_NUM-SENSOR_READINS_ACT_USED)*/)
/** The buffer used to implement FIFO queue */
typedef struct {
  uint8_t status; // Status Bit0: 1- data value present here
                //  Bit1: ACK received for logical server0
  uint8_t size; // number of bytes in buf - expected to be &buf-1
} PerSnrsRdgsMng_t;
enum {
  ePsrmRecPresent=0x01,
  ePsrmRecAckRxLog0=0x02,
  ePrsmRarL0Mask = (ePsrmRecPresent|ePsrmRecAckRxLog0),
  ePrsmEnd
} Psrm_status;
typedef struct {
  PerSnrsRdgsMng_t mng;
  uint8_t buf[cEepSensorReadingsSize];
} eepSnsrRdgsRec_t;
#define cSnsrRdgsRecSize sizeof(eepSnsrRdgsRec_t)
#define cPeiSnsrRdgsFifoEepStart 0x400
#define cPeiSnsrRdgsFifoEepLast 0xFFF
#define cSnsrRdgsStored ((cPeiSnsrRdgsFifoEepLast-cPeiSnsrRdgsFifoEepStart)/cSnsrRdgsRecSize)
typedef struct {
  eepSnsrRdgsRec_t rec[cSnsrRdgsStored];
} PeiSnsrRdgsFifo_t;


#endif //HALPersistentStore_H
