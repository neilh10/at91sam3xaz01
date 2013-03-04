/*
 * uiDebugSubSysMasks.cpp
 * Copyright (c)2012 Azonde, All Rights Reserverd.
 
 * Created: Nov/11/2012 5:45:04 PM
 *  Author: neil
 *
 *
 *  Console
 *  Parse the UI Debug Sub Sys Maks setting

 *  
 *  Debug Mode???
 *  XX[ppp] [parm1[|parm2]]
 *  - TimeSetting
 *  - PowerManage
 *  -Flash - config Memory
 *  -logg
 *  -device monitoring (Radio..)
 *  -Hibernation?
 *  I2C?
 *
 *  descriptio const str XXpp #mandatoryParse,Abs|flexible,#optionParse
 *  delimitor
 *  parameter
 *  function
 *
 * 
  * Need
  SerialscanHex2Char
Serial.scanHex1Char
 */ 

#include "sam.h"
#include "Arduino.h"
#include "az23project.h"
#include "sysDebug.h"
#define cOffsetDbgMask MASK_INDX_SystemMngXxx
#include "uiConsole.h"
#include "HilDataFlash.h"

#ifdef __cplusplus
extern "C" {
#endif
//void uiConsoleParse(void);
//EepromIntrnl_readBuf(); becomes HilDataFlash_
//result_t EepromIntrnl_readBuf(void *pul_addr, uint16_t size, uint8_t *pBuf);;//HALpersStorInternal.cpp - was HALEepromInternal
//EepromIntrnl_writeBufReq()
result_t EepromIntrnl_writeByteReq(void *pul_addr, uint8_t data);//was HALEeprom.writeByteReq
result_t EepromIntrnl_writeBufReq(void *pul_addrEeprom, uint16_t sizeToWr, uint8_t *pBuf,uint8_t progCallBack_id);//was HALEeprom.writeBufReq

// EepromIntrnl_writeBufReq();

#ifdef __cplusplus
}
#endif



#ifdef __cplusplus
extern "C" {
#endif

/** D0C0U0T User Subsystems Debug Masks cmd parsing.
 * <p> </p>
 */
//#define DBG_DBGCONSOLE_TEST

//#include "debug.h"
#include "HALpersistentStore.h"
#include "HALpersistentStoreDef.h"
  result_t readIntEeprom(uint8_t options);
  void defaultDebugMasks();
 static const char strInternalBufSizeIssue[] = " InternalBufSizeIssue ";
 static const char strDebugMasks[] = " debugMasks";
// static const prog_uchar strSensorsMng[] = " sensorsMng";
 //static const uint8_t strNlCr[] = "\n\r ";
 static const char strFail[] = " DebugSubSysMasks_EeRd_Fail ";
 static const char strFailEepromIntrnl[] = "\n\r **FailEepromIntrnl";

  
/****************************************************/
/*  command result_t DebugCmdPort.init() {
    dbg(DBG_USR1, "Initializing\n");
  return SUCCESS;
  }*/


/*norace*/ uint8_t debugMasks[MASK_INDX_SIZE]; //njh moved to sysDebug.cpp

/*dc***************************************************/
 result_t DebugCmd_init() {
//  defaultDebugMasks();
  readIntEeprom(0);
  return SUCCESS;
  }

/*df*D9C9T9U9**************************************************/
void initMasks() {
// Set up default configuration for masks
uint8_t errLp;
  for (errLp=0; errLp<MASK_INDX_SIZE;errLp++) {
    debugMasks[errLp] =eDbgAll_errEvt;
    }
} // initMasks */
#if 0 
/*dc***************************************************/
//inline async command uint8_t systemMngXxxDebugMask.rd(){
//  return debugMasks[MASK_INDX_SystemMngXxx];
//}
/*dc***************************************************/
//inline async command uint8_t dataMngDebugMask.rd(){
//  return debugMasks[MASK_INDX_DataMngM];
//}
/*dc***************************************************/
inline async command uint8_t sensorsMngDebugMask.rd(){
  return debugMasks[MASK_INDX_SensorsMngM];
}
/*dc***************************************************/
inline command uint8_t sensorsMngXxxDebugMask.rd(){
  return debugMasks[MASK_INDX_SensorsMngXxx];
}
/*dc***************************************************/
inline async command uint8_t sensorsAdcHwDebugMask.rd(){
  return debugMasks[MASK_INDX_SensorsAdcHwM];
}
/*dc***************************************************/
inline async command uint8_t radioLinkLayer.rd(){
  return debugMasks[MASK_INDX_RadioLinkLayer];
}
/*dc***************************************************/
inline async command uint8_t radioSend.rd(){
  return debugMasks[MASK_INDX_RadioSend];
}

/*dc***************************************************/
inline command uint8_t routeInM.rd(){
  return debugMasks[MASK_INDX_RouteInM];
}
/*dc***************************************************/
inline command uint8_t HALHwStateSondeMask.rd(){
  return debugMasks[MASK_INDX_HALHwStateSondeM];
}
/*dc***************************************************/
inline async command uint8_t FlashSerial.rd(){
  return debugMasks[MASK_INDX_FlashSerial];
}
#endif //#if 0
/*dc***************************************************/
/*inline command uint8_t HALTwiPerphlM.rd(){
  return debugMasks[MASK_INDX_HALTwiPerphlM];
}*/
/*df*D9C9T9U9**************************************************/
result_t readIntEeprom(uint8_t options) {
  debugSubSysMasks_t debugSubSysMasks;
  uint8_t mskLp;
  result_t retResult;
  
  if (SUCCESS == (retResult = HilDataFlash.readBuf(getAddrEeInt(debugSubSysMasks),sizeof(debugSubSysMasks_t),(void *)&debugSubSysMasks))) {
    if ((eDssmMngmntPrsntYes == (eDssmMngmntPrsntMsk & debugSubSysMasks.mngmnt)) //Data valid and present
      && ((MASK_INDX_SIZE == debugSubSysMasks.size) //AND data is same revision
          || options)) //OR data_same_revision over ride is forced
    {
      for (mskLp=0;mskLp<MASK_INDX_SIZE;mskLp++){
        debugMasks[mskLp] = debugSubSysMasks.dssMasks[mskLp];
      }
    } else {
      Serial.print(strFail);
      Serial.print('=');
      Serial.print(debugSubSysMasks.mngmnt); //Hex?
      Serial.print('-');

      Serial.print(debugSubSysMasks.size); //Hex
      //call Printutils.printHex(debugSubSysMasks.size);
      //sMasks.size);
    }
  } else {Serial.print(strFailEepromIntrnl);}
  
  return retResult;
} //readEprom
/*df*D9C9T9U9**************************************************/
result_t parserReadIntEeprom(const char * pNxtChar) {
//  debugSubSysMasks_t debugSubSysMasks;
  uint8_t options;
//  uint8_t mskLp, retResult;

  //Check if the read should be forced
  pNxtChar +=2;
  options = ('F' == *pNxtChar) ? 1:0;
  return readIntEeprom(options);
} //parserReadIntEeprom
  debugSubSysMasks_t subSysMasks; //Need temp buffer
  
/*df*D9C9T9U9**************************************************/
result_t writeIntEeprom() {
  uint16_t bufMaskSize;
  uint8_t mskLp;  
//  pNxtChar++;
  //  options = ('F' == *pNxtChar) ? 1:0;
  bufMaskSize = MASK_INDX_SIZE;
  if ( eDssmManualSize < bufMaskSize) {
    Serial.print(strInternalBufSizeIssue);

    // WARNING: internal EEPROM is not large enough for all values
    bufMaskSize = eDssmManualSize;
  }  
  for (mskLp=0; mskLp<bufMaskSize; mskLp++){
    subSysMasks.dssMasks[mskLp] = debugMasks[mskLp];
  }
  subSysMasks.size = MASK_INDX_SIZE;
  subSysMasks.mngmnt = eDssmMngmntPrsntYes;//eDssmMngmntPrsntMsk;
  
  return ( HilDataFlash.writeBufReq(
    getAddrEeInt(debugSubSysMasks),
    (uint16_t)(bufMaskSize+cDebugSubSysMaskHeader),
    &subSysMasks
	,(uint8_t)0//, progCallBack_id	
	));
  //response writeBufDone
}//writeIntEprom


/*df*D9C9T9U9**************************************************/
result_t optMasks(const char * pNxtChar) {
  /*Parse short cuts for setting up for calibration tests
   * See help section
   */
  result_t retResult=SUCCESS;

  pNxtChar+=2;
  switch (*pNxtChar) {
  case 'p': initMasks();//pressure
    debugMasks[MASK_INDX_SensorsMngXxx] |= ( eDbgConsoleAccess|eDbgSnrsMng_Prsr|eDbgSnrsMng_PrsrCalbr);
    break;
  case 'd': initMasks(); //default
    debugMasks[MASK_INDX_SystemMngXxx]=eDbgAll_errEvt;
    debugMasks[MASK_INDX_DataMngM]=eDbgDataMng_sdmsAction|eDbgAll_errEvt;
    debugMasks[MASK_INDX_SensorsMngM]=eDbgSnsrMng_Output|eDbgAll_errEvt;
    //  debugMasks[MASK_INDX_SensorsMngXxx]=eDbgAll_errEvt;
    //  debugMasks[MASK_INDX_SensorsAdcHwM]=eDbgAll_errEvt;
    debugMasks[MASK_INDX_RadioSend]=eDbgRadio_Send|eDbgAll_errEvt;//|eDbgRadio_FramerReceive;
    debugMasks[MASK_INDX_RouteInM]=eDbgRouteIn_RxAll|eDbgAll_errEvt;
    //  debugMasks[MASK_INDX_HALHwStateSondeM]=eDbgAll_errEvt;
    //  debugMasks[MASK_INDX_FlashSerial]=eDbgAll_errEvt;
    break;
  case 'e': initMasks();break;//error only
  case 'm': initMasks();
    debugMasks[MASK_INDX_SystemMngXxx] |= (eDbgConsoleAccess|eDbgSysMngXxx_PwrMin);  
    debugMasks[MASK_INDX_RadioSend]    |= (eDbgRadio_Send);            
    break;
  case 'M': initMasks();
    debugMasks[MASK_INDX_SystemMngXxx] |= (eDbgConsoleAccess);  
    debugMasks[MASK_INDX_RadioLinkLayer]|=(eDbgRadio_RmApiDiscard|eDbgRadio_RmApiRxOctet|eDbgRadio_RmApiTxOctet);
    debugMasks[MASK_INDX_RadioSend]     |=(eDbgRadio_Send|eDbgRadio_ATrsp|eDbgRadio_Cntl);
    break;
  case 's': initMasks();//standard
    debugMasks[MASK_INDX_SystemMngXxx] |= (eDbgConsoleAccess);  
    debugMasks[MASK_INDX_RadioLinkLayer]|=(eDbgRadio_RmApiMng|eDbgRadio_RmApiDiscard);
    debugMasks[MASK_INDX_RadioSend]    |= (eDbgRadio_Send);  
    break;
  case 'S': initMasks();
    debugMasks[MASK_INDX_SystemMngXxx] |= (eDbgConsoleAccess);  
    debugMasks[MASK_INDX_RadioSend]       |=(eDbgRadio_Send);            
    debugMasks[MASK_INDX_HALHwStateSondeM]|=(eDbgPwrMng);  
    break;
  default:
    //print Help
    Serial.print(
   "\n\r d  default"
   "\n\r p  pressure calbriation"
   "\n\r e  errors"
   "\n\r m  minimal basic comms+ insufficient power"
   "\n\r M  Maximal comms"
   "\n\r s  standard comms + comms cntl"
   "\n\r S  Standard comms + power"
     );
    break;
  }
  return retResult;
} //readIntEeprom


/*df***************************************************/
void uiConsole_debugCmdHelpLocal(){
  Serial.print(
  "\n\r d[X[00-FF]][d W R T S D] debug mask X=A-all a-adcHw d-DataMng h-hwState m-Mng r-Rad x-Route or indx 0-F"
  );
} //helpCmd
/*dc***************************************************/
inline result_t uiConsole_debugCmdhelp() {
  uiConsole_debugCmdHelpLocal();
  return SUCCESS;
}//help

/*dc*D9C9T9U9**************************************************/
const result_t uiConsole_debugCmd(char *dataBufPtr,uint8_t pos,uint8_t inConPos) {
	//dbgOut1(eDbgAll_errEvt,22,"uiConsole_debugCmd",inConPos);
//command result_t DebugCmd.debugFn(const char *dataBufPtr)
  /** 'debug' Cmd processing.
    input
      01234567
      d  - print out state all
      dIAA - set to value  
       I = set to MASK_INDX_ value
          'a' - set all to value
          'd' -  DataMngM
          'h' - adc
          'm' - SensorsMngM
          'r' - RouteInM
          'x' - Radio, transmit
      dXAAw - set indx X (0-F) to AA
      'W' - write to EEPROM
      'R' - read from EEPROM
      'S' - startup
 Fut  'T[p|e]' setup for pressure and electrical conductivity
      'D' - default
  **/
  {
  int  vvalue;
  uint8_t mskCntr; // mskIdx;
  const char * pNxtChar;
  uint8_t primSwChar;
  result_t retResult = SUCCESS;
  int maskIndx = MASK_INDX_NOP;


  primSwChar = *(dataBufPtr+1); //Main Selector character
  pNxtChar = dataBufPtr+2;//&rxBuf[2] - the hex numbers
  switch (primSwChar) { //Look for primary selector
    //case 'A': and a few other used below
    case 'd': optMasks(dataBufPtr); break;
    case 'R': retResult = parserReadIntEeprom(dataBufPtr); break;
    case 'W': Serial.print(" tbd-Future");/*retResult = writeIntEeprom();*/ break;
    default: //not present so look for two hex characters
	
      //if (SUCCESS == (retResult = scanHex2Char(pNxtChar,&vvalue))) {
      if (1 == sscanf(pNxtChar,"%x",&vvalue)) {
        // found two hex numbers, now look for selector
        // assumed retResult = SUCCESS;
        switch (primSwChar) { //set all to read values
        case 'A':
          for (mskCntr = 0; mskCntr<MASK_INDX_SIZE;mskCntr++) {
            debugMasks[mskCntr] = vvalue;
          }
          maskIndx = MASK_INDX_NOP;
          break;
        case 'a': maskIndx = MASK_INDX_SensorsAdcHwM; break;
        case 'd': maskIndx = MASK_INDX_DataMngM; break;
        case 'h': maskIndx = MASK_INDX_HALHwStateSondeM; break;
        case 'm': maskIndx = MASK_INDX_SensorsMngM; break;
        //case 'R': - already handled above
        case 'r': maskIndx = MASK_INDX_RadioSend; break;
        //case 'W': - already handled above
        case 'x': maskIndx = MASK_INDX_RouteInM; break;          
        default:
          //check for valid offsets 0-F
          //retResult = scanHex1Char((dataBufPtr+1),&maskIndx);
		  if (1== sscanf((dataBufPtr+1),"%x",&maskIndx)) {retResult =FAIL;} 
      
        } 
      } else {retResult = FAIL;}
      if ((MASK_INDX_NOP != maskIndx) && (SUCCESS == retResult)) {
        debugMasks[maskIndx] = vvalue;  
      }
      break;
  }
    //print them all 
  Serial.print(strDebugMasks);

  //if (SUCCESS == retResult) {
  Serial.println();
  for (mskCntr = 0; mskCntr<MASK_INDX_SIZE;mskCntr++) {
    Serial.print(' ');
    Serial.print(mskCntr);//hex    
  }
  Serial.println();
  for (mskCntr = 0; mskCntr<MASK_INDX_SIZE;mskCntr++) {
    Serial.print(' ');
    Serial.print(debugMasks[mskCntr]);    //hex
  }
  //}
  return retResult;
  }
  
  } 
#ifdef __cplusplus
}
#endif
