/*
 * sysDebug.cpp
 * Copyright (c)2012 Azonde, All Rights Reserverd.
 
 * Created: 10/25/2012 5:45:04 PM
 *  Author: neil
 *
 *  Description - see sysDebug.h
 *
 *  
 */ 
#include "sam.h"
#include "Arduino.h"
#include "az23project.h"
#include "sysDebug.h"
//#include "rtt.h"


// ***************************************************************
sysDebugClass::sysDebugClass(void) {
  for (uint8_t modLp=0;modLp<MASK_INDX_SIZE;modLp++) {
	  moduleAcc[modLp] = eDbgAll_errEvt;
   }
  }//sysDebugClass
  
  // ***************************************************************


//#define DBG_DBGCONSOLE_TEST
//includes SensorsAdc;
//includes TosTime;
//includes FramerRfc1622;


//  uses {
//    interface PrintUtils as PrintUtils;
//    interface DebugMsg as sendDbg;
//    interface Time;
//    interface Clock;
 

//implementation {
//#include "sondeBM001.h"  //might be able to remove.
//#include "DelugeMsgs.h"
//#include "Deluge.h"
//#include "DelugeMetaData.h"
//#include "mx_mods.h"
//#include "Timer.h"

//  static const prog_uchar strCrLf[] = "\n\r";
//  static const prog_uchar strSpace[] = " ";
//  static const prog_uchar strDash[] = "-";
//  static const prog_uchar strSqBL[] = "[";
//  static const prog_uchar strSqBR[] = "]";
//  static const prog_uchar strColon[] = ":";
//  static const prog_uchar strTOSmsg[] = " TOSmsg:";
//  static const prog_uchar strMHop[] = " MHop:";
//  static const prog_uchar strSr12BitAll[] = " StrSr12BitAll:";
//  static const prog_uchar strAM_DelugeAdvMsg[] = " AM_DelugeAdvMsg:";
// static const prog_uchar sPortHeader[] = "\n\r     A  B  C  D  E  F  G";
// static const prog_uchar sPin[] = "\n\r PIN ";
// static const prog_uchar sPort[] = "\n\rPort ";
// static const prog_uchar sDdr[] = "\n\r DDR ";


/*cf***************************************************/
void sysDebugClass::out1(const char *pStrIn, char uniqueId,uint32_t param1) {
   hdr(pStrIn,uniqueId);
   Serial.print(param1);
   //Serial.print(' ');
   }
   
/*cf***************************************************/
uint16_t sysDebugClass::rdDbgMask(uint8_t module_id) {
	 return moduleAcc[module_id];
}	

/*cf***************************************************/
void sysDebugClass::hdr(const char *pStrIn,char uniqueId) {
	Serial.println();
	dbgTime();
	Serial.print(pStrIn);
	Serial.print(':');
	Serial.print(uniqueId);
	Serial.print(':');
	}

/*cf***************************************************/
void sysDebugClass::dbgTime(void) {

// printUtils.time(w64Two_u32_u time)

//See icu\source\il8n\calendar.cpp
//  uint32_t TimeLow32Tbms;
  uint32_t TimeSecsTsec; // Large range
  uint16_t Time_mSecTbms;//0-1023 binary mS
  w64Two_u32_u time;
  //tos_time_t tosTime=RTT_GetTime(); GetCurrentTime(); //GetTickCount();
  uint32_t time_mS = RTT_GetTime(RTT);
#define JAVA_TIME
#ifndef JAVA_TIME
  struct {
    uint16_t days; //large, doesn't matter if overflows
    uint8_t hrs; //0-23 hours
    uint8_t mins; //0-59 minutes
    uint8_t secs; //0-59 seconds
  } Time24hr;
  uint32_t SecsLeftInDayTsec; //0-86,399secs
  uint16_t SecsLeftInHrTsec; //0-3599 secs
#endif

  //Expect that there is no binary seconds - add it in some way
  //time.two_u32.l.w32 = (tosTime.low32 & cTimeMaskOff_bmSec ) | (((uint16_t)call Clock.readCounter())<<cTcnt2ShiftTo_bmSec); 
  //time.two_u32.m.w32 = tosTime.high32;
  time.two_u32.l.w32 = time_mS;
   time.two_u32.m.w32 = 0;
  Time_mSecTbms = time.two_u32.l.two_u16.l & TIME_MSEC_MASK;
  TimeSecsTsec  = time.w64 >> TIME_CONVRT_MSEC_SEC_SHIFT;
  
//  TimeLow32Tbms = call Time.getLow32();
//  TimeSecsTsec = TimeLow32Tbms >>TOS_TIME_CONVRT_MSEC_SEC_SHIFT; //Cutoff binary Tbms
//  Time_mSecTbms = (uint16_t)TimeLow32Tbms & TOS_TIME_MSEC_MASK;
  
  //call PrintUtils.nlcr();  
#ifndef JAVA_TIME
  //9 char hex time as
  //xxxxxxxx:
  //Print in hex as 19 char
  //day hrs min sec bmsecs   
  //xx:hh:mm:ss:mmm:
#define cSecsInAMin 60
#define cSecsInAnHr cSecsInAMin*60  
#define cSecsInAday cSecsInAnHr*24
  Time24hr.days = TimeSecsTsec/cSecsInAday;
  SecsLeftInDayTsec = (TimeSecsTsec - (uint32_t)Time24hr.days*cSecsInAday);
  Time24hr.hrs = SecsLeftInDayTsec/cSecsInAnHr;
  SecsLeftInHrTsec = SecsLeftInDayTsec - Time24hr.hrs*cSecsInAnHr;
  Time24hr.mins = SecsLeftInHrTsec/cSecsInAMin;
  Time24hr.secs = SecsLeftInHrTsec - Time24hr.mins*cSecsInAMin;
  PrintUtils.printHex(Time24hr.days); //print first two digits in Hex
  PrintUtils.printP(strColon);
  PrintUtils.printDec(Time24hr.hrs);
  PrintUtils.printP(strColon);
  PrintUtils.printDec(Time24hr.mins);
  PrintUtils.printP(strColon);
  PrintUtils.printDec(Time24hr.secs);
  PrintUtils.printP(strColon);
  PrintUtils.printDecword3(Time_mSecTbms);
#else
//Serial.print("dbTime:");
  Serial.print(TimeSecsTsec,DEC);
  //SerconClass.print3(TimeSecsTsec,DEC);
  Serial.print('.');
  //Print 3 Hex chars for 10 bits, giving a fraction of 1000mS
//  call PrintUtils.printHexNibble(Time_mSecTbms>>8);  
  //  call PrintUtils.printHex(Time_mSecTbms);
  //Print 2 dec char for 8 bits giving 1/256 or 4mS
   Serial.print(Time_mSecTbms,DEC);
  
#endif
  Serial.print(':');
}

#if 0
/*ce***************************************************/
command void sendDbg.AdcBuf(const prog_uchar * pStr,const tSensorsLogAdc *pLogAdc) {
	int8_t cnt;
	int8_t length;

	call PrintUtils.printP(strCrLf);
	call PrintUtils.printP(pStr);
	
	call PrintUtils.printP(strColon);
	call PrintUtils.printHex(pLogAdc->status);
	call PrintUtils.printP(strDash);
	call PrintUtils.printHex(pLogAdc->iaCntrl);
	call PrintUtils.printP(strDash);
	call PrintUtils.printHex(pLogAdc->iaGain);
	call PrintUtils.printP(strDash);

	length = pLogAdc->numAdcReadings;
	call PrintUtils.printHex(length);
	
	for (cnt = 0; cnt < SENSORS_LOG_ADC_SIZE ; cnt++){
		if (0==(cnt & 0x3)) {
			call PrintUtils.printP(strColon);
		} else {
			call PrintUtils.printP(strSpace);
		}
		call PrintUtils.printHexword(pLogAdc->data[cnt]);
	}
	call PrintUtils.printP(strCrLf);
	
} //.AdcBufEvt() */
#endif

#if 0

//*****************************************************
void  dbgMsgSensorReadings(const tSensorReadings *pRd) {
int8_t cnt;
int8_t length;
  call PrintUtils.printHex(pRd->msgType);
  call PrintUtils.printP(strSpace);
  length = pRd->length;
  call PrintUtils.printHex(length);
  call PrintUtils.printP(strSpace);
  call PrintUtils.printHexlong(pRd->time);
  call PrintUtils.printP(strDash);
  call PrintUtils.printHex(pRd->StatusSonde);
  call PrintUtils.printP(strDash);
  call PrintUtils.printHexword(pRd->StatusSensors);

  //Adjust length to number data points
  length = ((length -MT_Sr12bitAllMt02OvrhdLen)>>1);
  if (length >SENSOR_READINGS_ACT_NUM) {
    length = SENSOR_READINGS_ACT_NUM;
  }
  for (cnt = 0; cnt < length ; cnt++){
    if (0==(cnt & 0x3)) {
      call PrintUtils.printP(strColon);
    } else {
      call PrintUtils.printP(strSpace);
    }
    call PrintUtils.printHexword(pRd->data[cnt]);
  }
//  call PrintUtils.printP(strCrLf);    
}  
/*ce***************************************************/
event void sendDbg.sensorReadings(const prog_uchar * pStr,uint8_t uniqueId,const tSensorReadings *pRd) {

  //** Merge with MtSr12bitAll if possible
  dbgMsgHdr(pStr,uniqueId);
  dbgMsgSensorReadings(pRd);
} //.sensorReadings() */

/*ce***************************************************/
 void dbgSensorsLogAdc(const tSensorsLogAdc *pRd) {
int8_t cnt;
int8_t length;

  call PrintUtils.printHex(pRd->status);
  call PrintUtils.printP(strDash);
  call PrintUtils.printHex(pRd->iaCntrl);
  call PrintUtils.printP(strDash);
  call PrintUtils.printHex(pRd->iaGain);
  call PrintUtils.printP(strDash);
  length = pRd->numAdcReadings;
  call PrintUtils.printHex(length);

  if (length >SENSOR_READINGS_ACT_NUM) {
    length = SENSOR_READINGS_ACT_NUM;
  }
  for (cnt = 0; cnt < length ; cnt++){
    if (0==(cnt & 0x3)) {
      call PrintUtils.printP(strColon);
    } else {
      call PrintUtils.printP(strSpace);
    }
    call PrintUtils.printHexword(pRd->data[cnt]);
  }
//  call PrintUtils.printP(strCrLf);    
  
 } //dbgSensorsLogAdc() */
/*ce***************************************************/
event void sendDbg.sensorsLogAdc2(const prog_uchar * pStr,uint8_t uniqueId,uint16_t prm2,
  const tSensorsLogAdc *pRd) {
  //Print with two supplied paramters - one 16bits, and ptr
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHexword(prm2);
  call PrintUtils.printChar(':');
  dbgSensorsLogAdc(pRd);
} //.sensorLogAdc() */

/*ce***************************************************/
 event void sendDbg.sensorsLogAdc(const prog_uchar * pStr,uint8_t uniqueId,const tSensorsLogAdc *pRd) {

  dbgMsgHdr(pStr,uniqueId);
  dbgSensorsLogAdc(pRd);
 } //.sensorLogAdc() */
 
/*ce***************************************************/
 event void sendDbg.halLogger(const prog_uchar * pStr,uint8_t uniqueId,const tHalLogger *pRd,uint8_t recLength) {
//int8_t cnt;
//int8_t length;

  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHex(pRd->destAckFlags);
  call PrintUtils.printHex(pRd->chksum);
  if  (recLength > 0) {
    call PrintUtils.printP(strSqBL);
    dbgMsgSensorReadings(&pRd->record);
    call PrintUtils.printP(strSqBR);
  }
//  call PrintUtils.printP(strCrLf);    
 }//sendDbg.halLogger

 /*ce***************************************************/
event void sendDbg.txStatusDm8B(const prog_uchar * pStr,uint8_t uniqueId,  const void *pMsgIn) {
// Modem Status 0x8B processing -
// XBP 900 DigiMesh v182x 90000903_B pg 33
  tMsgRxGeneric *pMsg = (tMsgRxGeneric *)pMsgIn;

  //Print with two supplied paramters - one 16bits, and ptr
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHex(pMsg->u.rxTxStatus8B.frameId);
  call PrintUtils.printChar(':');
  call PrintUtils.printHex(pMsg->u.rxTxStatus8B.remAddr1);
  call PrintUtils.printHex(pMsg->u.rxTxStatus8B.remAddr2);
  call PrintUtils.printP(PSTR(" Retry="));
  call PrintUtils.printHex(pMsg->u.rxTxStatus8B.txRetryCnt);
  call PrintUtils.printP(PSTR(" Discovery="));
  call PrintUtils.printHex(pMsg->u.rxTxStatus8B.statusDiscovery);

} //sendDbg.txStatusDm8B

/*cf***************************************************/
void tosMsgHdr(const TOS_Msg *pMsg) {
  
  call PrintUtils.printP(strTOSmsg);
  call PrintUtils.printHexword(pMsg->addr);
  call PrintUtils.printChar('-');
  call PrintUtils.printHex(pMsg->type);
  call PrintUtils.printChar('-');
  call PrintUtils.printHex(pMsg->group);
  call PrintUtils.printChar('@');
  call PrintUtils.printHex(pMsg->length);
}
/*cf***************************************************/
void tosMHopHdr(const TOS_MHopMsg *pMsg) {
  
  call PrintUtils.printP(strMHop);
  call PrintUtils.printHexword(pMsg->sourceaddr);
  call PrintUtils.printChar('-');
  call PrintUtils.printHexword(pMsg->originaddr);
  call PrintUtils.printChar('+');
  call PrintUtils.printHexword(pMsg->seqno);
  call PrintUtils.printChar('+');
  call PrintUtils.printHex(pMsg->hopcount);
}
/*ce***************************************************/
void MtSr12bitAll(tSr12bitAllMt02 *pMsg) {
  int8_t cnt;
  int8_t lenInWords;
  uint8_t msgTypeLength = pMsg->msgTypeLen;
  
  call PrintUtils.printP(strSr12BitAll);
  call PrintUtils.printHex(msgTypeLength);
  call PrintUtils.printChar('-');
  call PrintUtils.printHexlong(pMsg->time);
  call PrintUtils.printChar('-');
  call PrintUtils.printHex(pMsg->StatusSonde);
  call PrintUtils.printChar('+');
  call PrintUtils.printHexword(pMsg->StatusSensors);
  call PrintUtils.printChar('=');
  lenInWords = ((msgTypeLength - MT_Sr12bitAllMt02OvrhdLen )>>1);
  /* Debug if needed
  call PrintUtils.printHex(pMsg->msgTypeLen);
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(MT_Sr12bitAllMt02HdrLen);
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(lenInWords);
  call PrintUtils.printChar('='); */
  
  for (cnt = 0; cnt < lenInWords ; cnt++){
    call PrintUtils.printHexword(pMsg->SensorReading[cnt]);
    
    if (0x0F==(cnt&0x0F)) {
          call PrintUtils.printP(strCrLf);
        } else if (0x3==(cnt & 0x3)) {
          call PrintUtils.printChar(':');
        } else {
          call PrintUtils.printChar(' ');
        }
      }
} //MtSr12bitAll

/*ce***************************************************/
void Am_DelugeAdvMsgHndl(const TOS_Msg *pTosMsg) {
  DelugeAdvMsg *pMsg = (DelugeAdvMsg *)pTosMsg->data;
//  uint8_t msgTypeLength = pTosMsg->length;
  
  call PrintUtils.printP(strAM_DelugeAdvMsg);
  call PrintUtils.printHexword(pMsg->sourceAddr);
  call PrintUtils.printChar('-');
  call PrintUtils.printHex(pMsg->version);
  call PrintUtils.printChar('-');
  call PrintUtils.printHex(pMsg->type);
  call PrintUtils.printChar(':');


  call PrintUtils.printHexword(pMsg->nodeDesc.vNum);
  call PrintUtils.printChar(' ');
  call PrintUtils.printHexlong(pMsg->nodeDesc.uid);
  call PrintUtils.printChar(' ');
  call PrintUtils.printHexword(pMsg->nodeDesc.imgNum);
  call PrintUtils.printChar(':');

  call PrintUtils.printHexlong(pMsg->imgDesc.uid);
  call PrintUtils.printChar(' ');
  call PrintUtils.printHexword(pMsg->imgDesc.vNum);
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(pMsg->imgDesc.imgNum);
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(pMsg->imgDesc.numPgs);
  call PrintUtils.printChar('=');
  call PrintUtils.printHex(pMsg->imgDesc.numPgsComplete);
  
  call PrintUtils.printChar(':');  
  call PrintUtils.printHex(pMsg->numImages);
}
/*cf***************************************************/
void Am_SourceLabelInd(const TOS_Msg *pTosMsg) {
  tMng_SourceLabelInd_Am09 *pMsg = (tMng_SourceLabelInd_Am09 *)pTosMsg->data;
//  uint8_t msgTypeLength = pTosMsg->length;
  
  call PrintUtils.printP(PSTR("LabelInd:"));
  call PrintUtils.printD(pMsg->label);
}//Am_SourceLabelInd
/*cf***************************************************/
void Am_SourceStateChngeInd(const TOS_Msg *pTosMsg) {
  tMng_SourceStateChngInd_Am0B *pMsg = (tMng_SourceStateChngInd_Am0B *)pTosMsg->data;
//  uint8_t msgTypeLength = pTosMsg->length;
  
  call PrintUtils.printP(PSTR("SrcStateChngInd:Old/New="));
  call PrintUtils.printHex(pMsg->oldState);
  call PrintUtils.printChar('/');
  call PrintUtils.printHex(pMsg->newState);
  call PrintUtils.printChar(':');
  call PrintUtils.printD(pMsg->label);
}//Am_SourceStateChngeInd

/*cf***************************************************/
void printTosMhopMsg(const TOS_Msg *pMsg) {
  int8_t cnt;
  int8_t length;
  TOS_MHopMsg *pMHop = (TOS_MHopMsg *)pMsg->data;
  
  tosMsgHdr(pMsg);
  tosMHopHdr(pMHop);
  call PrintUtils.printP(strCrLf); 
  length = pMsg->length - (/*eTOS_MSG_Overhead +*/ eTOS_MHopMsg_Overhead);
/* //Debug if needed
  call PrintUtils.printHex(eTOS_MSG_Overhead);
  call PrintUtils.printChar('+');
  call PrintUtils.printHex(eTOS_MHopMsg_Overhead);
  call PrintUtils.printChar('+');
  call PrintUtils.printHex(length);
  call PrintUtils.printChar('=');
  call PrintUtils.printHex(TOSH_DATA_LENGTH);
  call PrintUtils.printP(strCrLf); /* */
  
  if (length >TOSH_DATA_LENGTH) {
    length = TOSH_DATA_LENGTH;
  }

  switch (pMHop->data[0]) {
    case MT_Sr12bitAllMt02: MtSr12bitAll((tSr12bitAllMt02 *)pMHop->data); break;
    default:
      for (cnt = 0; cnt < length ; cnt++){
        call PrintUtils.printHex(pMHop->data[cnt]);
    
        if (0x0F==(cnt&0x0F)) {
          call PrintUtils.printP(strCrLf);
        } else if (0x3==(cnt & 0x3)) {
          call PrintUtils.printChar(':');
        } else {
          call PrintUtils.printChar(' ');
        }
      }
  }
//  call PrintUtils.printP(strDash);
//  call PrintUtils.printHex(pMsg->crc);
} //.printTosMhopMsg() */
/*cf***************************************************/
void printTosSMsg(const TOS_Msg *pMsg) {
  int8_t cnt;
  int8_t length;
 // TOS_MHopMsg *pMHop = (TOS_MHopMsg *)pMsg->data;
  
  tosMsgHdr(pMsg);
//  tosMHopHdr(pMHop);
  call PrintUtils.printP(strCrLf); 
  length = pMsg->length; //- eTOS_MSG_Overhead;
  
  if (length >TOSH_DATA_LENGTH) {
    length = TOSH_DATA_LENGTH;
  }

  switch (pMsg->type) {
//    case MT_Sr12bitAllMt02: MtSr12bitAll((tSr12bitAllMt02
    //    *)pMHop->data); break;
    //case AM_DELUGEREQMSG AM_DELUGEDATAMSG AM_NETPROGMSG
  case AM_DELUGEADVMSG:
    Am_DelugeAdvMsgHndl(pMsg);
    call PrintUtils.printP(strCrLf); 
    //break; - fall through
  case AM_SOURCE_LABEL_IND:
    Am_SourceLabelInd(pMsg);
    break;
  case AM_SOURCE_STATE_CHANGE_IND:
    Am_SourceStateChngeInd(pMsg);
    break;
  default:
      for (cnt = 0; cnt < length ; cnt++){
        call PrintUtils.printHex(pMsg->data[cnt]);
    
        if (0x0F==(cnt&0x0F)) {
          call PrintUtils.printP(strCrLf);
        } else if (0x3==(cnt & 0x3)) {
          call PrintUtils.printChar(':');
        } else {
          call PrintUtils.printChar(' ');
        }
      }
  }
//  call PrintUtils.printP(strDash);
//  call PrintUtils.printHex(pMsg->crc);
} //.printTosSMsg() */
/*ce***************************************************/
event void sendDbg.tosSMsg(const prog_uchar * pStr,const uint8_t uniqueId,const TOS_Msg *pMsg) {
  
  dbgMsgHdr(pStr,uniqueId);
  printTosSMsg(pMsg);
}
/*ce***************************************************/
event void sendDbg.tosMhopMsg(const prog_uchar * pStr,const uint8_t uniqueId,const TOS_Msg *pMsg) {
  
  dbgMsgHdr(pStr,uniqueId);
  printTosMhopMsg(pMsg);
}
/*  typedef struct _MsgRcvEntry {
    uint8_t Proto;
    uint8_t Token;	// Used for sending acknowledgements
    uint16_t Length;	// Does not include 'Proto' or 'Token' fields
    TOS_MsgPtr pMsg;
  } MsgRcvEntry_t ; */

#define NoCompile
#ifdef NoCompile

/*ce***************************************************/
event void sendDbg.Rfc1622Msg(const prog_uchar * pStr,uint8_t uniqueId,const char *pMsg) {
  //int8_t cnt;
  //int8_t length;
//TOS_MHopMsg *pMHop = (TOS_MHopMsg *)pMsg->data;
    MsgRcvEntry_t *pRfc1622 = (MsgRcvEntry_t *)pMsg;

    dbgMsgHdr(pStr,uniqueId);
    
    call PrintUtils.printHex(pRfc1622->Proto);
    call PrintUtils.printHex(pRfc1622->Token);
    call PrintUtils.printHex(pRfc1622->Length);

    //printTosMhopMsg( (const TOS_Msg *) pMsg); - causes compiler error 
    printTosMhopMsg((const TOS_MsgPtr) pRfc1622->pMsg);
    /* */
}
#endif
/*ce***************************************************/
 event void sendDbg.bufGen(const prog_uchar * pStr,const uint8_t uniqueId,const void *pInBuf,uint8_t length) {
   int8_t cnt;
   uint8_t *pRd = (uint8_t *) pInBuf;

  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printP(strColon);
  call PrintUtils.printHex(length);
  call PrintUtils.printP(strColon);

#define MAX_LENGTH 0x20
  if (length >MAX_LENGTH) {
    length = MAX_LENGTH;
    call PrintUtils.printHex(length);
    call PrintUtils.printP(strColon);
  }
  for (cnt = 0; cnt < length ; cnt++,pRd++){
    if (0==(cnt & 0x3)) {
      call PrintUtils.printP(strColon);
    } else {
      call PrintUtils.printP(strSpace);
    }
    call PrintUtils.printHex(*pRd);
  }
//  call PrintUtils.bufGen()    
  
} //.sensorLogAdc() */
/*ce***************************************************/
inline event void sendDbg.str1(const prog_uchar * pStr,uint8_t uniqueId) {
  dbgMsgHdr(pStr,uniqueId);
} //.word() */
/*ce***************************************************/
event void sendDbg.oChar(const uint8_t oChar) {
  call PrintUtils.printChar(oChar); 
} //.oChar() */
/*ce***************************************************/
event void sendDbg.oByte(const uint8_t outByte) {
  call PrintUtils.printHex(outByte); 
} //.oByte() */
/*ce***************************************************/
event void sendDbg.oWord(const uint16_t outWord) {
  call PrintUtils.printHexword(outWord); 
} //.oWord() */
/*ce***************************************************/
inline event void sendDbg.oStrD(const uint8_t * pStr) {
  call PrintUtils.printD(pStr); 
} //.oStr() */
/*ce***************************************************/
inline event void sendDbg.oStrP(const prog_uchar * pStr) {
  call PrintUtils.printP(pStr); 
} //.oStr() */
/*ce***************************************************/
event void sendDbg.oChrNbl(const uint8_t uniqueId,const uint8_t otByte) {
  call PrintUtils.printChar(uniqueId);
  call PrintUtils.printHexNibble(otByte); 
} //.oChrNbl() */
/*ce***************************************************/
event void sendDbg.oChrByt(const uint8_t uniqueId,const uint8_t otByte) {
  call PrintUtils.printChar(uniqueId);
  call PrintUtils.printHex(otByte); 
} //.oChrByt() */
/*ce***************************************************/
event void sendDbg.oChrWord(const uint8_t uniqueId,const uint16_t otWord) {
  call PrintUtils.printChar(uniqueId);
  call PrintUtils.printHexword(otWord); 
} //.oChrByt() */


/*ce***************************************************/
inline event void sendDbg.oByteSpace(const uint8_t otByte) {
    call PrintUtils.printHex(otByte); 
    call PrintUtils.printChar(' ');
} //.oByteSpace() */
/*ce***************************************************/
inline event void sendDbg.oWordSpace(const uint16_t otWord) {
    call PrintUtils.printHexword(otWord); 
    call PrintUtils.printChar(' ');
} //.oWordpace() */
/*ce***************************************************/
inline event void sendDbg.oWord32Space(const uint32_t otWord) {
    call PrintUtils.printHexlong(otWord); 
    call PrintUtils.printChar(' ');
} //.oWordSpace() */
/*ce***************************************************/
inline event void sendDbg.oEol() { //End Of Line
    call PrintUtils.printChar('\n');
    call PrintUtils.printChar('\r');
} //.oEol() */

/*ce***************************************************/
event void sendDbg.u8u16(const prog_uchar * pStr,uint8_t uniqueId,const uint8_t outByte,const uint16_t outWord) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHex(outByte); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHexword(outWord); 
} //.u8u16 */
/*ce***************************************************/
event void sendDbg.u8u16u32(const prog_uchar * pStr,uint8_t uniqueId,const uint8_t outByte,const uint16_t outWord,const uint32_t outLong) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHex(outByte); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHexword(outWord); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHexlong(outLong); 
} //.u8u16u32 */

/*ce***************************************************/
event void sendDbg.u8u32(const prog_uchar * pStr,uint8_t uniqueId,const uint8_t outByte,const uint32_t outLong) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHex(outByte); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHexlong(outLong); 
} //.u8u32 */

/*ce***************************************************/
event void sendDbg._uint8_(const prog_uchar * pStr,uint8_t uniqueId,const uint8_t outByte) {
  
  dbgMsgHdr(pStr,uniqueId);
//  call PrintUtils.printChar('=');
  call PrintUtils.printHex(outByte); 
  //call PrintUtils.printChar(':');
} //.word() */
/*ce***************************************************/
event void sendDbg._2uint8_(const prog_uchar * pStr,uint8_t uniqueId,uint8_t param1,uint8_t param2) {
  
  dbgMsgHdr(pStr,uniqueId);
//  call PrintUtils.printChar('=');
  call PrintUtils.printHex(param1); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(param2); 
} //.word() */

/*ce***************************************************/
event void sendDbg._uint8_2(const prog_uchar * pStr,uint8_t uniqueId,uByteWrd w16) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHex(w16.b.m); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(w16.b.l); 
} //.word() */
/*ce***************************************************/
event void sendDbg._uint8_3(const prog_uchar * pStr,uint8_t uniqueId,w32_4u8_u w32) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHex(w32.u8.b3); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(w32.u8.b2); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(w32.u8.b1); 
} //.word() */
/*ce***************************************************/
event void sendDbg._uint8_4(const prog_uchar * pStr,uint8_t uniqueId,w32_4u8_u w32) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHex(w32.u8.b4); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(w32.u8.b3); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(w32.u8.b2); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHex(w32.u8.b1); 
} //.word() */
/*ce***************************************************/
event void sendDbg.u16u32(const prog_uchar * pStr,uint8_t uniqueId,const uint16_t outWord,const uint32_t outLong) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHexword(outWord); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHexlong(outLong); 
} //.u16u32 */
/*ce***************************************************/
event void sendDbg.word(const prog_uchar * pStr,uint8_t uniqueId,const uint16_t outWord) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHexword(outWord); 
  
} //.word() */
/*ce***************************************************/
event void sendDbg.word_2(const prog_uchar * pStr,uint8_t uniqueId,
  const uint16_t outWordB,const uint16_t outWordA) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHexword(outWordB); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHexword(outWordA); 
  
} //.word3() */
/*ce***************************************************/
event void sendDbg.word_3(const prog_uchar * pStr,uint8_t uniqueId,
  const uint16_t outWordC,const uint16_t outWordB,const uint16_t outWordA) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHexword(outWordC);
  call PrintUtils.printChar(' ');  
  call PrintUtils.printHexword(outWordB); 
  call PrintUtils.printChar(' ');
  call PrintUtils.printHexword(outWordA); 
  
} //.word3() */
/*ce***************************************************/
event void sendDbg.word32(const prog_uchar * pStr,uint8_t uniqueId,const uint32_t outWord) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHexlong(outWord); 
  
} //.word() */
/*ce***************************************************/
//event void sendDbg.word64(const prog_uchar * pStr,uint8_t uniqueId,const uint32_t upperWord,const uint32_t lowerWord) {
event void sendDbg.word64(const prog_uchar * pStr,uint8_t uniqueId,const uint64_t outWord) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printHexword64(outWord); 
  
} //.word() */

  /*df***************************************************/
  void printPortValue(uint8_t portValue) {
    call PrintUtils.printHex(portValue);
    call PrintUtils.printChar(' ');
  }
  /*df***************************************************/
  void printSeqPin() {
    uint8_t portValue;

    call PrintUtils.printP(sPin);
    portValue = inp(PINA); printPortValue(portValue);
    portValue = inp(PINB); printPortValue(portValue);
    portValue = inp(PINC); printPortValue(portValue);
    portValue = inp(PIND); printPortValue(portValue);
    portValue = inp(PINE); printPortValue(portValue);
    portValue = inp(PINF); printPortValue(portValue);
    portValue = inp(PING); printPortValue(portValue);

  } //printSeqPin
  /*df***************************************************/
  void printSeqDdr() {
    uint8_t portValue;

    call PrintUtils.printP(sDdr);
    portValue = inp(DDRA); printPortValue(portValue);
    portValue = inp(DDRB); printPortValue(portValue);
    portValue = inp(DDRC); printPortValue(portValue);
    portValue = inp(DDRD); printPortValue(portValue);
    portValue = inp(DDRE); printPortValue(portValue);
    portValue = inp(DDRF); printPortValue(portValue);
    portValue = inp(DDRG); printPortValue(portValue);
 } //printSeqDdr
  /*df***************************************************/
  void printSeqPort() {
    uint8_t portValue;

    call PrintUtils.printP(sPort);
    portValue = inp(PORTA); printPortValue(portValue);
    portValue = inp(PORTB); printPortValue(portValue);
    portValue = inp(PORTC); printPortValue(portValue);
    portValue = inp(PORTD); printPortValue(portValue);
    portValue = inp(PORTE); printPortValue(portValue);
    portValue = inp(PORTF); printPortValue(portValue);
    portValue = inp(PORTG); printPortValue(portValue);

  } //printSeqPort
  
/*ce***************************************************/
event void sendDbg.memPort(const prog_uchar * pStr,uint8_t uniqueId) {
  
  dbgMsgHdr(pStr,uniqueId);
    call PrintUtils.printP(sPortHeader);
    printSeqPin();
    printSeqPort();
    printSeqDdr();
  
} //.memPort() */
/*ce***************************************************/
//      event void str(const prog_uchar * pStr,const uint8_t uniqueId,const prog_uchar * pStr2);
event void sendDbg.str(const prog_uchar * pStr,const uint8_t uniqueId,const prog_uchar * pStr2) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printP(pStr2);
  
} //.str() */
/*ce***************************************************/
event void sendDbg.strD(const prog_uchar * pStr,const uint8_t uniqueId,const char * pStrD) {
  
  dbgMsgHdr(pStr,uniqueId);
  call PrintUtils.printD(pStrD); 
  
} //.strD() */
/*ce***************************************************/
//      event void str(const prog_uchar * pStr,const uint8_t uniqueId,const prog_uchar * pStr2);
event void sendDbg.timeAddrData(const prog_uchar * pStr,uint16_t addr, uint16_t data) {

  dbgTime();
  call PrintUtils.printP(pStr);
  call PrintUtils.printChar(':');
  call PrintUtils.printHexword(addr);
  call PrintUtils.printChar('-');
  call PrintUtils.printHexword(data);
  
} //.str() */
#endif //#if 0
/*de***************************************************/
//inline    async event result_t Clock.fire() { return SUCCESS;    }
//} //end implementation

//sysDebugClass sysDebug;
//extern "C" {  
//	inline void SysTick_Handler(void) { osLoopClass.timerTick()}
//}//extern "C" 
