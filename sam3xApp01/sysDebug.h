/* sysDebug.h sysDebug library
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
 * Class sysDebug interface
 *
 * */
#ifndef sysDebug_h
#define sysDebug_h

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

	typedef enum {
		MASK_INDX_SystemMngXxx, //0 SystemMng SystemMngStartup
		MASK_INDX_DataMngM,//1
		MASK_INDX_SensorsMngM, //2 SensorsMngM
		MASK_INDX_SensorsMngXxx,//3 SensorsMngPrsrM SensorsMngCondM
		MASK_INDX_SensorsAdcHwM, //4 RadioSendM
		MASK_INDX_RadioLinkLayer,    //5 RadioLinkLayer
		MASK_INDX_RadioSend,  //6 RadioSendM
		MASK_INDX_RouteInM,  //7 routineInM
		MASK_INDX_HALHwStateSondeM,  //8
		MASK_INDX_FlashSerial,  //9
		//  MASK_INDX_HALTwiPerphlM,
		MASK_INDX_SIZE,
		//The size of this is also used in sonde_gen.h
		MASK_INDX_NOP=0xff,
		MASK_INDX_DEF_VALUE=0xff //usually all off 0x00 or all on 0xff
	} eMaskIndx;

#ifdef __cplusplus
}
#endif // __cplusplus

class SerconClass : public Print {
	public:
	SerconClass(void);
	 size_t print3(unsigned int, int = DEC);
	};


class sysDebugClass  {
  public:
    sysDebugClass(void);
	void hdr(char const *pStr,char uniqueId);
	void out1(const char *pStr,char uniqueId,uint32_t param1);
    void dbgTime(void);
	uint16_t rdDbgMask(uint8_t moduleId);

  private:
   uint16_t moduleAcc[MASK_INDX_SIZE];
    
};

extern sysDebugClass sysDebug;

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus


typedef enum { //mask 
//Guideline: module outputs should be 0x01, startup 0x80
// see eMaskIndx/DebugSubSysMasksM for offsets
  eDbgAll_errEvt =0x01,
  //0 SystemMngXxx HPLClock TimerM MASK_INDX_SystemMngXxx
  eDbgSysMngXxx_Startup=0x02,
  eDbgSysMngXxx_startUpEvt=0x02,
  eDbgSysMngXxx_Evt=0x04,
  eDbgHalClock_Evt  =0x08,

  eDbgSysMngXxx_PwrMin=0x10,  
  eDbgHplClock_Evt  =0x20,
  eDbgHplWatchdog   =0x20,
  eDbgPerStorXxx_Evt=0x40,
  eDbgConsoleAccess =0x80,
  
  //1 SensorsDataMngM MASK_INDX_DataMngM
  eDbgDataMng_Startup=0x02,
  eDbgDataMng_sdmsAction =0x04,
  eDbgDataMng_Fifo=0x08,

  eDbgDataStoreMng_FlyTime = 0x10,
  eDbgDataStoreMng_Time = 0x20,
  eDbgDataStoreMng_Msg = 0x40,
  //DataStoreM
  //eDbgSnsrDataStore_Output=0x01,
  //2 SensorsMngM
  eDbgSnsrMng_Io=0x02,
  eDbgSnsrMng_DebugEvt =0x04,
  eDbgSnsrMng_StateChng=0x08,
  eDbgSnsrMng_Output   =0x10,
  eDbgSnsrMng_Results  =0x20,
  eDbgSnsrMng_FilterOut = 0x40,
  eDbgSnsrMng_FilterDis = 0x080, //Turn ON filtering
  //3 SensorsMngPrsrM SensorsMngCondM
  eDbgSnrsMng_Prsr = 0x02,
  eDbgSnrsMng_PrsrX10 = 0x04, //Only use X1 data - changes processing
  eDbgSnrsMng_PrsrCalbr = 0x08,// Calibration data

  eDbgSnrsMng_CondOut   =0x10,
  eDbgSnrsMng_CondIo    =0x20,
  eDbgSnrsMng_CondDetail=0x40,
  eDbgSnrsMng_CondCalbr =0x80,
  //4 'a'SensorsAdcHwM
  eDbgSnsrAdcHw_cmd = 0x02,
  eDbgSnsrAdcHw_Evt = 0x04,
  eDbgSnsrAdcHw_Output=0x08,

  eDbgSnsrAdcHw_Twi = 0x10,
  eDbgSnsrAdcHw_Mcp = 0x20,
  eDbgSnsrTwi_input =0x40,
  eDbgSnsrTwi_start =0x80,
  //5 RadioLL RadioLink Layer
  eDbgRadio_FramerMng=0x02, //very dangerous - from int
  eDbgRadio_FramerMngTx=0x04, //very dangerous - from int
  eDbgRadio_FramerTxOctet=0x04, //very dangerous - from int
  eDbgRadio_FramerRxOctet=0x08, //very dangerous - from int
  
  eDbgRadio_RmApiMng=0x10, //Packet info from background
  eDbgRadio_RmApiTxOctet=0x20, //very dangerous - from int
  eDbgRadio_RmApiRxOctet=0x40, //very dangerous - from int
  eDbgRadio_RmApiDiscard=0x80, // unprocessed messages
  //6 'r'RadioSendM & RadioXBeeM
  eDbgRadio_Cntl=0x02,
  eDbgRadio_Send=0x04,
  eDbgRadio_ATrsp=0x08,
  //eDbgRadio_ATrspND=0x10, ??
  
  //7 RouteIn (Rx&Tx)
  eDbgRouteIn_RxAll=0x02, //RxAll messages
  eDbgRouteIn_RxRadio=0x04,
  eDbgRouteIn_TxAll=0x08, //TxAll messages
  eDbgRouteIn_DefaultEvent=0x10,
  //8 HALHwStateSonde
  eDbgHalPwr=0x02,
  eDbgHalPwrRadio=0x04,
  eDbgHalPwrAnlgMaster=0x08,
  eDbgHalPwrAnlg=0x10,
  eDbgHalPwrRecBat=0x20,
  eDbgHalMemPort=0x40,
  eDbgPwrMng=0x80,
  //9 FLASH EEPROMPage
  eDbgFlsh_Mng   =0x02, //Manage
  eDbgFlsh_StorIo=0x04, //Input/Output
  eDbgFlsh_IoInit=0x08, //Entry points
  eDbgFlsh_req=0x10,
  eDbgFlsh_execCmd=0x20,
  eDbgFlsh_done   =0x40,
  eDbgFlsh_bytes  =0x80, //!! careful
  //***
  eDbgConsole,
  eDbgEnumsEnd,
  //eDbg_DEBUG_CONSOLE - in files as doesn't work here
} eDbgEnums;



#define dbgOut(mask,uniqueId,strIn) \
  if (mask & sysDebug.rdDbgMask(cOffsetDbgMask)) sysDebug.hdr(strIn,uniqueId)
#define dbgOut1(mask,uniqueId,strIn,param1)\
  if (mask & sysDebug.rdDbgMask(cOffsetDbgMask)) sysDebug.out1(strIn,uniqueId,param1)
 /*
#define dbg_Uint8_(mask,uniqueId,strIn, out8bits) \
if (mask & sysDebug.rd()) \
sysDebug._uint8_(strIn,uniqueId,(uint8_t)out8bits)
#define dbg_Uint8_G(dbgGate,mask,uniqueId,strIn, out8bits) \
if (dbgGate && (mask & sysDebug.rd())) \
sysDebug._uint8_(strIn,uniqueId,(uint8_t)out8bits)
#define dbgWord_2(mask,uniqueId,strIn, out16bitsB,out16bitsA) \
if (mask & sysDebug.rd()) \
sysDebug.word_2(strIn,uniqueId,(uint16_t)out16bitsB,(uint16_t)out16bitsA)
#define dbgWord_3(mask,uniqueId,strIn, out16bitsC,out16bitsB,out16bitsA) \
if (mask & sysDebug.rd()) \
sysDebug.word_3(strIn,uniqueId,(uint16_t)out16bitsC,(uint16_t)out16bitsB,(uint16_t)out16bitsA)
#define dbgWord(mask,uniqueId,strIn, out16bits) \
if (mask & sysDebug.rd()) \
sysDebug.word(strIn,uniqueId,(uint16_t)out16bits)
#define dbgWordG(dbgGate,mask,uniqueId,strIn, out16bits) \
if (dbgGate && (mask & sysDebug.rd())) \
sysDebug.word(strIn,uniqueId,(uint16_t)out16bits)
#define dbgWord32(mask,uniqueId,strIn, out32bits) \
if (mask & sysDebug.rd()) \
sysDebug.word32(strIn,uniqueId,(uint32_t)out32bits)
#define dbgWord64(mask,uniqueId,strIn, out64bits) \
if (mask & sysDebug.rd()) \
sysDebug.word64(strIn,uniqueId,(uint64_t)out64bits)
#define dbg_u8u16(mask,uniqueId,strIn, prm2,prm1) \
if (mask & sysDebug.rd()){\
sysDebug.u8u16(strIn,uniqueId,prm2,prm1);\
    }
#define dbg_u8u16u32(mask,uniqueId,strIn, prm3,prm2,prm1) \
    if (mask & sysDebug.rd()){\
      sysDebug.u8u16u32(strIn,uniqueId,prm3,prm2,prm1);\
    }
#define dbg_u8u32(mask,uniqueId,strIn, prm2,prm1) \
    if (mask & sysDebug.rd()){\
      sysDebug.u8u32(strIn,uniqueId,prm2,prm1);\
    }
#define dbg_Uint8_2(mask,uniqueId,strIn, prm2,prm1) \
    if (mask & sysDebug.rd()){\
      uByteWrd w16; \
      w16.b.l = prm1;\
      w16.b.m = prm2;\
      sysDebug._uint8_2(strIn,uniqueId,w16);\
    }
#define dbg_Uint8_3(mask,uniqueId,strIn, prm3,prm2,prm1) \
    if (mask & sysDebug.rd()){\
      w32_4u8_u w32; \
      w32.u8.b1 = prm1;\
      w32.u8.b2 = prm2;\
      w32.u8.b3 = prm3;\
      sysDebug._uint8_3(strIn,uniqueId,w32);\
    }
#define dbg_Uint8_4(mask,uniqueId,strIn, prm4,prm3,prm2,prm1) \
    if (mask & sysDebug.rd()){\
      w32_4u8_u w32; \
      w32.u8.b1 = prm1;\
      w32.u8.b2 = prm2;\
      w32.u8.b3 = prm3;\
      w32.u8.b4 = prm4;\
      sysDebug._uint8_4(strIn,uniqueId,w32);\
    }
#define dbg_Uint16_2(mask,uniqueId,strIn, prm2,prm1) \
    if (mask & sysDebug.rd()){\
      w32Two_u16_u w32; \
      w32.two_u16.l = prm1;\
      w32.two_u16.m = prm2;\
      sysDebug.word32(strIn,uniqueId,(uint32_t)w32.w32);\
    }
#define dbg_u16u32(mask,uniqueId,strIn, prm2,prm1) \
    if (mask & sysDebug.rd()){\
      sysDebug.u16u32(strIn,uniqueId,prm2,prm1);\
    }

#define dbgBuffer(mask,uniqueId,strIn,ptr,num)\
    if (mask & sysDebug.rd()) \
      sysDebug.bufGen(strIn,uniqueId,ptr,num)
#define dbgSnsrReadings(mask,uniqueId,strIn, ptr) \
    if (mask & sysDebug.rd()) \
      sysDebug.sensorReadings(strIn,uniqueId,ptr)
#define dbgLogAdc(mask,uniqueId,strIn, ptr) \
    if (mask & sysDebug.rd()) \
      sysDebug.sensorsLogAdc(strIn,uniqueId,ptr)
#define dbgHalLogger(mask,uniqueId,strIn, ptr,recLen) \
    if (mask & sysDebug.rd()) \
      sysDebug.halLogger(strIn,uniqueId,ptr,recLen)
#define dbgTxStatusDm8B(mask,uniqueId,strIn, pMsg) \
    if (mask & sysDebug.rd()) \
      sysDebug.txStatusDm8B(strIn,uniqueId,pMsg)
#define dbgTosMhopMsg(mask,strIn,uniqueId, ptr) \
    if (mask & sysDebug.rd()) \
    sysDebug.tosMhopMsg(strIn,uniqueId,ptr)
#define dbgTosMsg(mask,strIn,uniqueId, ptr) \
    if (mask & sysDebug.rd()) \
    sysDebug.tosSMsg(strIn,uniqueId,ptr)


#define dbgMemPort(mask,uniqueId,strIn)\
    if (mask & sysDebug.rd()) \
      sysDebug.memPort(strIn,uniqueId)
*/
#define errEvt(uniqueId,strIn) \
  if (eDbgAll_errEvt & sysDebug.rdDbgMask(cOffsetDbgMask)) \
      sysDebug.hdr(strIn,uniqueId);

#define errEvt1(uniqueId,strIn,parm1) \
    if (eDbgAll_errEvt & sysDebug.rdDbgMask(cOffsetDbgMask)) \
      sysDebug.out1(strIn,parm1);
/*
#define errEvtUint8(uniqueId,strId,uint8) \
    if (eDbgAll_errEvt & sysDebug.rd()) \
      sysDebug._uint8_(strId,uniqueId,uint8);
#define errEvt2Uint8(strId,uniqueId,callerId,clientId) \
    if (eDbgAll_errEvt & sysDebug.rd()) \
      sysDebug._2uint8_(strId,uniqueId,callerId,clientId);
#define errEvtWord16(uniqueId,strId,wrd16) \
    if (eDbgAll_errEvt & sysDebug.rd()) \
      sysDebug.word(strId,uniqueId,wrd16);
*/
#ifdef __cplusplus
}
#endif	

#endif//sysDebug_h