/* $Id:  HilDataFlash.cpp $
 * Copyright (c) 2012 Azonde Neil Hancock
 * 
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL BIOMONITORS BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF
 * BIOMONITORS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * HAL Data Flash for
 * SAM3X - 4K bytes
 * This interfaces through to the internal Flash
 * See Atmel ASF example flash_prgram_example.c from
 * The top page of 16K bytes is assigned to the flash (different X8 /
 * X4), determined by the interface.
 * It can only be written once before it needs to be erased.
 * A rolling buffer of 2K is used, increment upwards. The memory is
 * read from the top down to find the latest buffer. When the end is
 * reached the whole buffer is erased, and rewritten.
 * (FUT a copy made to serial flash?)
 * 
 * Default read 128bit access. (optional lower power 64-bit access,
 * Primitives
 * lockBits 32
 * flash desriptor
 * erasing
 * locking
 * unlocking
 *
 * EEFC0 and EEFC1
 * - unlock page
 * - Erase Page and write page then lock
 * - Get Flash descriptor
 * - min write size 32bits
 * Write buffer  equal page size
 * 
 */
#include "sam.h"
#include "Arduino.h"
#include "az23project.h"
#include "UARTClass.h"
#include "sysDebug.h"
#define cOffsetDbgMask MASK_INDX_SystemMngXxx
#include "HilDataFlash.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#ifdef __cplusplus
}
#endif // __cplusplus

HilDataFlashClass HilDataFlash;  
/*dc***************************************************/
  HilDataFlashClass::HilDataFlashClass(void) {
    pWrFrmBuf=0;
    resetBytesLeft =  wrBytesLeft=0;
    //return 1; // 1 is good... only for INIT
    //call HPLEepromSync.init();
   }

  
/*dc*D9C9T9U9**************************************************/
  inline result_t HilDataFlashClass::readWord64bits(uint32_t addr,uint64_t *pValue) {
    //This is a convenience call - read is immediate
    return FAIL;//call HPLEepromSync.read(addr,pValue);
  }
  /*dc*D9C9T9U9**************************************************/
  /*
   command result_t HilDataFlashClass::readUint16(uint16_t addr,uint16_t *pValue) {
     //This is a convenience call - read is immediate
     result_t retResult;
     uint8_t *pChar=(uint8_t *)pValue;
     
     if (SUCCESS == (retResult = call HPLEepromSync.read(addr,pChar))) {
       retResult = call HPLEepromSync.read(addr+1,(pChar+1));
     }
     return retResult;
  }*/
/*dc*D9C9T9U9**************************************************/
result_t HilDataFlashClass::readBuf(uint32_t addr, uint16_t size, void *pBufIn) {
  result_t retResult=SUCCESS;
  int16_t sizeLp;
  uint64_t *pBuf = (uint64_t *)pBufIn; 
//  uint8_t readValue;
    
  if( 0!=pBuf ) {
    if (0 != size) {
      // addr checked in .read() 
      retResult=SUCCESS;
      for(sizeLp=0; sizeLp < size; sizeLp++,addr++,pBuf++ ) {
        //if(FAIL==call HPLEepromSync.read(addr,pBuf)) njh
          {
          retResult = FAIL;
          break;
        }  
      }
    } else errEvt('B',"HilDataFlashRdErr Size");
  } else errEvt('A',"HilDataFlashRdErr Buff");
  return retResult;
}

/*dc*D9C9T9U9**************************************************/
result_t HilDataFlashClass::writeWord64Req(uint32_t addr, uint16_t data,uint8_t idCaller) {
  pWrFrmBuf = 0; //leave an indication to event HLPEeprom.writeDone[idEepromCaller]()
  wrBytesLeft = 0;
  idHALEepromCaller = idCaller;
  return FAIL; //njh call HPLEepromAsync.writeReq(addr,data);
}
/*dc*D9C9T9U9**************************************************/
 void HilDataFlashClass::writeDoneEvt(result_t result) { //event
//When a byte write is complete - route it to appropiate next event
// based on whichever one is active
// resetBytesLeft --> resetBytesTask()
// wrBytesLeft --> writeTask()
// if fail result signal done as well.
  if (((0!=wrBytesLeft) & (0 != pWrFrmBuf)) || (0 != resetBytesLeft)) {
    if (0 != resetBytesLeft) {
      if (SUCCESS == result) {
        //njh post resetBytesTask();
      } else {
        //njh signal HALEeprom.resetBytesDone[idHALEepromCaller](result);
      }
    } else {
      if (SUCCESS == result) {
        //njh post writeTask();
      } else {
        //njh signal HALEeprom.writeBufDone[idHALEepromCaller](result,pWrFrmBuf);
      }
    }
  } else {
    //njh signal HALEeprom.writeByteDone[idHALEepromCaller](result);
  }
  return;
}

/*dc*D9C9T9U9**************************************************/
result_t HilDataFlashClass::writeBufReq(uint32_t addrEeprom, uint16_t sizeToWr, void *pBuf,uint8_t id) {
// write the supplied buffer.
// if SUCCESS is returned then
// *pBuf is owned by this function until event Eeprom.writeBufDone()
  result_t retValue=FAIL;
  idHALEepromCaller = id;
  
  if (0!=pBuf) {
    if (sizeToWr>0) {
      if (SUCCESS == FAIL)//nh post(writeTask()))
        { //Should be generic
        wrNextEepromAddr=addrEeprom;
        wrBytesLeft = sizeToWr;
        pWrFrmBuf = (uint64_t *)pBuf;
        retValue =  SUCCESS;
      } else errEvt('D',"HilDataFlashWrErr Post"); //should be generic
    } else errEvt('D',"HilDataFlashWrErr Size");
  } else errEvt('C',"HilDataFlashWrErr Buffer");
  return retValue;
}//.writeBuf



/*dt*D9C9T9U9**************************************************/
void HilDataFlashClass::writeTask() {
// Write the next value to the HPLEeprom.
// If complete signal to user, else
//    expect event HPLEeprom.writeDone()

    // are we done?
    if( wrBytesLeft <= 0 ) {
//      signal sendDbg.timeAddrData(strSignal,wrNextEepromAddr,wrBytesLeft);
      wrBytesLeft = 0;
      //njh signal HALEeprom.writeBufDone[idHALEepromCaller](SUCCESS,--pWrFrmBuf);
    } else {
//      signal sendDbg.timeAddrData(strWr,wrNextEepromAddr,wrBytesLeft);

      //njh call HPLEepromAsync.writeReq(wrNextEepromAddr,*pWrFrmBuf);
      // advance to the next byte
      wrNextEepromAddr++;
      pWrFrmBuf++;
      wrBytesLeft--;
    }
} //writeTask
/*dc*D9C9T9U9**************************************************/
result_t HilDataFlashClass::resetBytesReq(uint32_t addrEeprom, uint16_t sizeToReset, uint64_t resetValue,uint8_t id) {
  resetBytesNextEepromAddr=addrEeprom;
  resetBytesLeft = sizeToReset;
  resetBytesValue = resetValue;
  //njh post resetBytesTask();
  return SUCCESS;
} //HALEeprom.resetBytesReq

/*dt*D9C9T9U9**************************************************/
void HilDataFlashClass::resetBytesTask() {
// Write the next value to the HPLEeprom.
// If complete signal to user, else
//    expect event HPLEeprom.writeDone()

    // are we done?
    if( resetBytesLeft <= 0 ) {
//      signal sendDbg.timeAddrData(strSignal,wrNextEepromAddr,wrBytesLeft);
      resetBytesLeft = 0;
      //njh signal HALEeprom.resetBytesDone[idHALEepromCaller](SUCCESS);
    } else {
//      signal sendDbg.timeAddrData(strWr,wrNextEepromAddr,wrBytesLeft);

      //njh call HPLEepromAsync.writeReq(resetBytesNextEepromAddr,resetBytesValue);
      // advance to the next byte
      resetBytesNextEepromAddr++;
      resetBytesLeft--;
    }
}


