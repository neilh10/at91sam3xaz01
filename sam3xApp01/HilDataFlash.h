/* HilDataFlash.h HilDataFlash library
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
 * Class HilDataFlash to interface with internal DataFlash
 *
 * */
#ifndef HilDataFlash_h
#define HilDataFlash_h



class HilDataFlashClass  {
  public:
    HilDataFlashClass(void);
    result_t readWord64bits(uint32_t addr,uint64_t *pValue);
    result_t readBuf(uint32_t addr, uint16_t size, void *pBuf);
    result_t writeWord64Req(uint32_t addr, uint16_t data,uint8_t idCaller);
    void writeDoneEvt(result_t result); //At interrupt
    result_t writeBufReq(uint32_t addrEeprom, uint16_t sizeToWr, void *pBuf,uint8_t id);
    result_t resetBytesReq(uint32_t addrEeprom, uint16_t sizeToReset, uint64_t resetValue,uint8_t id);
    void writeTask(); //can it be done by friends in private
    void resetBytesTask();

  private:
      // Bit flags in EEPROM control register
/*  #define _EERIE 0x08
  #define _EEMWE 0x04
  #define _EEWE  0x02
  #define _EERE  0x01
*/
  uint64_t *pWrFrmBuf; // Buffer to read data from 
  uint16_t wrNextEepromAddr;//Next address
  int16_t wrBytesLeft;
  uint16_t resetBytesNextEepromAddr;//Next address
  int16_t resetBytesLeft;
  uint8_t resetBytesValue;
  uint8_t idHALEepromCaller;
};

extern HilDataFlashClass HilDataFlash;
#endif//HilDataFlash_h