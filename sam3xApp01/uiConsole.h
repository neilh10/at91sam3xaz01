/*  uiConsole.h user facing functions through a console library
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
#ifndef uiConsole_h
#define uiConsole_h

typedef enum {
	eSexi=0,//Status exit
	eSstd=1,//Standard read
	eScfg=2,//Standard config read and write
	eSadm=3,//Admin level
	eSdbg=4,//Debug
	//spare 5..7
	eShlp=0x8,//Help only, bit mapped
	eSstdhlp=eShlp|eSstd,
eScfghlp=eShlp|eScfg,
eSadmhlp=eShlp|eSadm,
eSdbghlp=eShlp|eSdbg,
  } eciStatus;
	
#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

	typedef enum {
	eCiKeySizeMax=5,
	} eConsoleInput;


  typedef struct {
	const eciStatus status;
    const char *pKey;
    const uint8_t keyUsed;
	//void (*uiFn)(void);
	//void (*uiFn)(String *pInConsole, uint8_t pos);
    const result_t (*uiFn)(char *pConsoleIn,uint8_t pos,uint8_t numCharsRx);
	const char *pHelp;
  } consoleInput_t;
  
#ifdef __cplusplus
}
#endif // __cplusplus


class uiConsoleClass  {
  public:
    uiConsoleClass(void);
    void serialEvent(void);
	bool serialAvailable(void);
    void adcCmd(void);
	uint16_t delayValue;


  private:
    const consoleInput_t *pConsoleInput;

};

extern uiConsoleClass uiConsole;

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

const result_t uiConsole_debugCmd(char *pConsoleIn, uint8_t pos,uint8_t inConPos);
const result_t uiConsole_pwrCmd(char *pConsoleIn,uint8_t pos,uint8_t inConPos);


#ifdef __cplusplus
}
#endif	

#endif//uiConsole_h