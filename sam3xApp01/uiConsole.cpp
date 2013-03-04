/*
 * uiConsole.cpp
 * Copyright (c)2012 Azonde, All Rights Reserverd.
 
 * Created: 10/25/2012 5:45:04 PM
 *  Author: neil
 *
 *
 *  Console
 *  Process a serial input and when LF recived inspect commands string
 *  Serial input could come from a number of sources,
 *  User over UART/USART,Rs232, RS485 ,USB, telnet
 *  or MT-cell interface 
 *  
 *  Commmands are constructured in a table and look for parms
 *  User Mode
 *  
 *  Debug Mode
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
 *  Ideas
 *  Establish a CStringList of keywords and each time you add to your current buffer,
 check the buffer against the elements of CStringList using the CStringList::Find() function.
 If you find a keyword, then begin a new string until you reach the "***" sequence
Also need role based commands
 https://github.com/insanum/gcalcli python mod for accessing google calendar
 http://www.soltech-solutions.com/cli-commands/cisco-cli-commands.html 
 */ 

#include "sam.h"
#include "Arduino.h"
#include "az23project.h"
#include "sysDebug.h"
#define cOffsetDbgMask MASK_INDX_SystemMngXxx
#include "uiConsole.h"
#include "HalTwiPca9698.h"

#ifdef __cplusplus
extern "C" {
#endif
void uiConsoleParse(void);


#ifdef __cplusplus
}
#endif

//String strInputConsole = "";         // console buffer
#define cInputConsoleSize 80
char inputConsole[cInputConsoleSize];
uint8_t inConPos; //Position for next character in console
//boolean inputConsoleComplete = false;  // true when ready for parsing
uiConsoleClass uiConsole;

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
uiConsoleClass::uiConsoleClass(void) {
   delayValue=250;
   //if (0==strInputConsole.reserve(100)) {Serial.print("malloc fail");}
}

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
inline bool uiConsoleClass::serialAvailable(void) {
	//FUT serial1.available() or
	return Serial.available();
}	

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
void uiConsoleClass::serialEvent(void) {//Fut:Should run at interrupt

   while (serialAvailable()) {
    // get the new byte:
    char readChar1 = (char)Serial.read();
	char readCharU = -1;
	#ifdef _SerialUsbDef_
	//SerialUSB.println(count,DEC);
	readCharU = SerialUSB.read();
	#endif  //_SerialUsbDef_		
		
	if ((-1 !=  readChar1) || (-1 != readCharU)) {			
      if (-1==readChar1) readChar1=readCharU;
         
		inputConsole[inConPos++]=readChar1;
		if (inConPos>=cInputConsoleSize-1) {inConPos=cInputConsoleSize-2;Serial.print("serialEvent fail");break;}
		if ('\n'==readChar1 || '\r'==readChar1 || '\t'==readChar1) {
		  inputConsole[inConPos+1]=0;//Str termination
		  uiConsoleParse(); //Parse input FUT: post to parse input at background
		  inConPos=0;//FUT: revisit as could cause a race.?
		}		
	}
  }
   }  
//void uiConsoleClass::adcCmd(void) {
//	
//}
const void (*uiFn)(char *pConsoleIn,uint8_t pos);

#ifdef __cplusplus
extern "C" {
#endif


/**********************************************
 * \brief 
 *
 *
 * \return none
 */
const result_t uiConsole_adcCmd(char *pConsoleIn,uint8_t pos,uint8_t inConPos) {
	result_t retResult=SUCCESS;
	//uiConsole.adcCmd();
	//String inputString( *pConsoleIn);         // a string to hold incoming data
	//dbgOut1(eDbgAll_errEvt,22,"uiConsole_adc",inConPos);


		uint8_t inChar = *pConsoleIn;
		switch (inChar) {
		case '0': uiConsole.delayValue=250; Serial.print("set to 0.25secs");break;
		case '1': uiConsole.delayValue=500; Serial.print("set to 1sec");break;
		case '2': uiConsole.delayValue=1000; Serial.print("set to 2sec");break;
		//case 'p':
		default:
		Serial.print("Invalid Input ");
		Serial.println(inChar,HEX); 
		retResult=FAIL;
		break;
	}
 return retResult;
}

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
const result_t uiConsole_b123Cmd(char *pConsoleIn,uint8_t pos,uint8_t inConPos) {
	Serial.println("uiConsole_b123Cmd - low powering");
	
	
	return FAIL;
}


/**********************************************
 * \brief 
 *
 *
 * \return none
 */
const result_t uiConsole_ledsCmd(char *pConsoleIn,uint8_t pos,uint8_t inConPos) {
/* led
      sz[z]xy 
	  0=disable 1=enable
	  s then as follows:
	   z=LedNum1-9 (with 0|a|r|g)0= off or a amber on, r red on g grn on
	   z=LedNum x - applies to all
	    y=state  f - flash    
	       [s=slow|f=fast flash"},
      
	  s[1->9[r|g]|0=off|s=step|x=all
	   [0=off|1=on|s=slow|f=fast flash
	   
	   eg
	led0 - disable all
	leds10 - off
	leds1a - ammber/both on 

*/
	result_t retResult=FAIL;
    uint8_t ledNumber, ledColour;
	result_t parseStatus = FAIL;
	
	Serial.println("uiConsole_ledsCmd");
	
        switch (*pConsoleIn) {
	        //case '1': result = LedsMngLocal.enable(0); break;
	        //case '0': result = LedsMngLocal.disable(0); break;
			default:
			  Serial.print("unknown option=");
			  Serial.print(*pConsoleIn);
			  break;
			case 't': //led't' for test [0|1]
				pConsoleIn++;
				if ('1' == *pConsoleIn) {
				  Serial.println(" ledTestStarted");
				  HalExpansionPort.ledTestStart();
				}else{
				  Serial.println(" ledTestStopped");
				  HalExpansionPort.ledTestStop();
				}		
				retResult = parseStatus = SUCCESS;			 
				break;
	        case 's': //led's'
			//look for next
			  pConsoleIn++;
			  ledNumber=((uint8_t)*pConsoleIn)-0;
			  if (ledNumber<10) {
				//found a led number, look for color a,r,g
				pConsoleIn++;
				parseStatus=SUCCESS;
				switch(*pConsoleIn) {
				  case 'a': ledColour=0x03;break;
				  case 'r': ledColour=0x02;break;
				  case 'g': ledColour=0x01;break;
				  case '0': ledColour=0;break;//Off
				  default: retResult=FAIL;parseStatus=FAIL;break;
				}				
			  }else {//not a number
			    switch (*pConsoleIn) {
			      default: break; 
				  case 'x':
				  break;
			    }
			  }
			  break; //case s
        }
 return retResult;
}

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
const consoleInput_t consoleInput[] = {
//{eSstd,"adc",3,uiConsole_adcCmd," [0|E] analog number\n\r  [a|z] series adc"},
{eSstd,"b123",4,uiConsole_b123Cmd,"Help field"},
{eSstd,"pwr",3,uiConsole_pwrCmd," pwr[c|b|s] c[1-8|a-f] b[f|s] l[s|w|b]] low power opts"},
{eSstdhlp,"",0,NULL,"Separate help info"},
{eSdbg,"d",1,uiConsole_debugCmd,"zxx z=offset x=hex value"},
{eSdbg,"led",3,uiConsole_ledsCmd,
	"tx test =1 start 0 stop walking bit\n\r"\
	"szzx s=0disable LEDs|1enable LEDs|s as follows:\n\r"\
	"z=LedNum1-9 with r|g,S=step,X=all,0=all off   x[0=off|1=on|S=slow|F=fast flash"},	

//	{"adc",3,uiConsole.adcCmd&},
//{"",0,NULL},
//	#endif
{eSexi,"",0,NULL,NULL}
};

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
void uiConsoleHelp(void) {
	const consoleInput_t *ciParser = consoleInput;
	//bool found=false;

	Serial.println("available cmds: ");	
	while (eSexi != ciParser->status) {
		Serial.print(ciParser->pKey);
		Serial.print(" ");
		Serial.println(ciParser->pHelp);
		ciParser++;
	}
}	
/**********************************************
 * \brief 
 *
 *
 * \return none
 */	
void uiConsoleParse(void) {
	uint8_t inCharOffset =0;
	char inCharVal;
	const consoleInput_t *pCiParser = consoleInput;
	bool found=false;
	uint8_t prsLp; 
	//uint8_t uiNumCharsRx = inConPos;
	//Parse 
	for (prsLp=0;((eSexi != pCiParser->status) && (false==found));pCiParser++,prsLp++) {
		const char *pKey =pCiParser->pKey;
		char key;
		const char keyUsed =pCiParser->keyUsed;

		if (keyUsed > 10) {
			//dbgOut1(eDbgAll_errEvt,22,"uiConsoleParse keyUsed to high",keyUsed);
			break;
		}
		for (uint8_t keyLp=0; ((keyLp<keyUsed) && (false==found));keyLp++){
			if (0==keyUsed) continue;
		  key = pKey[keyLp];
		  //inCharVal = strInputConsole.charAt(inCharOffset+keyLp);
		  inCharVal = inputConsole[inCharOffset+keyLp];
		  #if 0
		  Serial.print("Trying-");
		  Serial.print(inCharVal);
		  Serial.print(':');
		  Serial.println(key);
		  #endif
	      if ('?' == inCharVal) break;
		  if (key == inCharVal) {
			if (keyLp == (keyUsed-1)) {
				char *pInCon = &inputConsole[inCharOffset+keyLp+1];
			  found=true; //A match
			  if (SUCCESS != (*(pCiParser->uiFn))(pInCon,0,inConPos-(inCharOffset+keyLp))){
				  Serial.println("Usage:");
				  Serial.print(pCiParser->pHelp);
			  }
			  
			}			  
		  }
		}
	}
	if (!found) {
		Serial.print("Can't parse ");
		//Serial.println(strInputConsole);
		Serial.println(inputConsole);
		uiConsoleHelp();
	}	
	//strInputConsole=0;
	inputConsole[0]=0;
}
 
//const int adcTemp =3;


#ifdef __cplusplus
}
#endif