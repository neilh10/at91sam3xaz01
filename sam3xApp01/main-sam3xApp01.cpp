/*
 * sam3xApp01.cpp
 * Copyright (c)2012 Azonde, All Rights Reserverd.
 
 * Created: 10/25/2012 5:45:04 PM
 *  Author: neil
 *
 *For twi see sam3x_ek2\..\AT24CXX_EXAMPLE1\src\asf\sam\twi\twi.
 *
 *  
 */ 
#include "sam.h"
#define ARDUINO_MAIN
#include "Arduino.h"
#include "az23project.h"
#include "osLoop.h"
//#include "HardwareSerial.h"
#include "sysDebug.h"
#define cOffsetDbgMask MASK_INDX_SystemMngXxx
#include "uiConsole.h"

#include "HalTwiPca9698.h"
#include "rstc.h"


extern "C" {
//inline void TimeSysTick_Handler(void) { osLoop.timerTick();}

#define STREOL "\n\r"
#define PROJECT_DESC "-- Azonde Beta --" \
"-- Compiled: "__DATE__" "__TIME__" --\n\r"STREOL	

}//extern "C" 

//#define USBHOST
//#define USBHOSTKEYBOARD
#ifdef USBHOSTKEYBOARD
#include <KeyboardController.h>// Require keyboard control library
USBHost usbhKeyboard;// Initialize USB Controller
KeyboardController keyboard(usbhKeyboard);// Attach keyboard controller to USB
#endif //USBHOSTKEYBOARD
#ifdef USBHOST

#include <Usb.h>
USBHost usbh;
#endif
//#define LEDS_CPU

extern "C" {
uint32_t timerTick_10mS;
//how about hardware/arduino/sam/cores/arduino/hooks.c - sysTickHook()  return 0 to have Arduino's handler run afterwards
extern void TimeTick10mS(void) //hook in to system/libsam/source/timetick.c
{
  osLoop.timerTick();
  if (++timerTick_10mS >100) {
	  //1 second list
	  timerTick_10mS = 0;
	  HalExpansionPort.timerTick();
  }
}
   }	   
// Pin 13 is often a LED connected (variant.cpp) but not with Az23
// Use the descriptive name:
//int ledYellow = PIN_LED
int led2 = PIN_LED_RXL; // PIN_LED_TXL
uint16_t count=0;


 String strHelpSys = "Help String\n\r0 0.25Sec\n\r1 1sec\n\r";
// the setup routine runs once when you press reset:

/**
 * \brief SetupUserIf .
 * The LEDs and Serial interface
 *
 * \return none
 */
void setupUserIf() {
	int waitLp=0;
	
	//PowerOutput
	
	//RS232 output
	
	// initialize the digital pin as an output.
	pinMode(PIN_LED_CPURED, OUTPUT);
	pinMode(led2, OUTPUT);
	Serial.begin(115200);
	while (!Serial && waitLp<10) {
		delay(1000);
		waitLp++;
	}
	//Serial.println("Serial0:Blink_count <date>");
	//Serial.println(strHelpSys);
	#ifdef _SerialUsbDef_
	SerialUSB.begin(115200);
	while (!SerialUSB && waitLp<15) {
		delay(1000);
		waitLp++;
	}
	//SerialUSB.println("USB:Blink_count <date>");
	#endif //_SerialUsbDef_	
}


/**********************************************
 * \brief 

 *
 * \return none
 */// Outputs a string to all ports:
void printOut(const char *pOutStr) {
	Serial.println(pOutStr);

	#ifdef _SerialUsbDef_
	SerialUSB.println(pOutStr);
	#endif // _SerialUsbDef_
}

/**********************************************
 * \brief 

 *
 * \return none
 */
// the loop routine runs over and over again forever:
void loopUserIf() {
	int readChar1,readCharU=-1;
	//sysDebug.dbgTime();

   // dbgOut1(eDbgAll_errEvt,'A',"loopUserIf",(uint16_t)++count);
	//Serial.print(++count,DEC);
	
	digitalWrite(PIN_LED_CPURED, HIGH);   // turn the LED on (HIGH is the voltage level)
#ifdef LEDS_CPU	
	digitalWrite(led2, LOW);   //
#endif // LEDS_CPU	
	//digitalWrite(HwPwrIn30VLdoEnN,LOW);
	delay(uiConsole.delayValue);  // wait for a second
#ifdef LEDS_CPU	
	digitalWrite(PIN_LED_CPURED, LOW);    // turn the LED off by making the voltage LOW
#endif //LEDS_CPU
	digitalWrite(led2, HIGH);   //
    digitalWrite(HwPwrIn30VLdoEnN,HIGH);
	//HalExpansionPort.ledBitWalk();
	delay(uiConsole.delayValue);  // wait for a second	
	
	//return 1000;
}

/**********************************************
 * \brief 
 *
 *
 * \return none
 */
void loopTest1() {Serial.println(" loopTest1");}
void loopTest2() {Serial.println(" loopTest2");}
void loopTest3() {Serial.println(" loopTest3");}
void loopTest4() {Serial.println(" loopTest4");}
typedef enum {
	eOlr_loopUserIf,
	eOlr_loopTest1,
	eOlr_loopTest2,
	eOlr_loopTest3,
	eOlr_loopTest4,
	eOLr_numLoops
	//implicit eOsNoAction = 0xfe
} eOsLoopRef;

const osLoopRef_t osLoopDefs[eOLr_numLoops+1]= {
//Controlling table of loopFn interfaces. Setup called seperately
//Timer_10mS  loopFn
	{ 10,loopUserIf,setupUserIf	},
	{  0,loopTest1,NULL	},
	{  0,loopTest2,NULL	},
	{  0,loopTest3,NULL	},
	{  0,NULL,NULL	}//End row
	};//osLoopDefs

//*********************************************************
// the loop routine runs over and over again forever:

/**
 * \brief Entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
uint resetCause = rstc_get_reset_cause(RSTC);
    /* Initialize the SAM system */
	init();//variant.cpp: which does SystemInit() & arduino init
    //WDT->WDT_MR = WDT_MR_WDDIS;
	//Power Supply
  initVirtualPin(HwPwrIn30VLdoEnN);
  digitalWrite(HwPwrIn30VLdoEnN,HIGH);  
  
  uint8_t resetType=(resetCause>>RSTC_SR_RSTTYP_Pos);

  #define RSTC_NRSTL_msk 0x10000
  //if (0 == (resetCause&RSTC_NRSTL_msk)) {puts(" Serious Hw Error NRTSL low");}
  rstc_reset_extern(RSTC);//Ensure peripherals reset - TwiPort, EthernetMod, Xbp,MtCell, Arduino
	
  delay(100); //mS

    HilInitAzPorts(); //requires
    setupUserIf();// Baud out from here
	//delay(100); //mS
			
  //printf("%s",PROJECT_DESC);
  Serial.print(PROJECT_DESC);
  Serial.print("ResetCause ");
  switch (0x07 & resetType) {
	  case 0: Serial.print("PwrUp");break;
	  case 1: Serial.print("Backup");break;
	  case 2: Serial.print("Watchdog");break;
	  case 3: Serial.print("Software "); break;
	  case 4: Serial.print("User");break;
	  default:  Serial.print("Unknown "); Serial.print(resetType);break;
  }

    HilInitPeripherals();
	
	//0x20 Sets to 1mS increment, life of 1193hr/49days
	//0x8000 sets to 1Sec increment.
	RTT_SetPrescaler(RTT,0x20);
	//sysDebug.dbgTime();

	#if defined(USBCON)
	USBDevice.attach();
	#endif
	    
	osLoop.setupLoops(osLoopDefs,eOLr_numLoops);
	osLoop.scheduleLoop(eOlr_loopUserIf,0);
	osLoop.scheduleLoop(eOlr_loopTest3,0);
	osLoop.scheduleLoop(eOlr_loopTest2,0);
	osLoop.scheduleLoop(eOlr_loopTest3,0);
	osLoop.scheduleLoop(eOlr_loopTest1,0);

	//osLoop.scheduleLoop(eOlr_loopUserIf);
	//do {
	//		Serial.print("nextLoopIs: ");
	//} while (true == osLoop.runNextLoop()); 

	//Need to do System Startup FSM
	//osLoop.schedulerStart();
		for (;;)
		{
			loopUserIf();
			//if (serialEventRun) uiConsole.serialEvent();//simple polled
			if (Serial.available()) uiConsole.serialEvent();//simple polled
			#ifdef USBHOST
			usbh.Task();
			#endif
			#ifdef USBHOSTKEYBOARD
			usbhKeyboard.Task();
			#endif //USBHOSTKEYBOARD
		}
	

}//main()









