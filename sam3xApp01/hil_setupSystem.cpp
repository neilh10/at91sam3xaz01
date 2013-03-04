/*
 * hil_setupSystem.cpp
 * Copyright (c)2012 Azonde, All Rights Reserved.
 
 * Created: 10/25/2012 5:45:04 PM
 *  Author: Neil Hancock
 */ 

#include "sam.h"
#include "Arduino.h"
#define AZ23_PRJ_FILE
#include "az23project.h"
#include "UARTClass.h"
#include "sysDebug.h"
#include "uiConsole.h"
#include "sysclk.h"
#include "HalTwiPca9698.h"
#include "HilDataFlash.h"
#include "osLoop.h"

//Objects
sysDebugClass sysDebug;
//HilDataFlashClass HilDataFlash;  
//uiConsoleClass uiConsole;
osLoopClass osLoop;
HalExpansionPortClass HalExpansionPort; 


#ifdef __cplusplus
extern "C" {
#endif


/**********************************************
 * \brief 

 *
 * \return none
 */void HilInitAzPorts(void) {
	//other stuff here SystemInit();
	//power supply ports
	
	
	//Enable power supply - do analysis later as to source.
	pinMode(HwPwrIn30VLdoEnN, OUTPUT);
	digitalWrite(HwPwrIn30VLdoEnN,LOW);//Ldo Enabled
	pinMode(HwPwrIn30VswShdn, OUTPUT);
	digitalWrite(HwPwrIn30VswShdn,LOW);//Switcher Enabled
	pinMode(HwPwrIn6VaEnN, OUTPUT);
	digitalWrite(HwPwrIn6VaEnN,LOW);//Ldo Enabled
	pinMode(HwPwrIn6VbEnN, OUTPUT);
	digitalWrite(HwPwrIn6VbEnN,HIGH);//Ldo disabled

}

/**********************************************
 * \brief Initialize Peripherals - TWI
 *
 * \return none
 */
void HilInitPeripherals() {
   

  initVirtualPin(HwSdaLTwi1);
  initVirtualPin(HwSclLTwi1); //Program the controller to dedicate TWD/TCK

  //uint32_t sysClk_hz sysclk_get_cpu_hz(); - future
  //  printf("SysClk %d HZ",sysClk_hz);
  sysclk_enable_peripheral_clock(ID_TWI1);

  TWI_DisableIt(HW_ADDR_TWI_PCA9698,0);
  TWI_ConfigureMaster(HW_ADDR_TWI_PCA9698,HW_TWI1_SPEED,VARIANT_MCK);
  
  HalExpansionPort.initilizePcaPorts(); 
  
  //Turn on power -  HePwrUsb5En then UOTGVBOF HIGH & Vbst12VEn(PC6)=0
  //
  
  HalExpansionPort.bit_Set(epbtPwrUsb5En);
  HalExpansionPort.portHwRefresh();
  
  initVirtualPin(HwVbst12VEn);
  pinMode(HwVbst12VEn, OUTPUT);
  digitalWrite(HwVbst12VEn,LOW);//No 12V

  // Never do this once Serial.begin() is called,!!
  PIOB->PIO_PER |= PIO_PB10A_UOTGVBOF;      //Enable PIO to control pin. WPEN must be clear. 
  PIOB->PIO_OER |= PIO_PB10A_UOTGVBOF;    //Set pin to output
  PIOB->PIO_SODR |= PIO_PB10A_UOTGVBOF;     //Write 1 - which turns ON USB 5V



#if 0
/* ------------------------------------------------------------------------ */
/* USB                                                                      */
/* ------------------------------------------------------------------------ */
/*! Multiplexed pin used for USB ID pin: */
//Arduino Pin85
#define USB_ID_GPIO                 (PIO_PB11_IDX)
#define USB_ID_FLAGS                (PIO_PERIPH_A | PIO_PULLUP)
/*! Multiplexed pin used for USB_VBOF: */
//Arduini Pin85  PINS_USB init in variant.cpp
#define USB_VBOF_GPIO               (PIO_PB10_IDX)
#define USB_VBOF_FLAGS              (PIO_PERIPH_A | PIO_DEFAULT)
/*! Active level of the USB_VBOF output pin. */
#define USB_VBOF_ACTIVE_LEVEL       LOW
/*! USB overcurrent detection pin. */
#define USB_OVERCURRENT_DETECT_PIN     PIO_PE5_IDX
#define USB_OVERCURRENT_DETECT_GPIO    (PIO_PE5_IDX)
#define USB_OVERCURRENT_DETECT_FLAGS   (PIO_INPUT | PIO_PULLUP)
 
 //pio.c:pio_configure_pin(uint32_t ul_pin, const uint32_t ul_flags)  
  	/* Configure USB_ID (UOTGID) pin */
  	//gpio_configure_pin(USB_ID_GPIO, USB_ID_FLAGS);
  	/* Configure USB_VBOF (UOTGVBOF) pin */
  	//gpio_configure_pin(USB_VBOF_GPIO, USB_VBOF_FLAGS);
  	/* Configure FAULT detect pin */
  	//gpio_configure_pin(USB_OVERCURRENT_DETECT_GPIO,
  	//USB_OVERCURRENT_DETECT_FLAGS);
  #endif
}

#ifdef __cplusplus
}
#endif

