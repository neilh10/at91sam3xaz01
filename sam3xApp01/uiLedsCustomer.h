/* uiLedsCustomer.h uiLedsCustomer library
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
 * Class uiLedsCustomer to manage scheduling of multipile leds.
 * The leds can be turned OFF, ON, FlashSlow, FlashFast.
 * The leds state can be all disabled, all enabled.
 * 
 * Some logical leds map to physical led with dual colour
 * Some logical leds map to physical led with single colour or a shared on a dual led
 * 
 * Power Management
 * Pwr Red/Grn or flash x/slow/fast (Red is on at power up)
 * BatA Red/Grn or flash x/slow/fast ExpD44 & ExpD45 
 * BatB Red/Grn or flash x/slow/fast ExpD43 & ExpD42
 * Signal Management
 * Sync Blue  On/Off/flash x/slow/fast ExpD41
 * Tx Grn or flash x/slow/fast ExpD33
 * Rx Red or flash x/slow/fast ExpD32
 * SignalStrength Red.Grn or flash x/slow/fast ExpD30 & ExpD31
 *
 * USB indication
 * UsbOtg -? label LedUsb4 Front (single colour) ExpD36
 * Usb5 -? Label Led Usb4 back (single colour)   ExpD37
 * Usb1  -? Label Led Usb3 Tob (single colour)    ExpD34
 * Usb2  -? label Led Usb3 btm  (single colour)   ExpD35
 * Usb3  -? label Led UsbA left (single colour)   ExpD26
 * Usb4  -? label Led UsbA rihgt (single colour)  ExpD27
 * 
 * */
#ifndef uiLedsCustomer_h
#define uiLedsCustomer_h


typedef enum {
  eula_off,
  eula_on,
  eula_flashfast,
eula_flashslow,
} euiLedsAct

typedef enum {
    //Type of led
    eulc_std,//Single colour
	eulc_Red, //loop always enabled asynchronously
	eulc_Grn,
	eulc_Both,
  } euiLedsColourType;
typedef enum {
	//Leds with multipile names
euln_1Red,
euln_1Grn,

  } euiLedsNumber;
  
class uiLedsCustomerClass  {
  public:
    bool led( uint8_t ledNumber,euiLedsColourType ledColour,euiLedsAct ledAction);
    ledMngOn();//All display  ON
    ledMngOff();//All display off
    //ledOn
    //ledOff
    //ledToggle
    //
	void timerTick(void);//System Input


  private:
    
};
#endif//uiLedsCustomer_h