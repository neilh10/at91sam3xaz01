// $Id: $

/*
 * Management of the UI leds.
 * Output high intensity LEDS and input Switches at the antenna headend.
 *  LEDS Working Red
 *  Leds Connection  Yellow
 *  Switch Monitor (extinguishes LED..)
 *  Switch Test    (extinguishes LED ..
 *  
 */



#ifdef __cplusplus
extern "C" {
#endif
  norace uint8_t ledsOn;
//#include "sondeBM001.h"

#define RED_LED_ON()     if (ledsOn & eLedsManageMsk){UiLed1On();}
#define RED_LED_OFF()    {UiLed1Off();}
#define GREEN_LED_ON()   if (ledsOn & eLedsManageMsk) {UiLed2On();}
#define GREEN_LED_OFF()  {UiLed2Off();}
#define YELLOW_LED_ON()  //if (ledsOn & eLedsManageMsk) TOSH_CLR_YELLOW_LED_PIN()
#define YELLOW_LED_OFF() //TOSH_SET_YELLOW_LED_PIN()
  
result_t uiLedsClass::init() {
    atomic {
      ledsOn = 0x00; //Turn all off and leave off
      dbg(DBG_BOOT, "LEDS: initialized.\n");
//      TOSH_MAKE_RED_LED_OUTPUT();
//      TOSH_SET_RED_LED_PIN(); //turn off
//      TOSH_MAKE_GREEN_LED_OUTPUT();
//      TOSH_SET_GREEN_LED_PIN(); //turn off
    }
//    RED_LED_ON();
//    GREEN_LED_ON();
        LED_DDR  |= LED1M;
        LED_PORT |= LED1M;
        LedRedOff;
        LedGrnOn;

    return SUCCESS;
  }

   result_t uiLedsClass::redOn() {
    dbg(DBG_LED, "LEDS: Red on.\n");
    /* atomic */ {
      RED_LED_ON();
      ledsOn |= eLedsRedMsk;
    }
    return SUCCESS;
  }

   result_t uiLedsClass::redOff() {
    dbg(DBG_LED, "LEDS: Red off.\n");
     /* atomic */ {
       RED_LED_OFF();
       ledsOn &= ~eLedsRedMsk;
     }
     return SUCCESS;
  }

   result_t uiLedsClass::redToggle() {
    result_t rval;
    /* atomic */ {
      if (ledsOn & eLedsRedMsk)
	rval = call uiLedsClass::redOff();
      else
	rval = call uiLedsClass::redOn();
    }
    return rval;
  }

   result_t uiLedsClass::greenOn() {
    dbg(DBG_LED, "LEDS: Green on.\n");
    /* atomic */ {
      GREEN_LED_ON();
      ledsOn |= eLedsGreenMsk;
    }
    return SUCCESS;
  }

   result_t uiLedsClass::greenOff() {
    dbg(DBG_LED, "LEDS: Green off.\n");
    /* atomic */ {
      GREEN_LED_OFF();
      ledsOn &= ~eLedsGreenMsk;
    }
    return SUCCESS;
  }

   result_t uiLedsClass::greenToggle() {
    result_t rval;
    /* atomic */ {
      if (ledsOn & eLedsGreenMsk)
	rval = call uiLedsClass::greenOff();
      else
	rval = call uiLedsClass::greenOn();
    }
    return rval;
  }

   result_t uiLedsClass::yellowOn() {
    dbg(DBG_LED, "LEDS: Yellow on.\n");
    /* atomic */ {
      YELLOW_LED_ON();
      ledsOn |= eLedsYellowMsk;
    }
    return SUCCESS;
  }

   result_t uiLedsClass::yellowOff() {
    dbg(DBG_LED, "LEDS: Yellow off.\n");
    /* atomic */ {
      YELLOW_LED_OFF();
      ledsOn &= ~eLedsYellowMsk;
    }
    return SUCCESS;
  }

   result_t uiLedsClass::yellowToggle() {
    result_t rval;
    /* atomic */ {
      if (ledsOn & eLedsYellowMsk)
	rval = call uiLedsClass::yellowOff();
      else
	rval = call uiLedsClass::yellowOn();
    }
    return rval;
  }
  
   uint8_t uiLedsClass::get() {
    uint8_t rval;
    /* atomic */ {
      rval = ledsOn;
    }
    return rval;
  }
  
   result_t uiLedsClass::set(uint8_t ledsNum) {

    /* atomic */ {
      ledsOn = ((eLedsMsk & ledsNum)|(eLedsManageMsk & ledsOn));
      if (ledsNum & eLedsRedMsk) {
        RED_LED_ON();
      } else {
        RED_LED_OFF();
      }
      if (ledsNum & eLedsGreenMsk) {
        GREEN_LED_ON();
      }else{
        GREEN_LED_OFF();
      }
      if (ledsNum & eLedsYellowMsk) {
        YELLOW_LED_ON();
      }else{
        YELLOW_LED_OFF();
      }
    }
    return SUCCESS;
  }
   result_t uiLedsClass::redSet(uint8_t ledsReq) {

    /* atomic */ {
      ledsOn = (eLedsManageMsk & ledsOn);
      if (ledsReq) {
        RED_LED_ON();
      } else {
        RED_LED_OFF();
      }
    }
    return SUCCESS;
  }
  
   result_t uiLedsClass::greenSet(uint8_t ledsReq) {
    /* atomic */ {
      ledsOn = (eLedsManageMsk & ledsOn);
      if (ledsReq) {
        GREEN_LED_ON();
      } else {
        GREEN_LED_OFF();
      }
    }
    return SUCCESS;
  }

   result_t uiLedsClass::yellowSet(uint8_t ledsReq) {
    /* atomic */ {
      ledsOn = (eLedsManageMsk & ledsOn);
      if (ledsReq) {
        YELLOW_LED_ON();
      } else {
        YELLOW_LED_OFF();
      }
    }
    return SUCCESS;
  }

uint8_t uiLedsClass::MngEnable(uint8_t enData) {
    /* atomic */ {
      ledsOn |= eLedsManageMsk;
      if (ledsOn & eLedsRedMsk) RED_LED_ON();
      if (ledsOn & eLedsGreenMsk) GREEN_LED_ON();
      //if (ledsOn & eLedsRedYellow) YELLOW_LED_ON();
    }
    return SUCCESS;
  }
uint8_t uiLedsClass::MngDisable(uint8_t disData) {
    /* atomic */ {
      ledsOn &= ~eLedsManageMsk;
      RED_LED_OFF();
      GREEN_LED_OFF();
      YELLOW_LED_OFF();
    }
    return SUCCESS;
  }
uint8_t uiLedsClass::MngStatus(uint8_t statData) {
    return ledsOn;
  }
#ifdef __cplusplus
 }
#endif
