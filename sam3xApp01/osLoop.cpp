/*
 * sam3xApp01.cpp
 * Copyright (c)2012 Azonde, All Rights Reserverd.
 
 * Created: 10/25/2012 5:45:04 PM
 *  Author: neil
 *
 *  Description - see isLoop.h
 *
 *  
 */ 
#include "sam.h"
#include "Arduino.h"
#include "az23project.h"
#include "osLoop.h"

extern "C" {

}//extern "C" 

// ***************************************************************
osLoopClass::osLoopClass(void) {
	u32_timerTick_10mS=0;
	maxNum_id =0;
	schedulerState=false;
  }//LoopClass
  
  // ***************************************************************
  void osLoopClass::timerTick() {
	//Determine which loops to run
    //if (++u32_timerTick_10mS >=1000) {
	//  u32_timerTick_10mS=0;
	//Serial.println("\n\r ****timerTick 10seconds");
	//}		  
	if (true==schedulerState) {
       for(uint8_t lpId=0;lpId <maxNum_id;lpId++) {//Loop through id
          if (0!=loopQue[lpId].qCntr_10mS) { //0 if no action
	         if (1==loopQue[lpId].qCntr_10mS--){//1 timer is fired
                scheduleLoop(lpId);//activate now
				//if stored copy != 1 then start timer again
				uint8_t timerCnt= (pManage+lpId)->timer_cnt;
				if (0!=timerCnt){
				   loopQue[lpId].qCntr_10mS=timerCnt+1; //reschedule		
				}
			 }
		  }
	   }			 
	}	
 }//timerTick
  
// ***************************************************************
void osLoopClass::setupLoops(const osLoopRef_t *pMngIn,uint8_t numLoops_id) {
	// idLp;
    noInterrupts();//atomic
	pManage = pMngIn;
	maxNum_id = numLoops_id;
      //memset( (void *)loopQue, eOsNoAction, sizeof(loopQue) );
	 for (uint8_t idLp=0;idLp<numLoops_id;idLp++ ) {
		  loopQue[idLp].qNext_id=eOsNoAction;
		  loopQue[idLp].qCntr_10mS=0;
	  }		  
      qHead_id = eOsNoAction;
      qTail_id = eOsNoAction;
    interrupts();
}//setupLoops

// ***************************************************************
bool osLoopClass::isWaiting( uint8_t this_id )  {
   return ((loopQue[this_id].qNext_id != eOsNoAction) || (qTail_id == this_id));
}

// ***************************************************************
inline bool osLoopClass::scheduleLoop( uint8_t my_id)  {
   return scheduleLoop(my_id,0); 
}
bool osLoopClass::scheduleLoop( uint8_t my_id, uint8_t time_10mS )  {
	if (my_id >= maxNum_id) return false;
    if( !isWaiting(my_id) )    {
      if( qHead_id == eOsNoAction )      {
        qHead_id = my_id;
        qTail_id = my_id;
      } else {
        loopQue[qTail_id].qNext_id = my_id;
        qTail_id = my_id;
      }
      return true;
    } else {
      return false;
    }
  }//scheduleLoop

// ***************************************************************
uint8_t osLoopClass::popLoop() {
   // move the head forward
   // if the head is at the end, mark the tail at the end, too
   // mark the task as not in the queue  
   uint8_t my_id=eOsNoAction;
   noInterrupts(); //atomic
   if( eOsNoAction != qHead_id) {
      my_id = qHead_id;
      qHead_id = loopQue[qHead_id].qNext_id;
      if( qHead_id == eOsNoAction ) {
        qTail_id = eOsNoAction;
      }
      loopQue[my_id].qNext_id = eOsNoAction;
   } 
   interrupts();
   return eOsNoAction;
}//popLoop

// ***************************************************************
void osLoopClass::runAllLoops(void)  {
   uint8_t nextLoopId ;
   
    while (eOsNoAction != (nextLoopId = popLoop()) )	{
       (*((pManage+nextLoopId)->loopFn))(); //Call nextLoopId
    } 
  }//allLoops  
  
// ***************************************************************  
bool osLoopClass::runNextLoop(void)  {
    uint8_t nextLoopId;
	bool retVal=true;
    noInterrupts();
    nextLoopId = popLoop();
    if( nextLoopId == eOsNoAction )  {
       retVal = false;
	}		
    interrupts();
	if (true==retVal) {
	   (*((pManage+nextLoopId)->loopFn))(); //Call it
	}	   
    return retVal;
  }//runNextLoop
  
// ***************************************************************
void osLoopClass::schedulerStart(void)  {
   uint8_t timerCnt;
   if (NULL==pManage) return; //***Not initialized
   for(uint8_t lpId=0;lpId <maxNum_id;lpId++) {//Loop through id
      //if stored copy != 1 then start timer again
	  timerCnt= (pManage+lpId)->timer_cnt;
	  if (0!=timerCnt){
		  loopQue[lpId].qCntr_10mS=timerCnt+1; //reschedule
	  }
   }	  
   schedulerState=true;
}//schedulerStart

// ***************************************************************
void osLoopClass::schedulerStop(void)   {schedulerState=false;}
bool osLoopClass::isSchedulerAct(void) {return schedulerState;}
	
	
//extern "C" {  
//	inline void SysTick_Handler(void) { osLoopClass.timerTick()}
//}//extern "C" 
