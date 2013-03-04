/* osLoop.h osLoop library
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
 * Class osLoop to manage scheduling of multipile run to completion loops.
 * The "loops" or simple threads can be set to be called regularly or scheduled asynchronously.
 *
 * */
#ifndef osLoop_h
#define osLoop_h


typedef struct {
	//Array of loop meta data here.
	//eOsLoopType timer_type;
	uint8_t timer_cnt; //0 for async. If !0 value for count down in 10mS
	void (*loopFn)(void);
	void (*setupFn)(void);
	} osLoopRef_t;
	
typedef struct {
	//Internal strucuture for managing the loop schedule
	uint8_t qNext_id;
	uint8_t qCntr_10mS;
	} osLoopQue_t;
	
typedef enum {
	/*eOsTrial1,
	eOsUiInterface,
	eOsNumLoops,*/
	eOsNoAction = 0xfe
} eOsLoopList;

typedef enum {
	//Type of timer
	eOlt_Async, //loop always enabled asynchronously
	eOlt_Time1mS,
	eOlt_Time100mS,
	} eOsLoopType;	
	
class osLoopClass  {
  public:
    bool scheduleLoop( uint8_t my_id); //push on to queue
    bool scheduleLoop( uint8_t my_id,uint8_t elapsedTime_10mS); //push on to queue with delay
	//bool scheduleLoop( void (*fn)(void));
	//bool scheduleLoop( void (*fn)(void),uint8_t elapsedTime_id );
	void schedulerStart(void);//Starts this scheduling
	void schedulerStop(void); //Stops all scheduling except for active loop
	bool isSchedulerAct(void); //true for Active, false for inactive
    void runAllLoops(void); //Runs all loops in queue and then returns
    bool runNextLoop(void); //Run next loop off queue. Returns true if ran, false if no action
    osLoopClass(void);
    void setupLoops(const osLoopRef_t *pMng,uint8_t numLoops);
	void timerTick(void);//System Input


  private:
	  uint8_t popLoop();
	  bool isWaiting( uint8_t this_id );
	  const osLoopRef_t *pManage; //Pointer to loop definitions externally maintained.
	  uint8_t qHead_id; //next loopId to give
	  uint8_t qTail_id; //last loopId to give
	  uint8_t maxNum_id;
	  uint32_t u32_timerTick_10mS;
	  bool schedulerState;
	  osLoopQue_t loopQue[10/*eOsNumLoops*/];// qNext_id //FIFO buffer of loopIds
    
};

#ifndef AZ23_PRJ_FILE
 extern osLoopClass osLoop;
#endif //AZ23_PRJ_FILE
#endif//osLoop_h