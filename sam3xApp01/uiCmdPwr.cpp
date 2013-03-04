/*
 * uiCmdPwr.cpp
 * Copyright (c)2012 Azonde, All Rights Reserverd.
 
 * Created: 10/25/2012 5:45:04 PM
 *  Author: neil
 *
 *
 *  CmdPwr
 *  Process a cmd string for chaning the powering.
  *  Serial input could come from a number of sources,

 *  
 
 */ 

#include "sam.h"
#include "Arduino.h"
#include "az23project.h"
#include "sysDebug.h"
#define cOffsetDbgMask MASK_INDX_SystemMngXxx
#include "uiConsole.h"
#include "pmc.h"
#include "hpl_efc.h" //Not ported to Arduino
#include "HalTwiPca9698.h"


#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/** Current MCK in Hz */
uint32_t current_mck_Hz;

/** Internal Flash Controller 0. */
#define EFC     EFC0

#define CLOCK_SELECT_MENU \
"\n\rSelect one of the following clock configurations:\n\r" \
"  1: 125KHz from Fast RC\n\r"                     \
"  a: 24MHz from PLL clock\n\r"                    \
"  f: 84MHz from PLL clock\n\r"
/*"  2: 250KHz from Fast RC\n\r"                     \
"  3: 500KHz from Fast RC\n\r"                     \
"  4: 1MHz from Fast RC\n\r"                       \
"  5: 2MHz from Fast RC\n\r"                       \
"  6: 4MHz from Fast RC\n\r"                       \
"  7: 8MHz from Fast RC\n\r"                       \
"  8: 12MHz from Fast RC\n\r"                      \*/
/*"  b: 32MHz from PLL clock\n\r"                    \
"  c: 48MHz from PLL clock\n\r"                    \
"  d: 60MHz from PLL clock\n\r"                    \
"  e: 72MHz from PLL clock\n\r"                    \*/

#define PWR_MODE_MENU \
"\n\rSelect one of the following power down  modes:\n\r" \
"s - sleep"\
"w - wait"\
"b - backup"

#define PLLA_COUNT    0x3Fu
result_t setSystemClock(uint8_t uc_key);
/**
 *  Reconfigure UART console for changed MCK and baudrate.
 */
#ifdef RECONFIG_CONSOLE
static void reconfigure_console(uint32_t ul_mck, uint32_t ul_baudrate)
{
	const sam_uart_opt_t uart_console_settings =
			{ ul_mck, ul_baudrate, UART_MR_PAR_NO };

	/* Configure PMC */
	pmc_enable_periph_clk(CONSOLE_UART_ID);

	/* Configure PIO */
	gpio_configure_group(CONF_UART_PIO, CONF_PINS_UART, CONF_PINS_UART_FLAGS);

	/* Configure UART */
	uart_init(CONF_UART, &uart_console_settings);
}
#endif
/**
 * \brief Set default clock (MCK = 24MHz).
 */
static void set_default_working_clock(void)
{
	/* Switch MCK to slow clock  */
	pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);

	/* Switch mainck to external xtal */
	pmc_switch_mainck_to_xtal(0/*, BOARD_OSC_STARTUP_US*/);

	/*
	 * Configure PLLA and switch clock.
	 * MCK = 12000000 * (7+1) / 1 / 4 = 24 MHz
	 */
	pmc_enable_pllack(7, PLLA_COUNT, 1);
	pmc_switch_mck_to_pllack(PMC_MCKR_PRES_CLK_4);

	/* Disable unused clock to save power */
	pmc_osc_disable_fastrc();

	/* Save current clock */
	current_mck_Hz = 24000000; /* 24MHz */
}
/**
 * \brief Set default clock (MCK = 24MHz).
 */
static void setRs232Off(void){
	//Rs232InEnN = 1, Rs232FoffEnN=0
	HalExpansionPort.bitClr(epbtRs232FoffEnN);
	HalExpansionPort.bit_Set(epbtRs232InEnN);
	HalExpansionPort.portHwRefresh();
	
	HalExpansionPort.goLowPowerMode();
	//digitalWrite(HwPwrIn30VLdoEnN,HIGH);
	digitalWrite(HwPwrIn30VLdoEnN,LOW);
}	

/**********************************************
 * \brief 
 *
 * 
 * \return none
 */
void enterSleepMode(void ) {/* Enter into sleep Mode */
	Serial.println("sleep");
	setSystemClock('1');//Slowest RC clock
	//	delay(10);
	setRs232Off();
	delay(10);

	pmc_enable_sleepmode(0); 
	//but if exits
	set_default_working_clock();
#ifdef RECONFIG_CONSOLE
	reconfigure_console(current_mck_Hz, CONF_UART_BAUDRATE);
#endif
	Serial.println("Exit from sleep Mode.\r");
}			
/**********************************************
 * \brief 
 *
 * 
 * \return none
 */
void enterWaitMode(void ) {/* Enter into sleep Mode */
	Serial.println(" Wait");
	delay(10);
	setRs232Off();
	delay(10);
	/* Wait for the transmission done before changing clock */
	//while (!uart_is_tx_empty(CONSOLE_UART)) {	}

	/* Configure 4Mhz fast RC oscillator */
	pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);
	pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_4_MHz);
	pmc_switch_mck_to_mainck(PMC_PCK_PRES_CLK_1);

	current_mck_Hz = 4000000; /* 4MHz */

	/* Disable unused clock to save power */
	pmc_osc_disable_xtal(0);
	pmc_disable_pllack();
    pmc_disable_all_pck(); //njh  PMC_SCDR
	pmc_disable_udpck(); //njh   PMC_SCDR
	pmc_disable_all_periph_clk(); //njh PMC_PCDR
	
	/* Set wakeup input for fast startup - need to set to allow RESET to work
	   Az23 uses PortInt_ into FWUP which is SUPC_WUMR |=FWUPEN */
	/** Wakeup pin for wait mode: -EK was RIGHT CLICK now?*/
	#define WAKEUP_WAIT_INPUT_ID     (1u << 14)
	pmc_set_fast_startup_input(WAKEUP_WAIT_INPUT_ID);

	/* Enter into wait Mode */
	while (1) {
		pmc_enable_waitmode();
	}
	/* Set default clock and re-configure UART */
	set_default_working_clock();
#ifdef RECONFIG_CONSOLE
	reconfigure_console(current_mck_Hz, CONF_UART_BAUDRATE);
#endif	

	Serial.println("Exit from wait Mode.");
}	

/**********************************************
 * \brief 
 *
 * 
 * \return none
 */
void enterBackupMode(void ) {/* Enter into sleep Mode */
	Serial.println(" backup");
	delay(10);
	setRs232Off();
	delay(10);
	
		/* Enable the PIO for wake-up */
//#if (BOARD == SAM3U_EK)
//	supc_set_wakeup_mode(SUPC, SUPC_WUMR_FWUPEN_ENABLE);
//#else
//#define WAKEUP_BACKUP_INPUT_ID   (1u << 14)
//	supc_set_wakeup_inputs(SUPC, WAKEUP_BACKUP_INPUT_ID,WAKEUP_BACKUP_INPUT_ID);
//#endif

	/* Switch MCK to slow clock  */
	pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);

	/* Disable unused clock to save power */
	pmc_osc_disable_xtal(0);
	pmc_disable_pllack();

	/* Enter into backup mode */
	pmc_enable_backupmode();
	
	/* Note: The core will reset when exiting from backup mode. */
		

#ifdef RECONFIG_CONSOLE
	reconfigure_console(current_mck_Hz, CONF_UART_BAUDRATE);
#endif	

	Serial.println("Exit from wait Mode.");	
}	

/** Clock list from fast RC */
const uint32_t g_fastrc_clock_list[][3] = {
	/* MCK,    FastRC,                  Prescaler */
{125000,   CKGR_MOR_MOSCRCF_4_MHz,  PMC_MCKR_PRES_CLK_32},
{250000,   CKGR_MOR_MOSCRCF_4_MHz,  PMC_MCKR_PRES_CLK_16},
{500000,   CKGR_MOR_MOSCRCF_4_MHz,  PMC_MCKR_PRES_CLK_8},
{1000000,  CKGR_MOR_MOSCRCF_4_MHz,  PMC_MCKR_PRES_CLK_4},
{2000000,  CKGR_MOR_MOSCRCF_4_MHz,  PMC_MCKR_PRES_CLK_2},
{4000000,  CKGR_MOR_MOSCRCF_4_MHz,  PMC_MCKR_PRES_CLK_1},
{8000000,  CKGR_MOR_MOSCRCF_8_MHz,  PMC_MCKR_PRES_CLK_1},
{12000000, CKGR_MOR_MOSCRCF_12_MHz, PMC_MCKR_PRES_CLK_1}
};
/** Clock list from PLL */
const uint32_t g_pll_clock_list[][4] = {
	/* MCK, MUL, DIV, PRES */
	/* MCK = 12000000 * (7+1) / 1 / 4 = 24 MHz */
{24000000, 7, 1, PMC_MCKR_PRES_CLK_4},
	/* MCK = 12000000 * (7+1) / 1 / 3 = 32 MHz */
{32000000, 7, 1, PMC_MCKR_PRES_CLK_3},
	/* MCK = 12000000 * (7+1) / 1 / 2 = 48 MHz */
{48000000, 7, 1, PMC_MCKR_PRES_CLK_2},
	/* MCK = 12000000 * (9+1) / 1 / 2 = 60 MHz */
{60000000, 9, 1, PMC_MCKR_PRES_CLK_2},
	/* MCK = 12000000 * (35+1) / 3 / 2 = 72 MHz */
{72000000, 35, 3, PMC_MCKR_PRES_CLK_2},
	/* MCK = 12000000 * (13+1) / 1 / 2 = 84 MHz */
{84000000, 13, 1, PMC_MCKR_PRES_CLK_2}
};
/** Current MCK in Hz */
uint32_t g_ul_current_mck;

/**
 * \brief Change clock configuration.
 *
 * \param uc_key Selection - see help.
 */

result_t setSystemClock(uint8_t uc_key)
{
uint32_t ul_id;
result_t retResult=SUCCESS;


#define MIN_CLOCK_FAST_RC_ITEM '1'
#define MAX_CLOCK_FAST_RC_ITEM '8'
#define MIN_CLOCK_PLL_ITEM     'a'
#define MAX_CLOCK_PLL_ITEM     'f'
	if ((uc_key >= MIN_CLOCK_FAST_RC_ITEM) &&
			(uc_key <= MAX_CLOCK_FAST_RC_ITEM)) {
		ul_id = uc_key - MIN_CLOCK_FAST_RC_ITEM;

		/* Save current clock */
		g_ul_current_mck = g_fastrc_clock_list[ul_id][0];
		Serial.print("Using RC Clock (Hz): ");
		Serial.println(g_ul_current_mck);

		/* Switch MCK to Slow clock  */
		pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);

		/* Switch mainck to fast RC */
		pmc_osc_enable_fastrc(CKGR_MOR_MOSCRCF_4_MHz);
		pmc_switch_mainck_to_fastrc(g_fastrc_clock_list[ul_id][1]);

		/* Switch MCK to mainck */
		pmc_switch_mck_to_mainck(g_fastrc_clock_list[ul_id][2]);

		/* Disable unused clock to save power */
		pmc_osc_disable_xtal(0);
		pmc_disable_pllack();
	} else if ((uc_key >= MIN_CLOCK_PLL_ITEM) &&
			(uc_key <= MAX_CLOCK_PLL_ITEM)) {
		ul_id = uc_key - MIN_CLOCK_PLL_ITEM;

		/* Save current clock */
		g_ul_current_mck = g_pll_clock_list[ul_id][0];
		Serial.print("Using PLL Clock (Hz): ");
		Serial.println(g_ul_current_mck);

		/* Switch MCK to slow clock  */
		pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);

		/* Switch mainck to external xtal */
		//pmc_switch_mainck_to_xtal(0, BOARD_OSC_STARTUP_US);
		pmc_switch_mainck_to_xtal(0);
		/* Configure PLLA and switch clock */
		pmc_enable_pllack(g_pll_clock_list[ul_id][1], PLLA_COUNT,
				g_pll_clock_list[ul_id][2]);
		pmc_switch_mck_to_pllack(g_pll_clock_list[ul_id][3]);

		/* Disable unused clock to save power */
		pmc_osc_disable_fastrc();
	} else {
		Serial.println("Unknown selection. Opts 1-8,a-f\n\r 1-8 is RC speeds,  a-f PLL Osc speed. Reset required afterwards ");
		retResult=FAIL;
	}
	delay(10); //Allow print output
	return retResult;
}
/**********************************************
 * \brief 
 *
 * buss access 
 * clock change 
 * lowpower mode - sleep, wait, backup
 * 
 * \return none
 */
const result_t uiConsole_pwrCmd(char *pConsoleIn,uint8_t pos,uint8_t inConPos) {
	result_t retResult=FAIL;
	result_t parseStatus = FAIL;
	bool prtStatus=true;
	
	Serial.println("uiConsole_pwrCmd - low powering");
	char swChar1=* pConsoleIn;
	pConsoleIn++;
	char swChar2= *pConsoleIn;
	pConsoleIn++;
	switch (swChar1) {
		case 'b': //buss 'f' 128Bit 's' 64bit
		  Serial.print("set bus mode -");
		  switch(swChar2){  
			  case 'f':	setSam3xBus128bits(); Serial.println("128bit"); break;
			  case 's':	setSam3xBus64bits();  Serial.println("64bit"); break;
			  default: Serial.println("-not changed");			  
		  }					
		break;
		case 'c':retResult = setSystemClock(swChar2); break;
		case 'l': //low power modes
		  switch(swChar2){ 
			  Serial.println("set power mode:");
			case 's': enterSleepMode(); break;
			case 'w': enterWaitMode();  break;
			case 'b': enterBackupMode();  break;
			default: Serial.println("-not changed");break;
		   }			  
		   break;
		case 's': prtStatus=true;
		default: break;
	}		
	if (prtStatus) {
		Serial.println("Current configuration:");
		Serial.print("  CPU Clock : MCK=");	Serial.print((int)current_mck_Hz); Serial.println(" Hz");
		if ((efc_get_flash_access_mode(EFC) & EEFC_FMR_FAM) == EEFC_FMR_FAM) {
			Serial.print("  Flash access mode : 64-bit\n\r");
		} else {
			Serial.print("  Flash access mode : 128-bit\n\r");
		}
	
	}
	return retResult;
}		


