/*
 * CFile1.c
 *
 * Created: 11/23/2012 9:02:31 PM
 *  Author: neil
 */ 


//BOARD=ARDUINO_DUE_X

#include "board.h"
//#  include "arduino_due_x/arduino_due_x.h"
//#  include "system_sam3x.h"
#include "sysclk.h"
#include "twi_master.h"

//#include "led.h"
#include "asf.h"
//#include "stdio_serial.h"
//#include "conf_uart_serial.h"
//#include "conf_board.h"
//#define TWI1       ((Twi    *)0x40090000U)
#define BOARD_BASE_TWI_EEPROM       TWI1
#define TWI_EXAMPLE  BOARD_BASE_TWI_EEPROM

//#define CONF_BOARD_TWI1
//#define CONF_BOARD_UART_CONSOLE
//#include "delay.h"
//#include "rstc.h"


//#define EEPROM_BUS_ADDR       0x50        //!< TWI slave bus address
#define PCA9698_TWI_ADDR       0x20        //!< Pca9698 TWI slave bus address - raw not shifted
//#define PCA_TWI_DID_ADDR       0x7c       //Device ID PCA9698 = 0xF8 including R/W followed by three reads
#define CHIP_TWI_ADDR       PCA9698_TWI_ADDR        //
//#define CHIP_TWI_ADDR       PCA_TWI_DID_ADDR        //

//#define EEPROM_MEM_ADDR       0xaa        //!< TWI slave memory address
#define EEPROM_MEM_ADDR       0x18        //!< Pca9698 TWI slave memory address for Confug register
#define TWI_SPEED             400000       //!< TWI data transfer rate

//@}


//! \name Slave EEPROM memory Test Pattern Constants
//@{

#define PATTERN_TEST_LENGTH     sizeof(test_pattern_Pca9698Config)
const uint8_t test_pattern_Pca9698Config[] = {
   0x27, //Bank0
   0x00, //Bank1 outputs
   0x00, //2
   0x00, //3
   0x00 //4
   };
   
   void statusDecode (uint32_t  status) {
	uint code=(status>>8);
    switch (status &0x0f) {
	case TWI_SUCCESS: printf("SUCCESS %x",code);break;
	case TWI_INVALID_ARGUMENT: printf("INVALID_ARGUMENT %x",code);break;
	case TWI_ARBITRATION_LOST: printf("ARBITRATION_LOST %x",code);break;
	case TWI_NO_CHIP_FOUND:   printf("NO_CHIP_FOUND %x",code);break;
	case TWI_RECEIVE_OVERRUN: printf("RECEIVE_OVERRUN %x",code);break;
	case TWI_RECEIVE_NACK:    printf("RECEIVE_NACK %x",code);break;
	case TWI_SEND_OVERRUN:    printf("SEND_OVERRUN %x",code);break;
	case TWI_SEND_NACK:       printf("SEND_NACK %x",code);break;
	case TWI_BUSY:            printf("BUSY %x",code);break;
	default: printf("unknown %x",(uint)status);break;
	}	
} 
void TwiLoop() {
//void TwiLoop (void) {
	
	
	 // TWI master initialization options.

  twi_master_options_t opt = {
    .speed = TWI_SPEED,
    .chip  = CHIP_TWI_ADDR,
	.smbus = 0,
  };

  // Initialize the TWI master driver.

  twi_master_setup(TWI_EXAMPLE, &opt);

	#define cPca9698cmd_AI_bit 0x80
#define cPca9698cmd_IpReg   (0x00|cPca9698cmd_AI_bit)
#define cPca9698cmd_OpReg   (0x08|cPca9698cmd_AI_bit)
#define cPca9698cmd_CfgReg  (0x18|cPca9698cmd_AI_bit)
#define cPac9698cmd_MaskInt (0x20|cPca9698cmd_AI_bit)

  twi_package_t packet_tx1 = {
    .addr[0]      = cPca9698cmd_CfgReg, // TWI slave memory address data MSB
    .addr[1]      = 0,//EEPROM_MEM_ADDR,      // TWI slave memory address data LSB
    .addr_length  = 1,//sizeof (uint16_t),    // TWI slave memory address data size
    .chip         = CHIP_TWI_ADDR,      // TWI slave bus address
    .buffer       = (void *)test_pattern_Pca9698Config, // transfer data source buffer
    .length       = sizeof(test_pattern_Pca9698Config)   // transfer data size (bytes)
  };

  // Perform a multi-byte write access then check the result.
 // while (twi_master_write(TWI_EXAMPLE, &packet) != TWI_SUCCESS);

  uint8_t data_received[PATTERN_TEST_LENGTH] = {0};


  twi_package_t packet_received = {

    .addr[0]      = cPca9698cmd_IpReg,// Cmd
    .addr[1]      = 0,//EEPROM_MEM_ADDR,      // TWI slave memory address data LSB
    .addr_length  = 0,    // TWI slave memory address data size
    .chip         = CHIP_TWI_ADDR,      // TWI slave bus address
    //.chip         = PCA_TWI_DID_ADDR,      // TWI slave bus address	
    .buffer       = data_received,        // transfer data destination buffer
    .length       = 4   // transfer data size (bytes)
  };

  // Perform a multi-byte read access then check the result.
  
    uint8_t status;
	#if 0
	status = twi_master_read(TWI_EXAMPLE, &packet_received);
  	putchar(' ');
  	putchar('r');

	if (status == TWI_SUCCESS) {
      LED_On(LED2_GPIO);
      LED_Off(LED1_GPIO);
	  	putchar('s');
		putchar('=');
		for (uint32_t lp=0;lp<packet_received.length;lp++) {
		  printf("%x ", data_received[lp]);		  
		}
	

	}else{
			
      LED_Off(LED2_GPIO);			
      LED_On(LED1_GPIO);
	  if (status > 0x20) {
		 putchar('b');//TwiBusy
		 putchar('0'+(status-0x20));
	  } else if (status >0x10){
		putchar('n');//TwiNack
		putchar('0'+(status-0x10));
	  } else {
		  putchar('f');
		  }					  

	}
    delay_ms (100);
	#endif //0
	//Write to Configuration registoer
	status = twi_master_write(TWI_EXAMPLE, &packet_tx1);
  	putchar(' ');
  	putchar('t');
	if (status == TWI_SUCCESS) {
	  	putchar('s');
	} else {
      statusDecode (status);
	}	
	delay_ms (100);			
		
   packet_tx1.addr[0]      = cPca9698cmd_OpReg;
   packet_tx1.buffer=data_received;
   packet_tx1.length=5;
    while (1) {
			//Write to Port registoer
		status = twi_master_write(TWI_EXAMPLE, &packet_tx1);
  		putchar(' ');
  		putchar('t');
		if (status == TWI_SUCCESS) {
	  		putchar('s');
		} else {
		  statusDecode (status);
		}	
	delay_ms (1000);
	data_received[3]++;	
		
	}		
}	

