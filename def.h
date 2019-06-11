/* 
 * File:   def.h
 * Author: vaio
 *
 * Created on 2016年12月15日, 上午 9:15
 */

#ifndef DEF_H
#define	DEF_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "mcc_generated_files/mcc.h"
#include <stdio.h>                  // ??? itoa() ??

//#define USE_OLD_MODE 1    //
//#define USE_RANGE  1     
    
#define MOTION_SENSOR_SUPPORT 1  
//#define BLE_MODE 1   
#define USE_SPI 1 
#define USE_EEPROM 1 
//#define EEPROM_DEBUG 1    
//#define MASTER_MODE 1    
#define STOCKER     1    
//#define  USE_SLEEP 1    
 #define USE_B5   1   
// #define DEMO_ONLY 1   
 //#define  TEST_MULTI 1  
//#define TEST_STOCKER 1  
 //#define DATA_LOGGER2 1 
#define WRITE_EEPROM_SIZE 290  

#define STOCKER_START_CODE 0x0F
    

//#define DATA_LOGGER 1   
    
    
//#define  _32BIT_  1
    
#ifdef MASTER_MODE   
 #define V0 'M'
#elif DATA_LOGGER2
#ifdef USE_OLD_MODE
 #define V0 'E'
#else
 #define V0 'D'
#endif
#else
#ifdef USE_OLD_MODE
 #define V0 'O'
#else
#define V0 'V'
#endif
#endif    
#define V1 '0'
#define V2 '1'
#define V3 '1'
#define V4 '6'
 
#ifdef MASTER_MODE
#define MAX_WAIT_TIME 20
#define MAX_WAIT_TIME2 10000  
#else
#ifdef _32BIT_    
#define MAX_WAIT_TIME 40
#define MAX_WAIT_TIME2 60000
#else
#define MAX_WAIT_TIME 1
#define MAX_WAIT_TIME2 10000    
#endif 
#endif    
 

 #ifdef DATA_LOGGER2
    #define  LOG_LOW  32
    #define  LOG_HIGH 210
   #define FONTS8   1 
#endif     
 #ifdef DATA_LOGGER
    #define  PC_BUF_DEBUG    1
#endif        
    
#define EPD_WAIT 10 //20170913
    
 #define USE_EPD 1
#define USE_MAX_LINE 1
//#define USE_MAX_EPD 1  
   
#define MAX_EPD_COL 15
#define MAX_EPD_ROW 7
    
    
 
//#define HAVE_TO_DELAY 1    
#define LCD_OFF 1
#define SEND_UART2  1
#define TEST_226  1    
//#define   FOR_TEST 1
//#define  PC_BUF_DEBUG    1
#define  GET_CHECKSUM_PC 1


//#define  G_SENSOR  1  

#ifdef USE_EPD    
    #define EPD 1
    #define EPD_029 1
    #define XY_SWAP 1   
#endif    
#define MAX_EPD_BUFFER 128
#define SECOND_FIX 1     /// RED
//#define EPD_CLEAR  0x00 // 0xFF RED
 #define EPD_CLEAR  0xFF // 0xFF RED
 // 1 2   
 // 0 0 WHITE
 // 0 1 RED
 // 1 0 RED
 // 1 1 BLACK  
#define EPD_WHITE  0xFF // 0xFF RED    
#define EPD_BLACK  0x00 // 0xFF RED  
    
    
//#define WIRE_3 1

#define  T_LCD_SIZE 4736
 
#define  T_COL_CHAR_SIZE16  16
#define  T_ROW_CHAR_SIZE16  16
    
#define  T_COL_CHAR_SIZE  8
#define  T_ROW_CHAR_SIZE  8
    
#ifdef  EPD_029
#define T_COL 296
#define T_ROW 16
#else
#define T_COL 250
#define T_ROW 16
#endif    
 
#define SSS1 461 // CHeck SumError
#define SSS2 479
    
#define SSS3 284  /// Longer CODE 8
#define SSS4 335 // 10
#define SSS5 539 // 8
#define SSS6 591  //18
    
#define SSS7 11 // PUT Error 0x63 for 516 568
#define SSS8 9 
#define SSS0 12    
    
#define SSS9 19 // Fix 519
    
//#define BATTERY_LOW 682   
#define BATTERY_LOW 650    
#define UART_INIT 0x8088    //0x8008
#define PACKET_DELAY1 6
#define PACKET_DELAY2 60000

#define BYTES_DELAY 6000
#define HEADER_SIZE 20
#define HEADER_BUFFER (HEADER_SIZE+2)

#define MULTI_BYTE   0x7B 
#define MULTI_index 7    
#define DATASIZE_l  8
#define DATASIZE_h  9
#define DATASIZE_ll  11
#define DATASIZE_hh  12 //20170919
#ifdef USE_B5 
    #define MAX_MULTIFLAG 4  
#else    
#define MAX_MULTIFLAG 3   
#endif    
#define MAX_ONEFLAG   240       
    
    
#define UART_BUFFER     257
#define UART_BUFFERSIZE (UART_BUFFER-2)
    
#define MAX_PC_BUFFER 600
#define MAX_PC_ARRAY  (MAX_PC_BUFFER+2)     
    
#define PHASE1   3
#define PHASE2   20
#define PHASE3   1
#define PHASE4   1
#define PHASE5   2
#define PHASE6   0
#define PHASE7   2
#define PHASE18  0    

 #define ZERO_SIZE 208 
 #define FOUR_SIZE 47

 #define RANG0 0
 #define SIZE0 208
  
  
 #define RANG1 208 // PC_BUF START
 #define SIZE1 32   

 #define CHECKSUM1 256
 #define CHECKSUMH 257
    
#define   BASE1    0x2A //0x52   
#define   EMPTY1   0x20  
    
#define   BASEH   0x05  //0x08   
    
#define    D0_LOC 240
#define    D0_BASE 0xD0
#define    D0_OFFSET 16    
#define    EMPTY_DATA  0x20    
    
 #define MID_BUF_SIZE 650   
    
 #define SKIP1 249
 #define SKIP1_SIZE 10 
    
#define SKIP2 509
#define SKIP2_SIZE 10 

#define LCD_SIZE 16    
    

#define SPEEDUP 1    

#define PC1_SIZE 250
#define PC2_SIZE 500
#define PC3_SIZE 750
#define PC4_SIZE 250

#define TAIL_CHECKSUM_BASE 0x2100    
#define TAIL_BASE          0x71   
#define TAIL_256BASE       0xFF   
    
 
#define DATA_NORMAL_SIZE   224    
#define DATA_MAX_SIZE      256  
#define DATA_MAX_SIZE2     512 
#define TAIL_ADDR0         55 //267    
#define TAIL_ADDR00        56 //268  
#define TAIL_ADDR1         59 //267    
#define TAIL_ADDR2         60 //268    
#define TAIL_ADDR3         61 //269    
#define TAIL_ADDR4         62 //270    

#define ADJ1_ADDR1         250//267    
#define ADJ1_ADDR2         251 //268    
#define ADJ1_ADDR3         252 //269    
#define ADJ1_ADDR4         253 //270    
#define ADJ1_ADDR5         254 //270 
#define ADJ1_ADDR6         255//270 
    
#define ADJ2_ADDR1         500 //267    
#define ADJ2_ADDR2         501 //268    
#define ADJ2_ADDR3         502 //269    
#define ADJ2_ADDR4         503 //270    
#define ADJ2_ADDR5         504 //270 
#define ADJ2_ADDR6         505 //270 

#define ADJ2_ADDR7         506 //267    
#define ADJ2_ADDR8         507//268    
#define ADJ2_ADDR9         508 //269    
#define ADJ2_ADDR10        509 //270    
#define ADJ2_ADDR11        510 //270 
#define ADJ2_ADDR12        511 //270      
    
#define CHECKSUM1_L        0      
#define CHECKSUM1_H        1      
#define CHECKSUM2_L        2      
#define CHECKSUM2_H        3      
#define CHECKSUM3_L        4      
#define CHECKSUM3_H        5   
#define CHECKSUM4_L        6      
#define CHECKSUM4_H        7
#define CHECKSUM5_L        8      
#define CHECKSUM5_H        9 
#define CHECKSUM6_L        10      
#define CHECKSUM6_H        11
#define DATASIZE           12  
    
#define ENTER_CODE 0x10
#define CHECKSUM_BASE     0x50    
 
 #define LOTID_MIN 3  
 #define LOTID_MAX 15
    
 #define BLE_MAX 36
 #define BLE_BUF (BLE_MAX+4)
   
  
#define RW       LATBbits.LATB3  // PORTBbits.RB3
 
#define rd     	 LATBbits.LATB14  //PORTBbits.RB14  
#define RW1    	 LATBbits.LATB15//PORTBbits.RB15
#define RS    	 LATAbits.LATA8 //PORTAbits.RA8
#define P0	     PORTC    //LATC	//PORTC
#define PLED     LATCbits.LATC9 //PORTAbits.RA8

    
#define BUTTON_UP       PORTBbits.RB0  // PORTBbits.RB3    
#define BUTTON_DOWN     PORTBbits.RB1  // PORTBbits.RB3  
    
extern unsigned char UART_Buffer[UART_BUFFER];

extern unsigned int rx_bytes;

extern void print_letter(int x, int y, char letter);

extern unsigned int  BatteryLevel;
extern unsigned int idle;
extern unsigned int idle2;
extern unsigned char findFC;
extern unsigned char lcdOnFlag;
extern unsigned int  LCD_index;
extern unsigned int  times16;
extern unsigned char showBatteryFlag;
extern unsigned char FinishFlag;
extern unsigned char DisplayFlag;
extern unsigned int rx_bytes;
extern unsigned int rx_index;
extern unsigned int rx2_bytes;
extern unsigned int rx2_index;
extern unsigned char rs232_s;
extern unsigned char rs232_FC;
extern unsigned char wait_bytes;
extern unsigned char lcd_log_flag;
extern unsigned int pc_buf_index;
extern unsigned int lcd_buf_index;
extern unsigned char UART_Buffer[UART_BUFFER];
extern unsigned char UART2_Buffer[UART_BUFFER];
extern unsigned char adcBuf[16];
extern unsigned char LCD_BUF[];

extern void EPD_printf(unsigned char cc,unsigned char *buf);
extern void sendBLE (void);

#define NVM_Page_Base	0xbd008000
#define NVM_Page_Size	4096

extern	unsigned int NVMOffset;
extern	unsigned char * NVMDataPointer;

extern	void UART1_WaitEmpty (void);
extern	void UART1_Clear (void);
extern	unsigned int i,j;

#ifdef _32BIT_
#define UART_DELAY 3000
#define EMPTY_DELAY 65000
#else
#define CHECKSUM_DELAY 500 //20170913
#define FILE_DELAY 100 //20170913
#ifdef MASTER_MODE
#define UART_DELAY 50 //20170913
#define EMPTY_DELAY 100
#else
#define UART_DELAY 50 //20170913
//#define EMPTY_DELAY 20000
#define EMPTY_DELAY 50
#endif
#endif    

#define  ADD_FF 1

#define SIZE1_LOW 256
#define SIZE1_HIGH 257

#define PC_SEND 0x77
#define PC_REV  0xF7
#define SKIP_FF  1

#define FORMAT_CODE 0x7F
#define FORMAT_START 2
#define CMD_LOC      7
#define FORMAT_NED 30
#define FAST_MODE 1
#define START_CODE  0xB5

#define LED_OFF IO_RC2_SetHigh // LED_OFF
#define LED_ON IO_RC2_SetLow  // LED_On

#define GIEE_SIZE (FOUR_SIZE-8)

#define TSMC_SIZE  220
 #define ASIM_SIZE 243
#define TAG_SIZE   246
#define ACONFIG_SIZE   258

#define SPI_DUMMY_SIZE 10
#define CHECKSUM_SIZE  16

#define eeDATASIZE      0x3FF
#define eeTAGSIZE       0x3FE
#define eeTSMCFLAG      0x3FD
#define eeACONFIG       0x3FC
#define eeGIEE          0x3FB
#define S2EEPFOM_H      0x3FA 
#define S2EEPFOM_L      0x3F9 
#define eeSTOCKER       0x3F8


#define S2EEPFOM_START  400
#define S2EEPFOM_MAX    900 
#define eeCHECKSUM_BUF_end 0x3EA
#define eeCHECKSUM_BUF  0x3E0

#ifdef	__cplusplus
}
#endif
    
#endif	/* DEF_H */

