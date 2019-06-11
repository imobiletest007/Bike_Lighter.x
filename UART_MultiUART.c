

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------
// History 
// 2017.01.10 version 0.1
// 2017.09.13 version 0.1 add rs232 buffer 8-200
//                          move 0x60 latter
//                         split ASIM and AConfig into two function
// 2017.09.18   Add GIEE code 

#include "def.h"
#include <stdio.h>                  // ??? itoa() ??
#include	<string.h> //memset
#ifdef _32BIT_
#include "mcc_generated_files/mcc.h"

#include "lcd.h"                    // LCD ??????
#include "st7066.h"

#else

#include "mcc_generated_files/eusart1.h"
#endif
#include "motion_sensor_custom.h"


#define IO_RC1_SetHigh   power_24_on_SetHigh            
#define IO_RC1_SetLow   power_24_on_SetLow  
#define IO_RD2_SetHigh  LCM_DB2_SetHigh
#define IO_RD2_SetLow   LCM_DB2_SetLow
#define IO_RC2_Toggle   M_INDICATOR_Toggle
#define IO_RC2_SetLow   M_INDICATOR_SetLow
#define IO_RC2_SetHigh  M_INDICATOR_SetHigh
#define IO_RC0_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define IO_RC0_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define IO_RD5_SetHigh()            do { LATDbits.LATD5 = 1; } while(0)
#define IO_RD5_SetLow()             do { LATDbits.LATD5 = 0; } while(0)

uint8_t tmr1_flag_10s=0;
uint8_t tmr1_flag_10s_index=0;	
	
unsigned char setFontFlag=0;
unsigned char twoButtonFlag=0;
void setFont8(void);
void setMaxLine(void);
unsigned char erack=0;
unsigned char stockerFlag=0;
unsigned char stockerFirst=1;
unsigned char aconfigFlag=0;
unsigned char gieeFlag=0;
//unsigned int tagSize=0;
//unsigned char tsmcFlag=0;
unsigned char buttonUpF=0;
unsigned char buttonDownF=0;
void buttonUp(void);
void buttonDown(void);
void RS232_Disable (void);
void RS232_WakeUp (void);
uint8_t SPI2_8bit(uint8_t data);
uint8_t SPI2_Read8bit(void);
void ClearLCD(void);
void putcLCD(unsigned char LCD_Char);
void LCD_Set_Cursor(unsigned char CurY, unsigned char CurX);
void EPD_print(void);
void   EPD_flash(void);
void ClearLCD(void);
void EPD_PowerOn(void);
void EPD_PowerOff(void);
void EPD_Init(void);
void EPD_Show(void);
void setMode(void);
unsigned char  mode=0;
#ifdef TEST_MULTI 
//unsigned int  DataSize=640;
unsigned char for_test=1;
unsigned int tagSize=640;
unsigned char tsmcFlag=1;
unsigned int  DataSize=220;
unsigned int  MultiFlag=1;

#else
unsigned char for_test=1;
unsigned int  DataSize=192;
unsigned int  MultiFlag=0;
unsigned int tagSize=244;
unsigned char tsmcFlag=1;
#endif

unsigned char EPD_Wait=0;
#ifdef _32BIT_
void SLEEP(void);
unsigned char recFlag;
unsigned char UART = 0;                          // '0 is UART0; '1' is UART1
#else
unsigned char recFlag;
extern void putrs1USART_2(unsigned char data);
unsigned char UART = 0;                          // '0 is UART0; '1' is UART1
void button(void);
void display (void);
#endif
#ifdef DATA_LOGGER2
//unsigned int s2EEPFOM_lnd=0;
unsigned char lineCnt=0;
unsigned char DataCount=0;
unsigned char Hex2ASC (unsigned char aa );
#endif			 
unsigned char sysReady=0;
unsigned char adcBuf[16];
unsigned int  BatteryLevel;
unsigned char lcdOnFlag=0;
unsigned int  LCD_index=0;
unsigned int  Line_index=0; //20170819
unsigned char EPD_index=0;
unsigned int  times16;
unsigned char showBatteryFlag=0;
unsigned char FinishFlag=0;
unsigned char DisplayFlag=0;
unsigned int rx_bytes=0;
unsigned int rx_index=0;
unsigned int rx2_bytes=0;
unsigned int rx2_index=0;
unsigned char rs232_s=0;
unsigned char rs232_FC=0;
unsigned char wait_bytes=0;
void store_data (unsigned char cc);
void log_data(unsigned char cc);
void uartWakeUp(void);
unsigned char uartWakeUpCnt=0;							  
void showTitle(void);
void sendTitle(void);
unsigned  int showBattery(void);
void pecket_delay(void);

unsigned char putdata (unsigned char c);
unsigned char displayP (void);
void displayN (void);
void checksum (void);
unsigned char decode ( unsigned char cc);
unsigned char encode ( unsigned char cc);
void sendDataOut (unsigned int start , unsigned int ending , unsigned char checksuml ,unsigned char checksumh );
void sendHeadOut (unsigned int DataSS,unsigned int DataS,unsigned char flag);
unsigned int GetDataSize (void);
//unsigned int  get_real_data_size(unsigned int data);
unsigned int realDataSize=0;
void step01(void);
void step02(unsigned char cc);
void step03(void);
void step04(void);
void step05(void);
void step06(void);
void step07(void);
void step08(void);
void step09(void);
void step10(void);
void step11(void);
void step12(void);
void step13(void);
void step14(void);
void step18(void);
void step102(void);
void step103(void);
void step105(void);
void step106(void);
void step108(void);
void step110(void);
void step081(void);
void step200(void);
void step199(void);
#ifdef MASTER_MODE
    void step300(void);
    void step301(void);
	void step302(void);				   
#endif
void step400(void); //STOCKER
void switchdefault(unsigned char cc);

unsigned int i,j;

unsigned int idle=0;
unsigned int idle2=0;
unsigned char findFC=0;
unsigned char findB5=0;
unsigned char step08F=0; //20170913


unsigned char getData;



//bit UART = 0;                          // '0 is UART0; '1' is UART1
//CA 60 00 A8 42 FF 00 00 00 00 80 62 FF 00 00 5A 13 00 00 5E D3 20 67 CD FF 51 E0 
//CA 60 00 C8 A2 00 80 65 19 00 00 00 40 00 12 C7 7A 00 D8 64 4F 00 60 12 20 
//FC FF B5 FF FF FF DF FF FD FF 55 56 FF FF FF 9F FF
//CA 60 00 A8 42 FF 00 00 00 00 80 D0 00 00 00 5A 13 FF 0F 28 00 80 01 A3 FF ED 60
//FC FF B5 FF FF FF DF FF FD FF 5A 58 FF FF FF 9F FF
//CA 60 00 E8 42 FF 00 00 00 00 80 D0 00 00 00 5A 13 FF 00 28 00 80 01 A3 FF AE AE 8D 60 
//CA 60 00 E8 42 FF 00 00 00 00 80 D0 FF FF 15 5A 13 00 0F 28 00 80 A4 E3 FF B8 01 45 10 High 50981 C725
//CA 60 00 E8 42 FF 00 00 00 00 80 D0 FF FF 15 5A 13 00 00 28 00 80 A4 E3 FF B8 01 4D E0 High

//FC FF B5 FF FF FF DF FF FD FF FF FD FF FF FF 9F FF
//CA 60 00 08 42 FF 00 00 00 00 80 D0 00 00 00 5A 13 FF 00 28 41 C0 
unsigned char status3[]={ //16828197 Bettary Good
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20    
  0x00, 0xE8, 0x42, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0xD0, 0xFF, 0xFF, 0x15, 0x5A, 0x13, 0x00, 0x00, 0x28, 0x00, 0x80,
  0xA4, 0xE3, 0xFF, 0xB8, 0x01, 0x4D, 0xE0,
};
unsigned char status1[]={ //16828197 Bettary Good
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20    
   0x00, 0xE8, 0x42, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0xD0, 0x00, 0x00,0x00 ,0x5A, 0x13, 0xFF,0x00,  0x28, 0x00, 0x80,
   0x01, 0xA3 , 0xFF , 0xAE , 0xAE , 0x8D , 0x60};
 
unsigned char status0[]={//16827776 Bettary Bad
    0x00, 0xA8, 0x42, 0x00, 0xCD, 0x0D, 0x5B, 0xDC,0x80,
    0xC2, 0xFF, 0xD6, 0xC8, 0xFF, 0x13, 0xAA, 0x00, 0x5E, 0x00,
    0x00, 0xBE, 0x69, 0x00, 0x24, 0x10}; // 10 Bytes header

const unsigned char status2[]={	
//0xCA, 0x60, 
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20    
  0x00, 0xA8, 0x42, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0x62, 0xFF, 0x00, 0x00, 0x5A, 0x13, 0x00, 0x00, 0x5E, 0x40, 0x00, 
  0xA7, 0x90, 0xFF, 0x48, 0x60, 	
};

unsigned char erack_Header[]={ 0x00, 0xCF, 0x0A}; // 10 Bytes header
unsigned char Header0[]={ 0x00, 0xAF, 0x0A}; // 10 Bytes header
unsigned char Header[]={ 0x80, 0x80, 0x0A }; // 10 Bytes header
unsigned char Header2[]={ 0x00, 0x81, 0x0A }; // 10 Bytes header
unsigned char Header3[]={ 0x80, 0x00, 0x0A }; // 10 Bytes header
unsigned char tail0[]={0x00, 0x90, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00};                                          

unsigned char UART_Buffer[UART_BUFFER];
unsigned char UART2_Buffer[UART_BUFFER];
//unsigned char UUID_Buf[BLE_BUF];
unsigned char UUID_Buf[BLE_BUF]={
0x61,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,
0x61,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,
0x61,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,
0x30,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31};

unsigned char UUID_index=0;
unsigned char UUID_index2=0;
unsigned char getUUID2 (void);
unsigned char getUUID (void);

unsigned char lcd_size=0;

unsigned int pc_buf_index=0;
unsigned int lcd_buf_index=0;

unsigned char lcd_log_flag=0;

unsigned char PC_h1_index=0;
unsigned char PC_h2_index=0;
unsigned char PC_h3_index=0;
unsigned char PC_h4_index=0;
unsigned char PC_H1[HEADER_BUFFER];
unsigned char PC_H2[HEADER_BUFFER];
unsigned char PC_H3[HEADER_BUFFER];
unsigned char PC_H4[HEADER_BUFFER];
unsigned int NVMOffset = 0;
unsigned char * NVMDataPointer;

#ifdef USE_EPD
extern void EPD_print(void);
#endif

/*
LCM_PutHex( *NVMDataPointer );

			NVMOffset += 4;
			if( NVMOffset > 4095 )
			{
				NVMOffset = 0;

				//??C32 NVM Function????Flash Page?
				NVMErasePage( ( void * ) NVM_Page_Base );
			}
 */           

#ifdef PC_BUF_DEBUG
#ifdef  DATA_LOGGER 
unsigned char dataLogger=0;
#endif					   
unsigned char PC_BUF[MAX_PC_ARRAY];
#endif

unsigned char File_Header[]={ 0x00, 0x9E, 0x0A };
unsigned char FILE_BUF0[]={
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x8C, 0x00, 0x4C, 0x00, 0xCC, 0x00, 0xDC, 0xD5, 0xD1, 0x7B, 0xF7, 0x12, 0xC5, 0xD0, 0xD7, 0xF7, 0xD5, 0xF3, 0xF1, 0xFF, 
  0x5F, 0x73, 0x25, 0x74, 0x47, 0xDC, 0x3D, 0x5D, 0xC4, 0x6D, 0x5F, 0x84, 0xFF, 0xE4, 0xD7, 0x9D, 0xD9, 0x6D, 0x79, 0x99, 
  0x77, 0xF7, 0x51, 0xE7, 0x2D, 0x58, 0xB3, 0x67, 0x3B, 0xB4, 0x4B, 0xB5, 0x56, 0x7E, 0xD0, 0xFB, 0x05, 0x17, 0x1C, 0x35, 
  0x58, 0xBE, 0xBC, 0xF6, 0xD5, 0xE9, 0x51, 0x36, 0xE3, 0x54, 0x4F, 0x5C, 0x5D, 0x51, 0x1D, 0x4F, 0x65, 0xF5, 0x7B, 0x31, 
  0xC1, 0xE5, 0xC4, 0x51, 0x68, 0x1F, 0xC2, 0x8D, 0x8D, 0x75, 0x74, 0xF9, 0x47, 0x84, 0x97, 0xBE, 0x57, 0x9D, 0x53, 0x70, 
  0x13, 0xFD, 0x12, 0x5B, 0xDF, 0x55, 0x52, 0x72, 0xD4, 0xC0, 0x34, 0xBC, 0xDD, 0xE5, 0x6E, 0x63, 0x74, 0x25, 0x5D, 0x13, 
  0x85, 0xC2
};
const unsigned char FILE_BUF1[]={
//CA 60 00 9E 0A 
//1  2  3  4  5  6  7  8  9  10 1  2  3  4  5  6  7  8  9  1
  // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
	0xCA, 0x2A, 0xCA, 0x04, 0x2A, 0x86, 0xE6, 0x04, 0x4A, 0xA6, 0x6E, 0x04, 0x0C, 0xCC, 0x74, 0x0C, 0x0C, 0x04, 0x00, 0x00, //1
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAC, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //2 
	0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x80, 0x00, 0xB6, 0xB6, 0xF4, 0x26, 0x26, 0xF4, 0x9E, 0x9E, 0x00, 0x00, 0x16, 0x16, //3 
	0x5C, 0xB6, 0xB6, 0x5C, 0xCE, 0xCE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //4  
	0x26, 0xCE, 0xB6, 0x74, 0xC6, 0x66, 0xE6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //5  
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x80, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, //6 
	0xBB, 0x70,																												//7
};	
const unsigned char Stocker_End[]={ // Stocker 
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0xD6, 0xC0, 0xDD, 0x74, 0x1A, 0x08, 0x53, 0xDC, 0xCD, 0x4D, 0xAA, 0xFF, 0x53,
 };         
const unsigned char Stocker_End0[]={ // Stocker 
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x1B, 0x74, 
 };
const unsigned char LCD_BUF01[]={ // Stocker 
//CA 60 80 80 0A 
// 1  2  3  4  5  6  7  8  9  10 1  2  3  4  5  6  7  8  9  1
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x4F, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2E, 0x3B, 0x98,  //1
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5C, 0x00, 0x8C, 0x00, 0x32, 0xF2, 0x2A, 0x5C, 0xA2, 0xEC, 0xC2,  //2 
  0x0C, 0x0C, 0xCC, 0x74, 0x8C, 0x04, 0x04, 0x04, 0x04, 0x4A, 0xC2, 0x0A, 0x5C, 0xB2, 0x9A, 0x2A, 0x5A, 0x32, 0xCC, 0x74,  //3 
  0x8C, 0x0C, 0x04, 0x04, 0x04, 0xC2, 0xA2, 0x32, 0x5C, 0xEA, 0x62, 0xCA, 0x2A, 0x82, 0x4A, 0x2A, 0x04, 0x04, 0x04, 0x04,  //4 
  0x04, 0xA2, 0x8A, 0x0A, 0x5C, 0x22, 0x2A, 0x42, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0xA2, 0x92, 0x22,  //5 
  0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0A, 0x92, 0x22, 0x5C, 0x0A, 0x72, 0x4C,  //6 
  0x4C, 0x0C, 0x0C, 0x0A, 0x22, 0x04, 0x04, 0x04, 0x04, 0xCA, 0x2A, 0x82, 0x5C, 0xEA, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //7 
  0x04, 0x04, 0x04, 0x04, 0x04, 0x8A, 0x2A, 0x9A, 0x5C, 0x4C, 0xCC, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //8 
  0x04, 0xCA, 0x2A, 0xE2, 0x5C, 0x4A, 0xA2, 0xC2, 0x9A, 0xC2, 0x32, 0xA2, 0x04, 0x04, 0x04, 0x04, 0x04, 0x22, 0xA2, 0xCA,  //9 
  0x5C, 0x0A, 0xF2, 0xCA, 0x2A, 0x04, 0x0A, 0x82, 0x22, 0x04, 0x72, 0x92, 0x2A, 0x0A, 0x82, 0x4A, 0x5C, 0xA2, 0xC2, 0xA2,  //10
  0x4A, 0x82, 0xC2, 0x0A, 0x74, 0x0C, 0x8C, 0x04, 0x04, 0x22, 0xCA, 0x0C, 0x8C, 0xB4, 0x0C, 0xAC, 0x54, 0x0C, 0x9C, 0x4C,  //11
  0xCC, 0x0C, 0xCC, 0xAC, 0x1C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //12
  0x04, 0x04, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x1B, 0x74,              //13   
};

#ifdef USE_EEPROM

#ifdef MASTER_MODE
void Master_checksum (void);
const unsigned char LCD_BUF00[]={
 // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   //1
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 	//2 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 	//3
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 	//4 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 	//5
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 	//6
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 	//7
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 	//8
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 	//9
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 	//10
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x73, 0xB3, 0x33, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 	//11
  0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 	//12
  0xF4, 0x95, 0xFF, 0x5F, 0xFF, 
/////////////////////// end
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};   

const unsigned char master_control[]={
  // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
    0xFF, 0xFF, 0x1F, 0xFF, 0x77, 0xFF, 0xFF, 0x07, 0xB0, 0xFF, 0xBB, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xD0, 0xFF, 0xF5, 0xFF
};							  
const unsigned char master_contro2[]={
  // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
    0xFF, 0xFF, 0xEF, 0xFF, 0xF7, 0xFF, 0x00, 0x87, 0xFF, 0x7F, 0x7F, 0x0B, 0xFF, 0x7F, 0xFF,  0x9F, 0xFF, 
};   																														 
//const unsigned char master_header[]={
 // 0xFC, 0xFF, 0xB5, 0xFF,   
//};
const unsigned char master_header[]={
  0xFC, 0xFF, 0xB5, 0xFF,   
};
#else
#ifdef DEMO_ONLY
unsigned char LCD_BUF00[]={
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //1
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xF2, 0x2A, 0x5C, 0x8A, 0x4A, 0xCA, 0x0C, //2
   0x4C, 0x0C, 0x74, 0x8C, 0x04, 0x04, 0x04, 0x04, 0x4C, 0xAC, 0xF4, 0xCA, 0xAA, 0x42, 0x8C, 0xFA, 0x92, 0xB2, 0x04, 0x04, //3 
   0x04, 0xF4, 0x8C, 0x04, 0x92, 0x5A, 0x4A, 0x2A, 0x82, 0x72, 0x04, 0x04, 0xF4, 0x2A, 0xB2, 0x22, 0x6A, 0xCC, 0x4C, 0xA2, //4
   0x82, 0x8C, 0x0C, 0x0C, 0x0C, 0xC2, 0x0C, 0x8C, 0x0C, 0xCA, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x32, 0xF2, 0xC2, 0x5C, //5
   0x42, 0x22, 0x92, 0x62, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x4A, 0xB2, 0x32, 0x5C, 0x4C, 0x0C, //6 
   0x8C, 0xEC, 0xF4, 0x0C, 0x1C, 0xF4, 0x0C, 0x2C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, //7
   0x04, 0x04, 0x04, 0x04, 0x8A, 0x22, 0xAA, 0xA2, 0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, //8
   0x72, 0xF2, 0x2A, 0xA2, 0x8C, 0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, //9
   0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x2A, 0xA2, 0x4C, 0x5C, 0x04, 0x04, //10
   0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, //11
   0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, //12
   0x04, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x64, 0x00, 0x90, //13
   0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00,                                                       //14

 ///////////// ENd     
  // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00,00
  
};
#else
const unsigned char LCD_BUF00[]={
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //1
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0xA2, 0xB2, 0x0A, 0x5C, 0x2A, 0xA2, 0xCA, 0x2A,  //2
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0C, 0xF4, 0x2A, 0xA2, 0xCA, 0x2A, 0xFA, 0x32, 0xF2, 0x2A, 0x04,  //3
  0x04, 0xF4, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0xF4, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //4 
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x32, 0xF2, 0xC2, 0x5C,  //5  
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x4A, 0xB2, 0x32, 0x5C, 0x4C, 0x0C,  //6 
  0x8C, 0xEC, 0xF4, 0x8C, 0x8C, 0xF4, 0x8C, 0x2C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //7 
  0x04, 0x04, 0x04, 0x04, 0x8A, 0x22, 0xAA, 0xA2, 0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //8 
  0x72, 0xF2, 0x2A, 0xA2, 0x8C, 0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //9 
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x2A, 0xA2, 0x4C, 0x5C, 0x04, 0x04,  //10
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //11
  0x04, 0x04, 0x04, 0x04, 0x6A, 0xA6, 0x4E, 0xCE, 0x96, 0xF6, 0x76, 0x04, 0xCC, 0x74, 0xEC, 0x04, 0x04, 0x04, 0x04, 0x04,  //12 
  0x04, 0x30, 0x04, 0x04, 0x60, 0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x64, 0x00, 0x90, //13
   0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00,                                                       //14
   //0x04, 0x50, 0x00, 0x00, 0x00, 0x00,
 ///////////// ENd     
  // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00,00
  
};
#endif //DEMO_ONLY
#ifdef EEPROM_DEBUG
unsigned char LCD_BUF0[300];
#endif ///EEPROM_DEBUG
#endif  // MASTER_MODE
#else
unsigned char LCD_BUF0[]={
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //1
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0xA2, 0xB2, 0x0A, 0x5C, 0x2A, 0xA2, 0xCA, 0x2A,  //2
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0C, 0xF4, 0x2A, 0xA2, 0xCA, 0x2A, 0xFA, 0x32, 0xF2, 0x2A, 0x04,  //3
  0x04, 0xF4, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0xF4, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //4 
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x32, 0xF2, 0xC2, 0x5C,  //5  
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x4A, 0xB2, 0x32, 0x5C, 0x4C, 0x0C,  //6 
  0x8C, 0xEC, 0xF4, 0x8C, 0x8C, 0xF4, 0x8C, 0x2C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //7 
  0x04, 0x04, 0x04, 0x04, 0x8A, 0x22, 0xAA, 0xA2, 0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //8 
  0x72, 0xF2, 0x2A, 0xA2, 0x8C, 0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //9 
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x2A, 0xA2, 0x4C, 0x5C, 0x04, 0x04,  //10
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,  //11
  0x04, 0x04, 0x04, 0x04, 0x6A, 0xA6, 0x4E, 0xCE, 0x96, 0xF6, 0x76, 0x04, 0xCC, 0x74, 0xEC, 0x04, 0x04, 0x04, 0x04, 0x04,  //12 
  0x04, 0x30, 0x04, 0x04, 0x60, 0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x64, 0x00, 0x90, //13
   0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00,                                                       //14
   //0x04, 0x50, 0x00, 0x00, 0x00, 0x00,
 ///////////// ENd     
  // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00,00
  
};
#endif
/*
unsigned char LCD_BUF0[]={
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //1
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xF2, 0x2A, 0x5C, 0x8A, 0x4A, 0xCA, 0x0C, //2
   0x4C, 0x0C, 0x74, 0x8C, 0x04, 0x04, 0x04, 0x04, 0x4C, 0xAC, 0xF4, 0xCA, 0xAA, 0x42, 0x8C, 0xFA, 0x92, 0xB2, 0x04, 0x04, //3 
   0x04, 0xF4, 0x8C, 0x04, 0x92, 0x5A, 0x4A, 0x2A, 0x82, 0x72, 0x04, 0x04, 0xF4, 0x2A, 0xB2, 0x22, 0x6A, 0xCC, 0x4C, 0xA2, //4
   0x82, 0x8C, 0x0C, 0x0C, 0x0C, 0xC2, 0x0C, 0x8C, 0x0C, 0xCA, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x32, 0xF2, 0xC2, 0x5C, //5
   0x42, 0x22, 0x92, 0x62, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x4A, 0xB2, 0x32, 0x5C, 0x4C, 0x0C, //6 
   0x8C, 0xEC, 0xF4, 0x0C, 0x1C, 0xF4, 0x0C, 0x2C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, //7
   0x04, 0x04, 0x04, 0x04, 0x8A, 0x22, 0xAA, 0xA2, 0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, //8
   0x72, 0xF2, 0x2A, 0xA2, 0x8C, 0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, //9
   0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x2A, 0xA2, 0x4C, 0x5C, 0x04, 0x04, //10
   0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, //11
   0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, //12
   0x04, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x64, 0x00, 0x90, //13
   0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00,                                                       //14

 ///////////// ENd     
  // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00,00
  
};
*/

/*
unsigned char LCD_BUF0[]={
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
   0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xF2, 0x2A, 0x5C, 0x8A, 0x4A, 0xCA, 0x0C, 0x4C, 0x0C, 0x74, 0x8C, 0x04, 0x04, 0x04, 
   0x04, 0x4C, 0xAC, 0xF4, 0xCA, 0xAA, 0x42, 0x8C, 0xFA, 0x92, 0xB2, 0x04, 0x04, 0x04, 0xF4, 0x8C, 0x04, 0x92, 0x5A, 0x4A, 
   0x2A, 0x82, 0x72, 0x04, 0x04, 0xF4, 0x2A, 0xB2, 0x22, 0x6A, 0xCC, 0x4C, 0xA2, 0x82, 0x8C, 0x0C, 0x0C, 0x0C, 0xC2, 0x0C, 
   0x8C, 0x0C, 0xCA, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x32, 0xF2, 0xC2, 0x5C, 0x42, 0x22, 0x92, 0x62, 0x04, 0x04, 0x04, 
   0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x4A, 0xB2, 0x32, 0x5C, 0x4C, 0x0C, 0x8C, 0xEC, 0xF4, 0x0C, 0x1C, 0xF4, 0x0C, 
   0x2C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x8A, 0x22, 0xAA, 
   0xA2, 0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x2A, 0xA2, 0x8C, 0x5C, 0x04, 
   0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 
   0x04, 0x04, 0x04, 0x04, 0x04, 0x72, 0xF2, 0x2A, 0xA2, 0x4C, 0x5C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 
   0x04, 

// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
   0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
   0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 

// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x00, 0x28, 0x64, 0x00, 0x90, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00,

 ///////////// ENd     
  // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00,00
  
};*/
/*
unsigned char LCD_BUF0[]={
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 1
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 2
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 3
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 4
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 5
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 6
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 7 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 8
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 9
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 10
  0x00,

// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x84, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0B, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x00, 0xD4, 0xA0, 0x00, 0x90, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00,

 ///////////// ENd     
  // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00,00
  
};*/
unsigned char LCD_BUF1[]={
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x84, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x57, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xA0, 0x00, 0x90, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 
  0xC4, 0xFE, 0xC8, 0x80, 0x00    // 0x0B 0xD4
 // 0x44, 0x92, 0x3B, 0x00, 0x00
 // 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00,
 // 0x0A, 0x00, 0x00, 0x44, 0x03, 0x84, 0x00, 0xC4, 0xFE, 0xAF, 0x80, 0xB5,   // 0x0B 0xD4
//0x0A, 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0xC4, 0xFE, 0xC8, 0x80, 0x00,   // 0x0B 0xD4
//  00 00 00 00 84 00 C4 FE C8 80 640
 //  00 00 00 00 84 00 84 07 4E 80 225
 //  00 00 00 00 84 00 84 C7 AE 80 228
};


unsigned char CHECKSUM_BUF[]={
    0xC1,0x06,
    0xBC,0xDA,
    0x63,0x14,   //erack 3096-0x40=0x28c6 ==>  63 14
     0x61,0x54, // TSMC Aconfig
     0x60,0x94, // TSMC
// 0x1B, 0x74,  // Stocker
     0xBF, 0xF4,  // Stocker
	 0x00, 0x00,  // 
	 0x00, 0x00,  //						   
};
  unsigned char LCD_BUF[]={
   
// 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
 // 0xCA, 0x60, 0x80, 0x80, 0x0A, 
  0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 0x0E, 0x8E, 0x4E, 0xCE, 0x2E, // 1
  0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, // 2
  0xF2, 0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0x86, 0x86, 0x46, 0x46, 0xC6, 0xC6, 0x26, 0x26, // 3
  0xA6, 0xA6, 0x66, 0x66, 0xE6, 0xE6, 0x16, 0x16, 0x96, 0x96, 0x56, 0x56, 0xD6, 0xD6, 0x36, 0x36, 0xB6, 0xB6, 0x76, 0x76, // 4
  0xF6, 0xF6, 0x0E, 0x0E, 0x8E, 0x8E, 0x4E, 0x4E, 0xCE, 0xCE, 0x2E, 0x2E, 0xAE, 0xAE, 0x6E, 0x6E, 0xEE, 0xEE, 0x1E, 0x1E, // 5
  0x9E, 0x9E, 0x5E, 0x5E, 0x82, 0x82, 0x42, 0x42, 0xC2, 0xC2, 0x22, 0x22, 0xA2, 0xA2, 0x62, 0x62, 0xE2, 0xE2, 0x12, 0x12, // 6
  0x92, 0x92, 0x52, 0x52, 0xD2, 0xD2, 0x32, 0x32, 0xB2, 0xB2, 0x72, 0x72, 0xF2, 0xF2, 0x0A, 0x0A, 0x8A, 0x8A, 0x4A, 0x4A, // 7
  0xCA, 0xCA, 0x2A, 0x2A, 0xAA, 0xAA, 0x6A, 0x6A, 0xEA, 0xEA, 0x1A, 0x1A, 0x9A, 0x9A, 0x5A, 0x5A, 0x86, 0x86, 0x86, 0x46, // 8
  0x46, 0x46, 0xC6, 0xC6, 0xC6, 0x26, 0x26, 0x26, 0xA6, 0xA6, 0xA6, 0x66, 0x66, 0x66, 0xE6, 0xE6, 0xE6, 0x16, 0x16, 0x16, // 9
  0x96, 0x96, 0x96, 0x56, 0x56, 0x56, 0xD6, 0xD6, 0xD6, 0x36, 0x36, 0x36, 0xB6, 0xB6, 0xB6, 0x76, 0x76, 0x76, 0xF6, 0xF6, // 10
  0xF6, 0x0E, 0x0E, 0x0E, 0x8E, 0x8E, 0x8E, 0x4E, 0x4E, 0x4E, 0xCE, 0xCE, 0xCE, 0x2E, 0x2E, 0x2E, 0xAE, 0xAE, 0xAE, 0x6E, // 11
  0x6E, 0x6E, 0xEE, 0xEE, 0xEE, 0x1E, 0x1E, 0x1E, 0x9E, 0x9E, 0x9E, 0x5E, 0x5E, 0x5E, 0x82, 0x82, 0x82, 0x42, 0x42, 0x42, // 12
  0xC2, 0xC2, 0xC2, 0x22, 0x22, 0x22, 0xA2, 0xA2, 0xA2, 0x62, 0x62, 0x62, 0xE2, 0xE2, 0xE2, 0x12, 
  //0xC1,0x06,
  //0x80, 0x80, 0x0A, 
  0x12, 0x12, 0x92, 0x92, 0x92, 0x52, 0x52, 0x52, 0xD2, 0xD2, 0xD2, 0x32, 0x32, 0x32, 0xB2, 0xB2, 0xB2, 0x72, 0x72, // 14
  0x72, 0xF2, 0xF2, 0xF2, 0x0A, 0x0A, 0x0A, 0x8A, 0x8A, 0x8A, 0x4A, 0x4A, 0x4A, 0xCA, 0xCA, 0xCA, 0x2A, 0x2A, 0x2A, 0xAA, // 15
  0xAA, 0xAA, 0x6A, 0x6A, 0x6A, 0xEA, 0xEA, 0xEA, 0x1A, 0x1A, 0x1A, 0x9A, 0x9A, 0x9A, 0x5A, 0x5A, 0x5A, 0x86, 0x86, 0x86, // 16
  0x86, 0x46, 0x46, 0x46, 0x46, 0xC6, 0xC6, 0xC6, 0xC6, 0x26, 0x26, 0x26, 0x26, 0xA6, 0xA6, 0xA6, 0xA6, 0x66, 0x66, 0x66, // 17
  0x66, 0xE6, 0xE6, 0xE6, 0xE6, 0x16, 0x16, 0x16, 0x16, 0x96, 0x96, 0x96, 0x96, 0x56, 0x56, 0x56, 0x56, 0xD6, 0xD6, 0xD6, // 18
  0xD6, 0x36, 0x36, 0x36, 0x36, 0xB6, 0xB6, 0xB6, 0xB6, 0x76, 0x76, 0x76, 0x76, 0xF6, 0xF6, 0xF6, 0xF6, 0x0E, 0x0E, 0x0E, // 19
  0x0E, 0x8E, 0x8E, 0x8E, 0x8E, 0x4E, 0x4E, 0x4E, 0x4E, 0xCE, 0xCE, 0xCE, 0xCE, 0x2E, 0x2E, 0x2E, 0x2E, 0xAE, 0xAE, 0xAE, // 20
  0xAE, 0x6E, 0x6E, 0x6E, 0x6E, 0xEE, 0xEE, 0xEE, 0xEE, 0x1E, 0x1E, 0x1E, 0x1E, 0x9E, 0x9E, 0x9E, 0x9E, 0x5E, 0x5E, 0x5E, // 21
  0x5E, 0x82, 0x82, 0x82, 0x82, 0x42, 0x42, 0x42, 0x42, 0xC2, 0xC2, 0xC2, 0xC2, 0x22, 0x22, 0x22, 0x22, 0xA2, 0xA2, 0xA2, // 22
  0xA2, 0x62, 0x62, 0x62, 0x62, 0xE2, 0xE2, 0xE2, 0xE2, 0x12, 0x12, 0x12, 0x12, 0x92, 0x92, 0x92, 0x92, 0x52, 0x52, 0x52, // 23
  0x52, 0xD2, 0xD2, 0xD2, 0xD2, 0x32, 0x32, 0x32, 0x32, 0xB2, 0xB2, 0xB2, 0xB2, 0x72, 0x72, 0x72, 0x72, 0xF2, 0xF2, 0xF2, // 24
  0xF2, 0x0A, 0x0A, 0x0A, 0x0A, 0x8A, 0x8A, 0x8A, 0x8A, 0x4A, 0x4A, 0x4A, 0x4A, 0xCA, 0xCA, 0xCA, 0xCA, 0x2A, 0x2A, 0x2A, // 25
  0x2A, 0xAA, 0xAA, 0xAA, 0xAA, 0x6A, 0x6A, 0x6A, 0x6A, 0xEA, 0xEA, 0xEA, 0xEA, 0x1A, 0x1A, 0x1A, 0x1A,
  //0xBC, 0xDA,
  //0x00,  0x81, 0x0A,
  0x9A, 0x9A, 0x9A, 0x9A, 0x5A, 0x5A, 0x5A, 0x5A, 0x86, 0x86, 0x86, 0x86, 0x86, 0x46, 0x46, 0x46, 0x46, 0x46, // 27
  0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x26, 0x26, 0x26, 0x26, 0x26, 0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0x66, 0x66, 0x66, 0x66, 0x66, // 28
  0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0x16, 0x16, 0x16, 0x16, 0x16, 0x96, 0x96, 0x96, 0x96, 0x96, 0x56, 0x56, 0x56, 0x56, 0x56, // 29
  0xD6, 0xD6, 0xD6, 0xD6, 0xD6, 0x36, 0x36, 0x36, 0x36, 0x36, 0xB6, 0xB6, 0xB6, 0xB6, 0xB6, 0x76, 0x76, 0x76, 0x76, 0x76, // 30
  0xF6, 0xF6, 0xF6, 0xF6, 0xF6, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x8E, 0x8E, 0x8E, 0x8E, 0x8E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, // 31
  0xCE, 0xCE, 0xCE, 0xCE, 0xCE, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0xAE, 0xAE, 0xAE, 0xAE, 0xAE, 0x6E, 0x6E, 0x6E, 0x6E, 0x6E, // 32
  0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
  //0x1F, 0xAC,
  // 1      2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};



//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void Delay (void);
void EPD_Display(void);

void eeWrite(void)
{
 DATAEE_WriteByte(eeDATASIZE , DataSize );
 DATAEE_WriteByte(eeTAGSIZE ,tagSize);
 DATAEE_WriteByte(eeTSMCFLAG,tsmcFlag);
 DATAEE_WriteByte(eeACONFIG ,aconfigFlag);
 DATAEE_WriteByte(eeGIEE, gieeFlag);
 
 for (i=0; i<CHECKSUM_SIZE;i++)
 DATAEE_WriteByte((eeCHECKSUM_BUF+i), CHECKSUM_BUF[i]);

}
void eeInit(void)
{
	DataSize=   DATAEE_ReadByte(eeDATASIZE);
	tagSize=   DATAEE_ReadByte(eeTAGSIZE);
	tsmcFlag=   DATAEE_ReadByte(eeTSMCFLAG);
	aconfigFlag=   DATAEE_ReadByte(eeACONFIG);
	gieeFlag=   DATAEE_ReadByte(eeGIEE);
    for (i=0; i<CHECKSUM_SIZE;i++)
     CHECKSUM_BUF[i]=DATAEE_ReadByte((eeCHECKSUM_BUF+i));
#ifdef DATA_LOGGER2    
    // s2EEPFOM_lnd= (int) (DATAEE_ReadByte(S2EEPFOM_H)<<8)+(DATAEE_ReadByte(S2EEPFOM_L) & 0x00FF);
#endif     
}     
                       
 //           LCD_Set_Cursor(0,12);
//            puthexLCD(adcResult>>8);
 //           puthexLCD(adcResult);
//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
void EPD_Display(void)
{
#ifdef    USE_SLEEP
    EPD_Wait=  EPD_WAIT; 
 #endif        
   ClearLCD();
#ifdef DEMO_ONLY   
   setMode();
#endif   
   EPD_index=0;
   LCD_Set_Cursor(0,0);
   LCD_index=Line_index;
   while(displayP()); 
  // displayP();
#ifdef TEST_MULTI  
  EPD_print(); //20170913
  EPD_flash();
#endif  
   
  i=0;
}

uint16_t dataeeAddr = 0x00;
uint8_t dataeeData = 0x00;
            volatile unsigned char value ;
            uint16_t address = 0x00;
void getEEPROM(void)
{
#ifdef  EEPROM_DEBUG
    for (address=0;address<WRITE_EEPROM_SIZE; address++)
     LCD_BUF0[address]  = DATAEE_ReadByte(address); 
    i=0;
#endif    
}   
void main_2 (void) {
    
	    unsigned char tCC; 

     rs232_s =0;
	findFC=0;
	idle=0;
	idle2=0;
    rx_bytes=0;
    rx_index=0;
     LED_ON();
	 
#ifdef  DATA_LOGGER 
  if (tCC == 0xFF )
      dataLogger=1;
  #endif
 
 
#ifdef USE_EEPROM
            if(!BUTTON_DOWN ||  !BUTTON_UP )
             {  
    	
               for (address=0;address<WRITE_EEPROM_SIZE; address++)
               DATAEE_WriteByte(address, LCD_BUF00[address]);
                
               while (!BUTTON_DOWN ||  !BUTTON_UP ) ;
            }
            else
            {
                value =   DATAEE_ReadByte(address); 
                if (1)//value == 0xFF)
                {
                    for (address=0;address<WRITE_EEPROM_SIZE; address++)
                    DATAEE_WriteByte(address, LCD_BUF00[address]);
                   DATAEE_WriteByte(S2EEPFOM_H,0);
                   DATAEE_WriteByte(S2EEPFOM_L,0);
                }
                else
                {
                    eeInit();
                }
            }
#endif 
     //
 //  volatile unsigned char value = 0xFF;
 //  unsigned char address = 0x300;
 //  eeprom_write(address, value);     // Writing value 0x9 to EEPROM address 0xE5 
  //  DATAEE_WriteByte(address, value);
 // value2 =   DATAEE_ReadByte(address);    // Reading the value from address 0xE5
    
  //  while ( value != value2) ;
    
    RS232_Disable (); //20170913
	//while() SPI2_Read8bit(); 
    for(i=0;i<1000;i++); // wait EPD
     EPD_Show();
     i=0;
     //    EPD_Show();
      //  for(i=0;i<10;i++); // wait EPD
     //     SPI2_8bit(0xAA);
        //    EPD_print();
          //                 EPD_flash();
       for(i=0;i<10000;i++); // wait EPD
     //  for(i=0;i<60000;i++);
       showTitle();
      //   SPI2_8bit(' ');
   #ifdef  DATA_LOGGER2   
  //  ClearLCD();
    LCD_Set_Cursor(1,0);
     DataCount=0;
    LCD_index=1;
    EPD_index=1;
	Line_index=1;
 #endif	

 #ifdef BLE_MODE 
        sendBLE ();
#endif
      while (1) {
          if ( EUSART1_DataReady)
                {
                    unsigned char rc;
        
                    IO_RC2_Toggle();
                    if(setFontFlag)
                    {
                        setFontFlag=0;
                        setFont8();
                    }
                    rc=EUSART1_Read();
                    idle =0;
                    idle2 =0;
                    if( lcd_log_flag)
                        log_data (~(rc));
                        i=1; 
                    store_data (rc);

                }   
           else
                {
                        idle++;
                        if (idle> MAX_WAIT_TIME2)
                        {


                            idle=0;
                            idle2++;
                           if ( idle2>MAX_WAIT_TIME)
                            {
                                IO_RC2_SetLow();
                                 FinishFlag=1;
                                 if(!sysReady)   
                                  {
                                     sysReady=1;  
                                   //  SPI2_8bit(0xAA);
                                   //  EPD_print();
                                    // EPD_flash();
                                  //   EPD_Show();
                                  }
                            idle=0;
                            idle2=0;
#ifdef DATA_LOGGER2 
                            DataCount=0; //20171120
#endif                            
                           }
                        }
                        if(buttonUpF)
                        {
                            twoButtonFlag=0;
                             while ( !BUTTON_UP ) 
                             {    
                                if(!BUTTON_DOWN  )
                                {
                                    twoButtonFlag=1;
                                    break;
                                }
                             }   
                            
                            if (twoButtonFlag)
                            {
                                for (address=0;address<WRITE_EEPROM_SIZE; address++)
                                 DATAEE_WriteByte(address, LCD_BUF00[address]);
                                 DataSize=192;
                                 MultiFlag=0;
                                tagSize=244;
                                tsmcFlag=1;
                                buttonDownF=0;
                                
                            }
                            else
                              buttonUp();
                            buttonUpF=0;
                             idle=0;
                            idle2=0; 
                        }
                        if(buttonDownF)
                        {
                             twoButtonFlag=0;
                             while ( !BUTTON_DOWN ) 
                             {    
                                if(!BUTTON_UP  )
                                {
                                    twoButtonFlag=1;
                                    break;
                                }
                             }   
                            
                            if (twoButtonFlag)
                            {
                                for (address=0;address<WRITE_EEPROM_SIZE; address++)
                                 DATAEE_WriteByte(address, LCD_BUF00[address]);
                                 DataSize=192;
                                 MultiFlag=0;
                                tagSize=244;
                                tsmcFlag=1;
                                buttonUpF=0;
                            }
                            else
                             buttonDown();
                            buttonDownF=0;
                             idle=0;
                            idle2=0; 
                        }

                }// RS232 else
  //////////////////////////////////////////////////////////////////// 
                if(FinishFlag)
                {
                    FinishFlag=0;
                    if( findFC || findB5 )
                	{  
                            if  ( lcd_log_flag)
                            {   
                               if (MultiFlag)
                               {
                                  for_test=0;
                                  checksum();
                               }   
                               else
                               {
                                   DataSize=GetDataSize(); 
                                   checksum();


                               }
                              //  display ();
                                DisplayFlag=1;
                                LCD_index=0;
                            }// LCD_log
                        
				            
                            rs232_s =0;
                    	    findFC=0;
                            findB5=0;
                            lcd_log_flag=0;
                            pc_buf_index=0;
                                
                      
					}//Fin FC
                  idle=0;
				  idle2=0; 
                  if (lcdOnFlag)
                  {
                     lcdOnFlag=0; 
                      #ifndef  DATA_LOGGER2  
                     ClearLCD();
                      #endif
                  }
                  
                    if (DisplayFlag)
                    {
                        DisplayFlag=0;
                        EPD_PowerOn();

                      #ifdef BLE_MODE 
                          sendBLE ();
                      #endif
                      Line_index=0; //20170913
                       #ifdef  DATA_LOGGER2  
                            EPD_print();
                            EPD_flash();
                       #else
                          EPD_Display();
                       #endif
                      i=0;
                      
                    }
#ifdef MASTER_MODE
  //  if( rs232_s !=300     )
                 Master_checksum ();
    {
        for ( i=0; i<4; i++)
        {
           putdata (master_header[i]);
           for (j=0;j< UART_DELAY;j++);
         }
         //    for ( i=0; i<20; i++)
          //         {
          //             putdata (master_control[i]);
          //             for (j=0;j< UART_DELAY;j++);
          //              
          //        }
       // step300();
         rs232_s=0;
    }
#endif                  
                  ////////////////////////
                     #ifdef  USE_SLEEP
                    BAUD1CONbits.WUE=1;
                   if (!DisplayFlag)
                   {
                        if (EPD_Wait)
                           EPD_Wait--;
                        else
                        {
                         EPD_PowerOff();
                       //  RS232_WakeUp ();
                         setFontFlag=1;
                        IO_RC2_SetHigh();
                        SLEEP();
                        IO_RC2_SetLow();
#ifdef  FONTS8 
                         EPD_PowerOn();
                         setFont8();
#endif                         
                        }
                   }
                  #endif
               

               }    // Idle time out
        ////////////////////////////////////////////////////////////////////  
   
       
      }// while
    
}



//-----------------------------------------------------------------------------
// putchar
//-----------------------------------------------------------------------------
//
// Return Value : UART0/1 buffer value
// Parameters   : character to be transmitted across UART0/1
//
// This is an overloaded fuction found in the stdio library.  When the
// function putchar is called, either by user code or through calls to stdio
// routines such as printf, the following routine will be executed instead 
// of the function located in the stdio library.
//
// The function checks the UART global variable to determine which UART to 
// use to receive a character.
//
// The routine expands '\n' to include a carriage return as well as a 
// new line character by first checking to see whether the character  
// passed into the routine equals '\n'.  If it is, the routine waits for 
// SCON0_TI/TI1 to be set, indicating that UART 0/1 is ready to transmit another 
// byte.  The routine then clears SCON0_TI/TI1 bit, and sets the UART0/1 output 
// buffer to '0x0d', which is the ASCII character for carriage return.
//
// The routine the waits for SCON0_TI/TI1 to be set, clears SCON0_TI/TI1, and sets
// the UART output buffer to <c>.  
//
//-----------------------------------------------------------------------------



unsigned char putdata (unsigned char c)
{
#ifdef _32BIT_
    //#ifdef  SEND_UART2  
//     UART2_Write( (const uint8_t )c);
//#else
     UART1_Write( (const uint8_t )c);
//#endif 
#else
     // while (!(SCON1 & 0x02));         // wait until UART1 is ready to transmit
     //  SCON1 &= ~0x02;                  // clear TI1 interrupt flag
      //SBUF1 = c;
        EUSART1_Write(c);  // 呼叫 app.c 的 UART1 函數來送出字串
#endif        
      return (c);              // output <c> using UART 1


}


//-----------------------------------------------------------------------------
// _getkey
//-----------------------------------------------------------------------------
//
// Return Value : byte received from UART0/1
// Parameters   : none

// This is an overloaded fuction found in the stdio library.  When the
// function _getkey is called, either by user code or through calls to stdio
// routines such as scanf, the following routine will be executed instead 
// of the function located in the stdio library.
//
// The function checks the UART global variable to determine which UART to 
// use to receive a character.
//
// The routine waits for SCON0_RI/RI1 to be set, indicating that a byte has
// been received across the UART0/UART1 RX line.  The routine saves the 
// received character into a local variable, clears the SCON0_RI/RI1 interrupt
// flag, and returns the received character value.
//
//-----------------------------------------------------------------------------

#if defined __C51__

char getchar ()  {
  char c;

  if (UART == 0) {
    while (!SCON0_RI);                      // wait until UART0 receives a character

    getData = SBUF0;                         // save character to local variable
    c=getData;
    SCON0_RI = 0;                           // clear UART0 receive interrupt flag
    return (c);                        // return value received through UART0
  }

  else if (UART == 1) {
    while (!(SCON1 & 0x01));           // wait until UART1 receives a character
    getData = SBUF1;                         // save character to local variable
    c=getData;
    SCON1 &= ~0x01;                    // clear UART1 receive interrupt flag
    return (c);                        // return value received through UART1
  }
  else {
	  return 0xff;
  }
}

#elif defined __ICC8051__
ccc
int getchar(void){
  char c;

  if (UART == 0) {
    while (!SCON0_RI);                      // wait until UART0 receives a character
    c = SBUF0;                         // save character to local variable
    SCON0_RI = 0;                           // clear UART0 receive interrupt flag
    return (c);                        // return value received through UART0
  }

  else if (UART == 1) {
    while (!(SCON1 & 0x01));           // wait until UART1 receives a character
    c = SBUF1;                         // save character to local variable
    SCON1 &= ~0x01;                    // clear UART1 receive interrupt flag
    return (c);                        // return value received through UART1
  }
  else {
	  return 0xff;
  }
}

#endif


//-----------------------------------------------------------------------------
// Delay
//-----------------------------------------------------------------------------
//
// Return Value : none
// Parameters   : none
//
// Used for a small pause of approximately 40 us.
//
//-----------------------------------------------------------------------------

void Delay(void)
{
  unsigned int x;
   for(x = 0;x < 500;x)
      x++;
}

unsigned char nowCC;
void store_data (unsigned char cc)
{
      nowCC =cc;
#ifdef DATA_LOGGER2
//lineCnt++;
//if (lineCnt > 5 )
//{
//    LCD_Set_Cursor(EPD_index++,0);
//    lineCnt=0;
//}
      DataCount++;
#ifdef  USE_RANGE 
       if((DataCount<LOG_LOW) || (DataCount>LOG_HIGH))  
#endif  
       {
            putcLCD(Hex2ASC ((nowCC >> 4) ));
            putcLCD(Hex2ASC ((nowCC & 0x0F) ));
            putcLCD(' ');
       }
//      if((s2EEPFOM_lnd+S2EEPFOM_START)<S2EEPFOM_MAX)  
     //  {
          
      //     DATAEE_WriteByte((s2EEPFOM_lnd+S2EEPFOM_START),Hex2ASC ((nowCC >> 4) ));
        //   s2EEPFOM_lnd++;
         //  DATAEE_WriteByte((s2EEPFOM_lnd+S2EEPFOM_START),(nowCC & 0x0F) );
         //  s2EEPFOM_lnd++;
         //  DATAEE_WriteByte((s2EEPFOM_lnd+S2EEPFOM_START),0x20);
        //   s2EEPFOM_lnd++;
    //   }
     // else
    //  {
       //    s2EEPFOM_lnd =0;
     //      DATAEE_WriteByte(S2EEPFOM_H,0);
     //      DATAEE_WriteByte(S2EEPFOM_L,0);
   //   }
#endif
	   switch (rs232_s)
	   {
	      case 0:
	      {
          #ifdef MASTER_MODE   
              if (nowCC==0xCA)
	          {
                  idle=0;
				  idle2=0; 
#ifdef STOCKER 
                   for ( i=0; i<15 ; i++)
                   {
                       putdata (master_contro2[i]);
                       for (j=0;j< UART_DELAY;j++);
                       //  for (j=0;j< 5000;j++);
                   }
                   rs232_s=302;
#else    			   
	               for ( i=0; i<20 ; i++)
                   {
                       putdata (master_control[i]);
                       for (j=0;j< UART_DELAY;j++);
                       //  for (j=0;j< 5000;j++);
                 }
                   rs232_s=300;
#endif    						
              }   
             
             // step300();
          #else
/*		 
   if (   nowCC==0xFC)
	  {
              //     putdata (0xFF);
           
                 
	    		  wait_bytes=0;
	    		  findFC=1;
	    		  idle=0;
	    		  idle2=0;
	    		  rs232_s=108;
              //     rs232_s=1;
                  rs232_FC=0;
	   }
 */ 
           //    if (   nowCC==0xDF) // STAUTS 20171122
           //     {
            //      putdata (0xCA);
           //
                 
	    	//	  wait_bytes=0;
	    		//  findFC=1;
	    	//	  idle=0;
	    	//	  idle2=0;
	    	//	  rs232_s=200;
             // 
              //    rs232_FC=0;
	         //  }
               if (cc==0xB5)
	    	  {
              //     putdata (0xFF);
           
                 
	    		  wait_bytes=0;
	    		  findFC=1;
	    		  idle=0;
	    		  idle2=0;
	    		  rs232_s=110;
              //     rs232_s=1;
                  rs232_FC=0;
	    	  }
            #endif     
	      }
	      break;
	      case 1:
              step01();
	       break;
	      case 2:
              step02(cc);
	      break;
	      case 3:  //// Send header
              step03();
		     break;
	      case 7:
	          step07();
	      break;
	      case 8:
              step08();
	      break;
          case 81:
              step081();
	      break;
  //////////////////////////////////////////////////////////////                        
          case 9:
              step09();
          break;
          case 10:
              step10();
	      break;      
  ///////////////////////////////////////////////////////////////////                        
         case 11:
             step11();
          break;                 
         case 12:
              step12();
	     break;               
  ///////////////////////////////////////////////////////////////////                        
         case 13:
               step13();
          break;                 
           case 14:
                step14();
	       break;                                      
           case 18 :
               step18();
           break;
           case 101 :
           break;
           case 102 :
              step102();
           break;
           case 103 :
              step103();
          case 105:
             step105();
             break;
          case 106:
             step106();
             break;		
          case 108:
             step108();   
          case 110:
             step110();
           break;
         case 199:
             step199();
             break;
          case 200:
             step200();
           break;
#ifdef MASTER_MODE           
          case 300:
             step300();
           break;
           case 301:
             step301();
          case 302:
             step302();  
           break;        
#endif           
          case 400:
             step400();  
           break;  					
	      default:
            switchdefault (cc);
	       break;
	   }
} // end od store_data
void step108(void)
{
   if ( nowCC==0xB5 )
	  {
              //     putdata (0xFF);
           
                 
	    		  wait_bytes=0;
	    		  findFC=1;
	    		  idle=0;
	    		  idle2=0;
	    		  rs232_s=110;
              //     rs232_s=1;
                  rs232_FC=0;
	   }
}  
void step110(void)
{
 wait_bytes++;
    if ((nowCC == 0xFF) )
    {
        
#ifdef DATA_LOGGER2       
putcLCD('H');
putcLCD(' ');
#endif
         putdata (0xca);
//    putdata (0x60);
          wait_bytes=0;
          findFC=1;
          idle=0;
          idle2=0;
          rs232_s=1;
          rs232_FC=0;
    }
    else
     rs232_s=0;   
 }     

void step01(void)
{
 wait_bytes++;
#ifdef DATA_LOGGER20
//lineCnt++;
//if (lineCnt > 5 )
//{
//    LCD_Set_Cursor(EPD_index++,0);
//    lineCnt=0;
//}

            
            putcLCD(Hex2ASC ((nowCC >> 4) ));
            putcLCD(Hex2ASC ((nowCC & 0x0F) ));
            putcLCD(' ');

#endif
   
   //   if ( nowCC==0xB5 )
     if ( nowCC==0xFC ) //20171120
	  {
              //     putdata (0xFF);
           
                 
	    		  wait_bytes=0;
	    		  findFC=1;
	    		  idle=0;
	    		  idle2=0;
	    		  rs232_s=108;  //20171120
#ifdef DATA_LOGGER2                   
                  DataCount=0; //20171120
#endif                  
              //     rs232_s=1;
                  rs232_FC=0;
	   }
 if ((nowCC == PC_SEND) ||  (nowCC == PC_REV))
  {
     aconfigFlag=0;
	 stockerFlag=0;			
     erack=0;
    // gieeFlag=0;
   //  tagSize=0;
    // tsmcFlag=0;
    if  (nowCC == PC_SEND)
	    recFlag=1;
    else
        recFlag=0;
  //   gieeFlag=1; 
      step08F=0;   
	 wait_bytes=0;
	 rs232_s=2;
   PC_h1_index=0;
   PC_h2_index=0;
   PC_h3_index=0;
   PC_h4_index=0;
  }
 
    if ( nowCC==0xDF) // Display On FOR TSMC Command
    {
        #ifdef DATA_LOGGER2       
        putcLCD('D');
        putcLCD(' ');
        #endif
       rs232_s=199;
        wait_bytes=0;
        for (j=0;j<1000;j++);
       	putdata (0x60);
       return;
    }
}
void step02(unsigned char cc)
{
 wait_bytes++;
 if(PC_h1_index<HEADER_SIZE)
    PC_H1[PC_h1_index++]=cc;
 if (cc==0xEF)
	recFlag=0;
#ifdef DATA_LOGGER20
//lineCnt++;
//if (lineCnt > 5 )
//{
//    LCD_Set_Cursor(EPD_index++,0);
//    lineCnt=0;
//}

      
            putcLCD(Hex2ASC ((cc >> 4) ));
            putcLCD(Hex2ASC ((cc & 0x0F) ));
            putcLCD(' ');
 
#endif
 if((wait_bytes==FORMAT_START) && (cc==FORMAT_CODE) )// format
 {        

 #ifdef DATA_LOGGER2       
putcLCD('F');
putcLCD(' ');
#endif
            rs232_s=105;
            wait_bytes=0;
            for (j=0;j<30000;j++);
           	putdata (0x60);
            return;
}
 if(wait_bytes==(CMD_LOC-1)) ///20170926
{
 if (cc==0xBB) // AConfig Write
      gieeFlag=0;
// else if   (cc==0x7F) // AConfig read
  //    gieeFlag=0;
}
 
if(wait_bytes==CMD_LOC)
{
#ifdef DATA_LOGGER2       
putcLCD('L');
putcLCD(' ');
#endif
    if (!recFlag)  //20171126
    {
#ifdef TEST_STOCKER        
	  if (cc==0x6E ) // STOCKER 20171124
#else        
   if (cc==0x6E) // STOCKER
#endif      
    // if ((cc==0x63) && (wait_bytes==SSS0))// ((wait_bytes!=SSS7) && (wait_bytes!=SSS8)) ) // 12 Bytes 11 fix 516 09 fix 568
     {
        #ifdef DATA_LOGGER2       
        putcLCD('V');
        putcLCD(' ');
        #endif
       rs232_s=106;
        wait_bytes=0;
  //      for (j=0;j<10000;j++);
         for (j=0;j<1000;j++); //20171122
       	putdata (0x60);
       return;
     }
    else
 #ifdef TEST_STOCKER        
	  if (0) //cc==0x0B ) // STOCKER 20171124
#else         
     if (cc==0x63 ) // 20170913
#endif         
    // if ((cc==0x63) && (wait_bytes==SSS0))// ((wait_bytes!=SSS7) && (wait_bytes!=SSS8)) ) // 12 Bytes 11 fix 516 09 fix 568
     {
        #ifdef DATA_LOGGER2   
         if(cc==0x63)
            putcLCD('R');
         else
            putcLCD('A'); 
        putcLCD(' ');
        #endif
        wait_bytes=PHASE2;
        recFlag=0;
        lcd_buf_index=0;
        step08F=1;
        gieeFlag=0;
         aconfigFlag=1;
     }
     else
#ifdef TEST_STOCKER        
	  if (1) //cc==0x0B ) // STOCKER 20171124
#else          
     if (cc==0x0B ) // STOCKER 20171124
#endif         
    // if ((cc==0x63) && (wait_bytes==SSS0))// ((wait_bytes!=SSS7) && (wait_bytes!=SSS8)) ) // 12 Bytes 11 fix 516 09 fix 568
     {
   
        #ifdef DATA_LOGGER2  
 		  putcLCD('S');
          putcLCD(' ');
        #endif
		stockerFlag=1;
        wait_bytes=PHASE2;
        recFlag=0;
        lcd_buf_index=0;
        step08F=1;
        gieeFlag=0;
         aconfigFlag=0;
        
     }
     else	 
     if (cc==0xBB ) // 20170913
    // if ((cc==0x63) && (wait_bytes==SSS0))// ((wait_bytes!=SSS7) && (wait_bytes!=SSS8)) ) // 12 Bytes 11 fix 516 09 fix 568
     {
       erack=1;
        #ifdef DATA_LOGGER2    
        putcLCD('E');
        putcLCD(' ');
        #endif
        wait_bytes=PHASE2;
        recFlag=0;
        lcd_buf_index=0;
        step08F=1;
        gieeFlag=1;
         aconfigFlag=0;
     }
     else
     {
            #ifdef DATA_LOGGER2       
            putcLCD('O');
            putcLCD(' ');
            #endif
          wait_bytes=PHASE2;
          recFlag=0;
           gieeFlag=1;
          lcd_buf_index=0;
        
      
     }
    }
    else  //      if(!recFlag)
    {
      
         
             #ifdef DATA_LOGGER2       
            putcLCD('P');
            putcLCD(' ');
            #endif
             for (j=0;j<1000;j++); // 20170913
              putdata (0x60);
        
    }
         
  // Jason 20170810 for (j=0;j<30000;j++);
  // Jason 20170810 putdata (0x60);
}

 if (cc==0xF5)
 // if (cc==0xF5 && wait_bytes>=SSS9) // 12 Bytes
 {
     #ifdef    USE_SLEEP  //20170814
      EPD_Wait=  EPD_WAIT; 
      EPD_PowerOn();
     #endif
        #ifdef DATA_LOGGER2       
       putcLCD('W');
       putcLCD(' ');
       #endif
    wait_bytes=PHASE2;
	recFlag=1;
    lcd_buf_index=0;
    
 
    
   //20170926 for ( i=0; i<=ZERO_SIZE ; i++)
    //  LCD_BUF0[lcd_buf_index++]=0x00;
  //  memset(void * p1, int c, register size_t n);//
   //  memset(LCD_BUF0,0x00,(ZERO_SIZE+1);
#ifdef USE_EEPROM
    lcd_buf_index=0;
   // for ( i=0; i<=ZERO_SIZE ; i++)
   //     DATAEE_WriteByte((uint16_t)lcd_buf_index++, (uint8_t)0x00);
 //    lcd_buf_index=(ZERO_SIZE +1);
   ////  for ( i=0; i<SIZE1 ; i++)
//	   DATAEE_WriteByte(lcd_buf_index++,0x04);
  //   DATAEE_WriteByte(lcd_buf_index++,0x50);
  //   for ( i=0; i<5 ; i++)
	//   DATAEE_WriteByte(lcd_buf_index++,0x00);
 //   DATAEE_WriteByte(D0_LOC,0x0B);
#else   
     
    memset(LCD_BUF0,0x00,(ZERO_SIZE+1));
    lcd_buf_index=(ZERO_SIZE +1);
    for ( i=0; i<SIZE1 ; i++)
	  LCD_BUF0[lcd_buf_index++]=0x04;
      LCD_BUF0[lcd_buf_index++]=0x50;
       for ( i=0; i<5 ; i++)
	  LCD_BUF0[lcd_buf_index++]=0x00;
       LCD_BUF0[D0_LOC]=0x0B;
#endif
       
	 lcd_buf_index=0;
 }

 if(wait_bytes>=PHASE2)
 {
    
   wait_bytes=0;    
   if (recFlag)
   {
  //     for ( i=0; i<1000 ; i++);
   //   putdata (0x60);
       if (PC_H1[MULTI_index]== MULTI_BYTE)   
       {
          DataSize=(decode((0xFF-(PC_H1[DATASIZE_h])))<<8 )+decode((0xFF-(PC_H1[DATASIZE_l])));
          DataSize--; // Skip 1 
          MultiFlag=1;  
        } 
        else
        {
         //20170919  DataSize=0;
          DataSize=(decode((0xFF-(PC_H1[DATASIZE_hh])))<<8 )+decode((0xFF-(PC_H1[DATASIZE_ll])));
          DataSize--; // Skip 1  include checksum
          
          tsmcFlag=0;
           gieeFlag=0;
          if ( DataSize >200 && DataSize <=TSMC_SIZE)
              tsmcFlag=1;
          else if (DataSize == ASIM_SIZE)
              gieeFlag=1;
          
          MultiFlag=0;  
        } 
       rs232_FC=1;
        rs232_s=18;
      //  step18();  //20170524     
     //  rs232_s=101;
    } 
	else
    {
#ifdef TEST_STOCKER
			  if (stockerFlag && stockerFirst ) // STOCKER 20171124
			   rs232_s=400;
               else   
#endif                   
	           rs232_s=3;
  //    putdata (0x60);   //20170913
    }
   }
}
void step022(unsigned char cc)
{
 wait_bytes++;
 if(PC_h1_index<HEADER_SIZE)
    PC_H1[PC_h1_index++]=cc;
 if (cc==0xEF)
	recFlag=0;
#ifdef DATA_LOGGER20
//lineCnt++;
//if (lineCnt > 5 )
//{
//    LCD_Set_Cursor(EPD_index++,0);
//    lineCnt=0;
//}

      
            putcLCD(Hex2ASC ((cc >> 4) ));
            putcLCD(Hex2ASC ((cc & 0x0F) ));
            putcLCD(' ');
 
#endif
 if((wait_bytes==FORMAT_START) && (cc==FORMAT_CODE) )// format
 {        

 #ifdef DATA_LOGGER2       
putcLCD('F');
putcLCD(' ');
#endif
            rs232_s=105;
            wait_bytes=0;
            for (j=0;j<30000;j++);
           	putdata (0x60);
            return;
}
 if(wait_bytes==(CMD_LOC-1)) ///20170926
{
 if (cc==0xBB) // AConfig Write
      gieeFlag=0;
// else if   (cc==0x7F) // AConfig read
  //    gieeFlag=0;
}
 
if(wait_bytes==CMD_LOC)
{

 
   if (cc==0x6E) // LCD ON
    // if ((cc==0x63) && (wait_bytes==SSS0))// ((wait_bytes!=SSS7) && (wait_bytes!=SSS8)) ) // 12 Bytes 11 fix 516 09 fix 568
     {
        #ifdef DATA_LOGGER2       
        putcLCD('V');
        putcLCD(' ');
        #endif
       rs232_s=106;
        wait_bytes=0;
  //      for (j=0;j<10000;j++);
         for (j=0;j<1000;j++); //20171122
       	putdata (0x60);
       return;
     }
    else
     if (cc==0x63 ) // 20170913
    // if ((cc==0x63) && (wait_bytes==SSS0))// ((wait_bytes!=SSS7) && (wait_bytes!=SSS8)) ) // 12 Bytes 11 fix 516 09 fix 568
     {
        #ifdef DATA_LOGGER2   
         if(cc==0x63)
            putcLCD('R');
         else
            putcLCD('A'); 
        putcLCD(' ');
        #endif
        wait_bytes=PHASE2;
        recFlag=0;
        lcd_buf_index=0;
        step08F=1;
        gieeFlag=0;
         aconfigFlag=1;
     }
     else
     if (cc==0xBB ) // 20170913
    // if ((cc==0x63) && (wait_bytes==SSS0))// ((wait_bytes!=SSS7) && (wait_bytes!=SSS8)) ) // 12 Bytes 11 fix 516 09 fix 568
     {
       erack=1;
        #ifdef DATA_LOGGER2   
         
        putcLCD('E');
       
        putcLCD(' ');
        #endif
        wait_bytes=PHASE2;
        recFlag=0;
        lcd_buf_index=0;
        step08F=1;
        gieeFlag=1;
         aconfigFlag=0;
     }
     else
     {
         if(!recFlag)
         {
          wait_bytes=PHASE2;
          recFlag=0;
           gieeFlag=1;
          lcd_buf_index=0;
         }
         else
         {
             for (j=0;j<1000;j++); // 20170913
              putdata (0x60);
         }
            #ifdef DATA_LOGGER20    
            putcLCD(Hex2ASC ((cc >> 4) ));
            putcLCD(Hex2ASC ((cc & 0x0F) ));
            putcLCD(' ');
            #endif
     }
  // Jason 20170810 for (j=0;j<30000;j++);
  // Jason 20170810 putdata (0x60);
}

 if (cc==0xF5)
 // if (cc==0xF5 && wait_bytes>=SSS9) // 12 Bytes
 {
     #ifdef    USE_SLEEP  //20170814
      EPD_Wait=  EPD_WAIT; 
      EPD_PowerOn();
     #endif
        #ifdef DATA_LOGGER2       
       putcLCD('W');
       putcLCD(' ');
       #endif
    wait_bytes=PHASE2;
	recFlag=1;
    lcd_buf_index=0;
    
 
    
   //20170926 for ( i=0; i<=ZERO_SIZE ; i++)
    //  LCD_BUF0[lcd_buf_index++]=0x00;
  //  memset(void * p1, int c, register size_t n);//
   //  memset(LCD_BUF0,0x00,(ZERO_SIZE+1);
#ifdef USE_EEPROM
    lcd_buf_index=0;
   // for ( i=0; i<=ZERO_SIZE ; i++)
   //     DATAEE_WriteByte((uint16_t)lcd_buf_index++, (uint8_t)0x00);
 //    lcd_buf_index=(ZERO_SIZE +1);
   ////  for ( i=0; i<SIZE1 ; i++)
//	   DATAEE_WriteByte(lcd_buf_index++,0x04);
  //   DATAEE_WriteByte(lcd_buf_index++,0x50);
  //   for ( i=0; i<5 ; i++)
	//   DATAEE_WriteByte(lcd_buf_index++,0x00);
 //   DATAEE_WriteByte(D0_LOC,0x0B);
#else   
     
    memset(LCD_BUF0,0x00,(ZERO_SIZE+1));
    lcd_buf_index=(ZERO_SIZE +1);
    for ( i=0; i<SIZE1 ; i++)
	  LCD_BUF0[lcd_buf_index++]=0x04;
      LCD_BUF0[lcd_buf_index++]=0x50;
       for ( i=0; i<5 ; i++)
	  LCD_BUF0[lcd_buf_index++]=0x00;
       LCD_BUF0[D0_LOC]=0x0B;
#endif
       
	 lcd_buf_index=0;
 }

 if(wait_bytes>=PHASE2)
 {
    
   wait_bytes=0;    
   if (recFlag)
   {
  //     for ( i=0; i<1000 ; i++);
   //   putdata (0x60);
       if (PC_H1[MULTI_index]== MULTI_BYTE)   
       {
          DataSize=(decode((0xFF-(PC_H1[DATASIZE_h])))<<8 )+decode((0xFF-(PC_H1[DATASIZE_l])));
          DataSize--; // Skip 1 
          MultiFlag=1;  
        } 
        else
        {
         //20170919  DataSize=0;
          DataSize=(decode((0xFF-(PC_H1[DATASIZE_hh])))<<8 )+decode((0xFF-(PC_H1[DATASIZE_ll])));
          DataSize--; // Skip 1  include checksum
          
          tsmcFlag=0;
           gieeFlag=0;
          if ( DataSize >200 && DataSize <=TSMC_SIZE)
              tsmcFlag=1;
          else if (DataSize == ASIM_SIZE)
              gieeFlag=1;
          
          MultiFlag=0;  
        } 
       rs232_FC=1;
        rs232_s=18;
      //  step18();  //20170524     
     //  rs232_s=101;
    } 
	else
    {
	  rs232_s=3;
  //    putdata (0x60);   //20170913
    }
   }
}
void step03(void)
{
    unsigned char fourSize;
     UART1_WaitEmpty ();
  for (j=0;j<1600;j++);
  wait_bytes++;
    putdata (0x60);   //20170925
 // if(wait_bytes>=PHASE3 )
  {
	for ( i=0; i<3 ; i++)
	{
   
         if  (erack) //20170918
			putdata (erack_Header[i]);
	    else
		 if  (gieeFlag) //20170918
			putdata (Header0[i]);
	    else
			putdata (Header[i]); 
	   for (j=0;j< UART_DELAY;j++);
	 }
	  rs232_s=4;
	   wait_bytes=0;
   }
  ////////////////////////
  if (stockerFlag ) //20171126
  {
	  putdata (0x0F);
  }
//	for (j=0;j<1000;j++); //20170813				   
   wait_bytes++;
 // if(wait_bytes>=PHASE4 )
  {
	for ( i=0; i<ZERO_SIZE ; i++)
	{
      if(MultiFlag)
         putdata (0x00);  
       else
#ifdef USE_EEPROM
       putdata (DATAEE_ReadByte(lcd_buf_index++));
#else          
       putdata (LCD_BUF0[lcd_buf_index++]);
#endif      
       for (j=0;j< UART_DELAY;j++);
	} 
	wait_bytes=0;
	rs232_s=6; //end
  }
 //////////////////////////////////
 // for (j=0;j<600;j++);  //20170813	remove
  wait_bytes++;
//  if(wait_bytes > (unsigned char)PHASE6 )
  {
    //for ( i=0; i<(FOUR_SIZE+1) ; i++) //20170918
      if (gieeFlag || aconfigFlag || stockerFlag )
      {
        for ( i=0; lcd_buf_index <(tagSize -2) ; i++)
        {
          if(MultiFlag)   
           putdata (LCD_BUF1[lcd_buf_index++]);
          else
#ifdef USE_EEPROM
       putdata (DATAEE_ReadByte(lcd_buf_index++));
#else          
       putdata (LCD_BUF0[lcd_buf_index++]);
#endif 
           for (j=0;j< UART_DELAY;j++);
        }
      }
      else    
      {
         
        for ( i=0; i<(FOUR_SIZE+1) ; i++)
        {
          if(MultiFlag)   
           putdata (LCD_BUF1[lcd_buf_index++]);
          else
 #ifdef USE_EEPROM
       putdata (DATAEE_ReadByte(lcd_buf_index++));
#else          
       putdata (LCD_BUF0[lcd_buf_index++]);
#endif 
           for (j=0;j< UART_DELAY;j++);
        }
      }
	wait_bytes=0;
	rs232_s=7; //end
    step07 (); //20171121
  }
}
void step07 (void)
{
//20170926 for (j=0;j<6000;j++);
//20170926 for (j=0;j<600;j++);
 wait_bytes++;
     if(1) //gieeFlag) //20170918
    {
         UART1_WaitEmpty ();
         
         if( aconfigFlag)
         {
             for(i=0;i< (ACONFIG_SIZE-tagSize); i++) // Add
                  putdata (0x04);
               for (j=0;j< CHECKSUM_DELAY;j++);
                putdata (   CHECKSUM_BUF[CHECKSUM4_L]);
                putdata (  CHECKSUM_BUF[CHECKSUM4_H] ); 
                 for(i=0;i< 13; i++) // Add
                  putdata (tail0[i]);
         }   
         else
             if(erack)
             {
                for (j=0;j< CHECKSUM_DELAY;j++);    
                putdata (   CHECKSUM_BUF[CHECKSUM3_L]);
                putdata (  CHECKSUM_BUF[CHECKSUM3_H] ); 
             }
         
			 else
				 if(stockerFlag)
			 {
            
               // for(i=0;i< 13; i++) // Add
                     if(!tsmcFlag)
                        putdata (0x30);
                for(i=0;i< 13; i++) // Add
                  putdata (Stocker_End[i]);
                for (j=0;j< CHECKSUM_DELAY;j++);
                putdata (   CHECKSUM_BUF[CHECKSUM6_L]);
                putdata (  CHECKSUM_BUF[CHECKSUM6_H] ); 
             }
             else
         {
            
                for(i=0;i< (TAG_SIZE-tagSize); i++) // Add
                  putdata (0x04);
                for (j=0;j< CHECKSUM_DELAY;j++);
                putdata (   CHECKSUM_BUF[CHECKSUM5_L]);
                putdata (  CHECKSUM_BUF[CHECKSUM5_H] ); 
         }
  //for ( i=0; i<2 ; i++)
//	{
	//  if(MultiFlag)   
  //       putdata (LCD_BUF1[lcd_buf_index++]);
 //     else
    //      putdata (LCD_BUF0[lcd_buf_index++]);
	//   for (j=0;j< UART_DELAY;j++);
	//}
   //  lcd_buf_index    -=2;
  //         for ( i=0; i<2 ; i++)
	//{
	//  if(MultiFlag)   
     //    putdata (LCD_BUF1[lcd_buf_index++]);
    //  else
      //    putdata (LCD_BUF0[lcd_buf_index++]);
	  // for (j=0;j< UART_DELAY;j++);
	//}
	wait_bytes=0;
    
#ifdef USE_OLD_MODE    
        rs232_s=100; //end 20171120 move back
#else
    rs232_s=102; //20171031
#endif    
    }
   
       else
 {
    for ( i=0; i<2 ; i++)
	{
	  if(MultiFlag)   
         putdata (LCD_BUF1[lcd_buf_index++]);
      else
 #ifdef USE_EEPROM
       putdata (DATAEE_ReadByte(lcd_buf_index++));
#else          
       putdata (LCD_BUF0[lcd_buf_index++]);
#endif 
	   for (j=0;j< UART_DELAY;j++);
	}
	wait_bytes=0;
   
	rs232_s=8; //end
  
  }
}
void step08(void)
{
  wait_bytes++;
  
  if( nowCC==0x57)
   {
      lcd_buf_index-=2;
       rs232_s=7; //end
   }
// for (j=0;j<600;j++);
 // if(wait_bytes>=PHASE7 ) //20170815
  if( 1) // step08F ) //&& nowCC==0x9F  ) //20170815
  {
    for ( i=0; i<8 ; i++)
	{
	  if(MultiFlag)   
        putdata (LCD_BUF1[lcd_buf_index++]);
       else
#ifdef USE_EEPROM
       putdata (DATAEE_ReadByte(lcd_buf_index++));
#else          
       putdata (LCD_BUF0[lcd_buf_index++]);
#endif 
	    for (j=0;j< UART_DELAY;j++);
	}
   	wait_bytes=0;
	rs232_s=81; //end
  }
    else
     {
      UART1_WaitEmpty ();
      // FinishFlag=1;  // 20170914
       rs232_s=100; //end
     }
}  
void step081(void)
{
   //  if(   nowCC==0x9F  ) //20170815
  {
      for ( i=0; i<5 ; i++)
	{
	  if(MultiFlag)   
        putdata (LCD_BUF1[lcd_buf_index++]);
       else
#ifdef USE_EEPROM
       putdata (DATAEE_ReadByte(lcd_buf_index++));
#else          
       putdata (LCD_BUF0[lcd_buf_index++]);
#endif 
	    for (j=0;j< UART_DELAY;j++);
	}
	wait_bytes=0;
    lcd_buf_index=0;
    if(MultiFlag)
    {
       UART1_WaitEmpty ();
     //   UART1_Clear ();
	  rs232_s=9; //end
    }
     else
     {
        UART1_WaitEmpty ();
        //FinishFlag=1;  // 20170914
       rs232_s=100; //end
     }
     }
}
void Empty_Delay (void)
{
      for (j=0;j<EMPTY_DELAY;j++);
}
void step09(void)
{
 wait_bytes++;
//  if( wait_bytes>=PHASE3 )
 if( nowCC== 0xB5 )
 {
     
    Empty_Delay ();
  
     sendHeadOut (DataSize,CHECKSUM_BUF[DATASIZE],1);
     wait_bytes=0;
     
	 rs232_s=10; //end
      step10();
 }  
}
void step10(void)
{
 wait_bytes++;
//if(wait_bytes>=PHASE7 )
 if (nowCC==0x7F)
 {
    
      Empty_Delay ();
    for (j=0;j<2000;j++);
    if (DataSize<=DATA_MAX_SIZE)
    { 
      unsigned int end=DataSize;
      sendDataOut (0 , end , CHECKSUM_BUF[CHECKSUM3_L] ,CHECKSUM_BUF[CHECKSUM3_H]); 
      /*
      for ( i=0; i<end ; i++)
      {
       //  if (decode(LCD_BUF[i]) < ENTER_CODE ) break; 
         putdata (LCD_BUF[i]);
          for (j=0;j<100;j++);
      }
    //  i++;
      for (j=0;j<6000;j++);
       putdata (CHECKSUM_BUF[CHECKSUM3_L]);
       for (j=0;j<100;j++);
       for (j=0;j<6000;j++);
       putdata (CHECKSUM_BUF[CHECKSUM3_H]);
       for (j=0;j<100;j++);
       
       
        for (j=0;j<60000;j++);
       */
      wait_bytes=0;
       FinishFlag=1;  
       rs232_s=100; //end
    }    
    else   
     if (DataSize<=DATA_MAX_SIZE2)
     {
         sendDataOut (0 ,DATA_MAX_SIZE , CHECKSUM_BUF[CHECKSUM1_L] ,CHECKSUM_BUF[CHECKSUM1_H]) ; 
         wait_bytes=0;
         rs232_s=11; //end
         UART1_WaitEmpty ();
        //  UART1_Clear ();
         step11();
     }
     else
     {
        sendDataOut (0 ,DATA_MAX_SIZE , CHECKSUM_BUF[CHECKSUM1_L] ,CHECKSUM_BUF[CHECKSUM1_H]) ;
        wait_bytes=0;
        rs232_s=11; //end
        UART1_WaitEmpty ();
       //  UART1_Clear ();
        step11();
     }
}// end wait
}
void step11(void)
{
  wait_bytes++;
 // if(wait_bytes>=PHASE3 )
 if( nowCC== 0x9F )
 {
     
    Empty_Delay ();
    sendHeadOut ((DataSize-DATA_MAX_SIZE),CHECKSUM_BUF[DATASIZE],0);
    wait_bytes=0;
	rs232_s=12; //end
    step12();
  }  
}
void step12(void)
{
//  for (j=0;j<600;j++);
//  for (j=0;j<600;j++);
 // for (j=0;j<600;j++);
 // for (j=0;j<600;j++);
 // for (j=0;j<600;j++);
//  for (j=0;j<600;j++);
//  for (j=0;j<600;j++);
//  for (j=0;j<600;j++);
  wait_bytes++;
 // if(wait_bytes>=PHASE7 )
  if( nowCC== 0x9F )
   {
    if (DataSize<=DATA_MAX_SIZE2)
    { 
       unsigned int end=(DataSize-DATA_MAX_SIZE);
       sendDataOut (0 , end , CHECKSUM_BUF[CHECKSUM3_L] ,CHECKSUM_BUF[CHECKSUM3_H]);                                 wait_bytes=0;
       rs232_s=100; //end
    }  
    else
    {   
        sendDataOut (0 , (DATA_MAX_SIZE) , CHECKSUM_BUF[CHECKSUM2_L] ,CHECKSUM_BUF[CHECKSUM2_H]);    
         wait_bytes=0;
         rs232_s=13; //end
           UART1_WaitEmpty ();
       //   UART1_Clear ();
          step13();
     }    
  }  
}
void step13(void)
{
  wait_bytes++;
 // if(wait_bytes>=PHASE3 )
// if( nowCC== 0xB5 )
  if( nowCC== 0x9F )
 {
     
    Empty_Delay ();
#ifdef FOR_TEST
       for (j=0;j<6553;j++);
     for (j=0;j<500;j++);
  	 for ( i=0; i<3 ; i++)
	  {
	    putdata (Header2[i]);
	    for (j=0;j<100;j++);
	  }
#else      
     for (j=0;j<5000;j++);
    // for (j=0;j<500;j++);
        if (for_test ) CHECKSUM_BUF[DATASIZE]=0x81;
        sendHeadOut ((DataSize-DATA_MAX_SIZE-DATA_MAX_SIZE),CHECKSUM_BUF[DATASIZE],0);
         UART1_WaitEmpty ();
#endif        
        /*
     for (j=0;j<6553;j++);
     for (j=0;j<500;j++);
      putdata (Header2[i]
	 for ( i=0; i<3 ; i++)
	  {
	    putdata (Header2[i]);
	    for (j=0;j<100;j++);
	  }
         * */
      wait_bytes=0;
	   rs232_s=14; //end
      step14();
   }  
}
void step14(void)
{
  for (j=0;j<5000;j++);
//  for (j=0;j<6553;j++);
 // for (j=0;j<600;j++);
 // for (j=0;j<600;j++);
 // for (j=0;j<600;j++);
 // for (j=0;j<600;j++);
//  for (j=0;j<600;j++);
 // for (j=0;j<600;j++);
  //for (j=0;j<600;j++);
 // for (j=0;j<600;j++);
  wait_bytes++;
 //if(wait_bytes>=PHASE7 )
 // if( nowCC== 0x9F )
  {
     unsigned int end=(DataSize-DATA_MAX_SIZE-DATA_MAX_SIZE);
    
     sendDataOut (0 , end , CHECKSUM_BUF[CHECKSUM3_L] ,CHECKSUM_BUF[CHECKSUM3_H]);
     wait_bytes=0;
    // FinishFlag=1;  
	 rs232_s=100; //end
  }
}
void step18(void)
{
    wait_bytes++;
    if(wait_bytes > (unsigned char)PHASE18 )
    {
       lcd_log_flag=1;
       wait_bytes=0;
       if (MultiFlag)
       {
           
       }
       else
       {
          
        //   idle=(MAX_WAIT_TIME-3);
        //   idle2=(MAX_WAIT_TIME2-3);
       }
       rs232_s=100; //end
    }
           

}
void step400(void)
{
     for (j=0;j<1000 ;j++);
  putdata (0x60);  
for ( i=0; i<3 ; i++)
	{
	 putdata (Header[i]);
	// for (j=0;j<BYTE_DELAY;j++);
       for (j=0;j<FILE_DELAY ;j++);
	 }
	
//for ( i=0; i<258 ; i++)
  for ( i=0; i<258 ; i++)
	{
	 putdata (LCD_BUF01[i]);
	// for (j=0;j<BYTE_DELAY;j++);
      for (j=0;j<FILE_DELAY ;j++);
	 }
 
rs232_s=100; //end
       wait_bytes=0;
     // idle=(MAX_WAIT_TIME+3);
    //  idle2=(MAX_WAIT_TIME2+3);
       FinishFlag=1;    
}
void step106(void)
{
for ( i=0; i<3 ; i++)
	{
	 putdata (File_Header[i]);
	// for (j=0;j<BYTE_DELAY;j++);
       for (j=0;j<FILE_DELAY ;j++);
	 }
#ifdef STOCKER	
for ( i=0; i<122 ; i++)
	{
	 putdata (FILE_BUF1[i]);
	// for (j=0;j<BYTE_DELAY;j++);
      for (j=0;j<FILE_DELAY ;j++);
	 }
#else 			   
for ( i=0; i<122 ; i++)
	{
	 putdata (FILE_BUF0[i]);
	// for (j=0;j<BYTE_DELAY;j++);
      for (j=0;j<FILE_DELAY ;j++);
	 }
#endif	
rs232_s=0; //end
       wait_bytes=0;
      idle=(MAX_WAIT_TIME+3);
      idle2=(MAX_WAIT_TIME2+3);
       FinishFlag=1;    
}
void step105(void)
{
    if( (nowCC== 0xBF )  )
    {
       // putdata (0x60);
     //   UART1_WaitEmpty ();
       //  lcd_log_flag=1;
         wait_bytes=0;
        for(i=0;i<60000;i++);
       idle=(MAX_WAIT_TIME+3);
       idle2=(MAX_WAIT_TIME2+3);
       FinishFlag=2;     
    }
    
}
void step102(void)
{
    if( (nowCC== 0x9F ) || (nowCC== 0x7F ) || (nowCC== 0xF9 ) )
    {
        UART1_WaitEmpty ();
      // for(i=0;i<10000;i++);
       idle=(MAX_WAIT_TIME+3);
       idle2=(MAX_WAIT_TIME2+3);
       FinishFlag=1;     
    }
    
}
void step103(void)
{
    if(  (nowCC== 0x7F ) )
    {
        UART1_WaitEmpty ();
        for(i=0;i<10000;i++);
       idle=(MAX_WAIT_TIME+3);
       idle2=(MAX_WAIT_TIME2+3);
       FinishFlag=1;     
    }
    
}

void step199(void)
{
    #ifdef DATA_LOGGER20
lineCnt++;
if (lineCnt > 5 )
{
    LCD_Set_Cursor(EPD_index++,0);
    lineCnt=0;
}

            
            putcLCD(Hex2ASC ((nowCC >> 4) ));
            putcLCD(Hex2ASC ((nowCC & 0x0F) ));
            putcLCD(' ');

#endif
    if (nowCC==0xF9)
    {
         rs232_s=102;
        wait_bytes=0;
         return;
    }
     if (nowCC==0xFD)
    {
         rs232_s=200;
        wait_bytes=0;
         return;
    }
}
void step200(void)
{  unsigned char cc=0;
#ifdef STOCKER 
for ( i=0; i<25 ; i++)
	{
	 putdata (status3[cc++]);
	// for (j=0;j<BYTE_DELAY;j++);
       for (j=0;j<UART_DELAY;j++);
	 }
//for (j=0;j<1000;j++);
for ( i=0; i<2 ; i++)
	{
	 putdata (status3[cc++]);
	// for (j=0;j<BYTE_DELAY;j++);
       for (j=0;j<UART_DELAY;j++);
	 }
     rs232_s=102; //end
       wait_bytes=0;
    //  idle=(MAX_WAIT_TIME+3);
    //  idle2=(MAX_WAIT_TIME2+3);
     //  FinishFlag=1;   
     
#else
for ( i=0; i<23 ; i++)
	{
	 putdata (status0[cc++]);
	// for (j=0;j<BYTE_DELAY;j++);
       for (j=0;j<UART_DELAY;j++);
	 }
//for (j=0;j<1000;j++);
for ( i=0; i<2 ; i++)
	{
	 putdata (status0[cc++]);
	// for (j=0;j<BYTE_DELAY;j++);
       for (j=0;j<UART_DELAY;j++);
	 }
     rs232_s=102; //end
       wait_bytes=0;
    //  idle=(MAX_WAIT_TIME+3);
    //  idle2=(MAX_WAIT_TIME2+3);
     //  FinishFlag=1;   
#endif      
}
#ifdef MASTER_MODE
void Master_checksum (void)
{
    
    unsigned int checksum;
    checksum=0;
  for ( i=0; i<241 ; i++)
   {
	  checksum+=~decode(~LCD_BUF00[i]);
    } 
    checksum+=0xA0;
    // decode
    i=0;
}

void step300(void)
{
 //    for ( i=0; i<60000 ; i++);
if( nowCC== 0x60||nowCC== 0xE0)  
{
      for (j=0;j<10000;j++);
       #ifdef DATA_LOGGER2
            putcLCD('W');
            putcLCD(' ');

        #endif
   for ( i=0; i<241 ; i++)
   {
	   putdata (LCD_BUF00[i]);
	    for (j=0;j< UART_DELAY;j++);
    }
     for (j=0;j<10000;j++);
      for ( i=0; i<4 ; i++)
   {
	   putdata (LCD_BUF00[241+i]);
	    for (j=0;j< UART_DELAY;j++);
    }
}
 rs232_s=301;   
}
 void step302(void)
{
 //    for ( i=0; i<60000 ; i++);
if( nowCC== 0x60||nowCC== 0xE0)  
{
      for (j=0;j<10000;j++);
       #ifdef DATA_LOGGER2
            putcLCD('S');
            putcLCD(' ');
        #endif
  	   putdata (0x9F);
	    for (j=0;j< UART_DELAY;j++);
      putdata (0xFF);
	  
}
 rs232_s=301;   
}
void step301(void)
{
    
}
#endif
void switchdefault (unsigned char cc)
{
  wait_bytes++;
  if (recFlag)
   {   
      if(MultiFlag)
      {    
                switch (rs232_FC)
                 {
                     case 0:
                     {    
                        if (cc==0xFC)
                         rs232_FC=1;
                     }    
                      break; 
                      case 1:
                     {    
                          if (cc==0xFF)
                          {
                               putcLCD('F');
                              rs232_FC=2;
                          }
                          else
                              rs232_FC=0;
                      }    
                      break; 
                      case 2:
                      {    
                        if (cc==0xB5)
                        {
                              putcLCD('B');
                            rs232_FC=4;
                        }   
                         else
                            rs232_FC=0;
                      }    
                      break; 
                       case 3:
                       {    
                          if (cc==0xFF)
                              rs232_FC=4;
                            else
                               rs232_FC=0;
                        }    
                        break; 
                        case 4:
                        {    
                            if (cc==0xFF)
                           {
                              wait_bytes=0;
                              findB5=1;
                              idle=0;
                               idle2=0;
                               putdata (0xca);
                               for (j=0;j<500;j++);
                                 putdata (0x60);
                                 for (j=0;j<500;j++);
                                 putdata (0x60);
                                wait_bytes=0;
                                 wait_bytes=0;
                                 if (MultiFlag)
                                 {
                                     MultiFlag++;
                                     if (MultiFlag>=MAX_MULTIFLAG)
                                     {
                                            idle=(MAX_WAIT_TIME+3);
                                            idle2=(MAX_WAIT_TIME2+3);
                                            FinishFlag=1;
                                     }
                                 }    
                                 rs232_FC=1;
                            }  
                            else
                               rs232_FC=0;
                            }    
                            break; 
                 } // end switch  
        }else { //// Not MultiFlag
          
        // if(gieeFlag && (wait_bytes>= DataSize))
             if( wait_bytes>= (DataSize+2)) //include checksum
          {
               for (j=0;j< 500;j++);// 20170813 
              UART1_WaitEmpty ();// 20170813 
              putdata (0x60); // 20170813 
              FinishFlag=1;
              rs232_s=0; //20170925
              idle=(MAX_WAIT_TIME)/SPEEDUP;
              idle2=(MAX_WAIT_TIME2)/SPEEDUP;
          }
        
         // if(wait_bytes> MAX_ONEFLAG)
         // {
          //     for (j=0;j< 500;j++);// 20170813 
           //   UART1_WaitEmpty ();// 20170813 
           //   putdata (0x60); // 20170813 
            //  FinishFlag=1;
         // }
        }
     } // end if not recFlag
     else
     {
       putdata (0x60); // 20170813 
       if (nowCC==0xFC)
        {
               FinishFlag=1;
              rs232_s=0; //20171005
        }
         #ifdef DATA_LOGGER20
            lineCnt++;
            if (lineCnt > 5 )
            {
                LCD_Set_Cursor(EPD_index++,0);
                lineCnt=0;
            }
            putcLCD(Hex2ASC ((nowCC >> 4) ));
            putcLCD(Hex2ASC ((nowCC & 0x0F) ));
            putcLCD(' ');
        #else                
           idle=(MAX_WAIT_TIME)/SPEEDUP;
           idle2=(MAX_WAIT_TIME2)/SPEEDUP;
       #endif
     }  
}
void sendHeadOut (unsigned int DataSS,unsigned int DataS ,unsigned char flag)
{
//   for (j=0;j<6553;j++);
   if (flag)
   {    
      putdata (0xca);
      for (j=0;j<500;j++);
	  putdata (0x60);
     for (j=0;j<500;j++);
   }
    if (DataSS<=DATA_MAX_SIZE)
      Header2[1]=DataS;
    else
      Header2[1]=0x81;
    
	 for ( i=0; i<3 ; i++)
	 {
       if (DataSS==(DATA_MAX_SIZE-1))
          putdata (Header3[i]);
       else        
       if(DataSS<DATA_MAX_SIZE)
          putdata (Header2[i]);
        else 
	       putdata (Header[i]);
	    for (j=0;j< UART_DELAY;j++);
	  }
}
void sendDataOut (unsigned int start , unsigned int ended , unsigned char checksuml ,unsigned char checksumh )
{
         for ( i=start; i<ended ; i++)
         {
            putdata (LCD_BUF[lcd_buf_index++]);
            for (j=0;j< UART_DELAY;j++);
         }
        putdata ( checksuml);
        for (j=0;j<100;j++);
        putdata ( checksumh );
}

void log_data(unsigned char cc)
{
#ifdef PC_BUF_DEBUG  
    if (pc_buf_index<MAX_PC_BUFFER)
      PC_BUF[pc_buf_index]  =cc;
#endif      
      if (MultiFlag)
      {   
          if(DataSize > SKIP1)
          {    
            if(  (pc_buf_index > SKIP1) && (pc_buf_index <= (unsigned int)(SKIP1+SKIP1_SIZE)))
               {
                 PC_H2[PC_h2_index++]  =decode(cc);
               }
            else
            if(  (pc_buf_index > SKIP2) && (pc_buf_index <= (unsigned int)(SKIP2+SKIP2_SIZE)))
               {
                 PC_H3 [PC_h3_index++]  =decode(cc);
               }
            else
            {   
                LCD_BUF[lcd_buf_index]  =cc;
                lcd_buf_index++;
            }
          } 
          else
          {
                
                LCD_BUF[lcd_buf_index]  =cc;
                lcd_buf_index++;
          }   

      }     
      else  ///////////////// MultiFlag
      {
         LCD_BUF[lcd_buf_index]  =decode(cc);
         lcd_buf_index++;
      }    
     
    
      pc_buf_index++;

}
unsigned char decode ( unsigned char cc)
{
    unsigned char aa =0;
    if (cc & 0x80)  aa |=0x01;
    if (cc & 0x40)  aa |=0x02;
    if (cc & 0x20)  aa |=0x04;
    if (cc & 0x10)  aa |=0x08;
    
    if (cc & 0x08)  aa |=0x10;
    if (cc & 0x04)  aa |=0x20;
    if (cc & 0x02)  aa |=0x40;
    if (cc & 0x01)  aa |=0x80;
    return aa;
}
unsigned char encode ( unsigned char cc)
{
    unsigned char aa =0;
    if (cc & 0x80)  aa |=0x01;
    if (cc & 0x40)  aa |=0x02;
    if (cc & 0x20)  aa |=0x04;
    if (cc & 0x10)  aa |=0x08;
    
    if (cc & 0x08)  aa |=0x10;
    if (cc & 0x04)  aa |=0x20;
    if (cc & 0x02)  aa |=0x40;
    if (cc & 0x01)  aa |=0x80;
    return aa;
}

void checksum (void)
{
   unsigned int checksum;
   unsigned char checksumhi;
   unsigned int checksum2;
   realDataSize=DataSize;  
   unsigned int tailDataSize=(decode(LCD_BUF[SIZE1_LOW] )<<8) +decode(LCD_BUF[SIZE1_HIGH]);
   
   if(MultiFlag) 
   {
        
       if (DataSize<= DATA_MAX_SIZE)
       {
       
        
        if (DataSize >=PC1_SIZE)    /////// 250~256
        {   
            checksum = (PC_H2[2] <<8) +PC_H2[0] ;
            for (i=PC1_SIZE;i<(DataSize);i++)
            {  
                if (decode(LCD_BUF[i]) < ENTER_CODE ) break;
               checksum +=decode(LCD_BUF[ i]);
            }
            
            checksumhi=(char)(checksum >> 8);
            CHECKSUM_BUF[CHECKSUM3_H]=encode((char)checksumhi);
            CHECKSUM_BUF[CHECKSUM3_L]=encode((char)(checksum & 0x00FF));
         
            
             checksum= tailDataSize;
        }
        else
        {
           CHECKSUM_BUF[CHECKSUM3_L]=LCD_BUF[DataSize];
           CHECKSUM_BUF[CHECKSUM3_H]=LCD_BUF[DataSize+2];
         //  checksum= TAIL_CHECKSUM_BASE+(DataSize);
           checksum= tailDataSize;
        }
         checksumhi=(char)(checksum >> 8);
         CHECKSUM_BUF[CHECKSUM1_H]=encode((char)checksumhi);
         CHECKSUM_BUF[CHECKSUM1_L]=encode((char)(checksum & 0x00FF));
        
        checksumhi=(char)(checksum >> 8);
        LCD_BUF1[TAIL_ADDR1]=encode((char)checksumhi);
        LCD_BUF1[TAIL_ADDR2]=encode((char)(checksum & 0x00FF));
        
        
        checksum2=TAIL_BASE+(checksum & 0x00FF)+ checksumhi    ;
        checksumhi=(char)(checksum2 >> 8);
        LCD_BUF1[TAIL_ADDR3]=encode((char)(checksum2 & 0x00FF));
        LCD_BUF1[TAIL_ADDR4]=encode((char)checksumhi);
        CHECKSUM_BUF[DATASIZE]=encode(DataSize+1);
       }
       else
        if (DataSize <= DATA_MAX_SIZE2)   
       {
             // Jason 
          if ((DataSize >=SSS1) && (DataSize<=SSS2))
          {   
            checksum= TAIL_CHECKSUM_BASE+ (DataSize-1);
           //  LCD_BUF1[TAIL_ADDR0]=0x44;
          //   LCD_BUF1[TAIL_ADDR00]=0x03;
          }
           else
           checksum= tailDataSize;  
        checksumhi=(char)(checksum >> 8);
        LCD_BUF1[TAIL_ADDR1]=encode((char)checksumhi);
        LCD_BUF1[TAIL_ADDR2]=encode((char)(checksum & 0x00FF));
      // if (DataSize >=461&& DataSize<=479)
       //   checksum2=TAIL_BASE+(checksum & 0x00FF)+ checksumhi  +0x22+0xC0  ;
      //  else
          checksum2=TAIL_BASE+(checksum & 0x00FF)+ checksumhi ;
        checksumhi=(char)(checksum2 >> 8);
        LCD_BUF1[TAIL_ADDR3]=encode((char)(checksum2 & 0x00FF));
        LCD_BUF1[TAIL_ADDR4]=encode((char)checksumhi);
        
           checksum = (PC_H2[2] <<8) +PC_H2[0] ;
        
           checksum +=( decode(LCD_BUF[ ADJ1_ADDR1])+decode( LCD_BUF[ ADJ1_ADDR2]) + decode(LCD_BUF[ ADJ1_ADDR3])
                   + decode(LCD_BUF[ ADJ1_ADDR4])+ decode(LCD_BUF[ ADJ1_ADDR5])+ decode(LCD_BUF[ ADJ1_ADDR6]));
        
           checksumhi=(char)(checksum >> 8);
           CHECKSUM_BUF[CHECKSUM1_H]=encode((char)checksumhi);
           CHECKSUM_BUF[CHECKSUM1_L]=encode((char)(checksum & 0x00FF));
           
           checksum=CHECKSUM_BASE;
           
           for (i=DATA_MAX_SIZE;i<(DataSize);i++)
                  checksum+=decode(LCD_BUF[ i]);
             
           
           checksumhi=(char)(checksum >> 8);
           CHECKSUM_BUF[CHECKSUM3_H]=encode((char)checksumhi);
           CHECKSUM_BUF[CHECKSUM3_L]=encode((char)(checksum & 0x00FF));
           
           CHECKSUM_BUF[DATASIZE]=encode(((DataSize-DATA_MAX_SIZE+1)));
       }
        else 
       {
           checksum = (PC_H2[2] <<8) +PC_H2[0] ;
           checksum =checksum + decode(LCD_BUF[ ADJ1_ADDR1])+decode( LCD_BUF[ ADJ1_ADDR2]) + decode(LCD_BUF[ ADJ1_ADDR3])
                   + decode(LCD_BUF[ ADJ1_ADDR4])+ decode(LCD_BUF[ ADJ1_ADDR5])+ decode(LCD_BUF[ ADJ1_ADDR6]);
           checksumhi=(char)(checksum >> 8);
           CHECKSUM_BUF[CHECKSUM1_H]=encode((char)checksumhi);
           CHECKSUM_BUF[CHECKSUM1_L]=encode((char)(checksum & 0x00FF));
           
#ifdef GET_CHECKSUM_PC
            checksum = (PC_H3[2] <<8) +PC_H3[0] ;
           
          checksum =checksum + decode(LCD_BUF[ ADJ2_ADDR1])+decode( LCD_BUF[ ADJ2_ADDR2]) + decode(LCD_BUF[ ADJ2_ADDR3])
                  + decode(LCD_BUF[ ADJ2_ADDR4])+ decode(LCD_BUF[ ADJ2_ADDR5])+ decode(LCD_BUF[ ADJ2_ADDR6]);
           checksum =checksum - decode(LCD_BUF[ ADJ1_ADDR1])-decode( LCD_BUF[ ADJ1_ADDR2]) - decode(LCD_BUF[ ADJ1_ADDR3])
                   - decode(LCD_BUF[ ADJ1_ADDR4])- decode(LCD_BUF[ ADJ1_ADDR5])-decode(LCD_BUF[ ADJ1_ADDR6]);
           
           checksum =checksum + decode(LCD_BUF[ ADJ2_ADDR7])+decode( LCD_BUF[ ADJ2_ADDR8]) + decode(LCD_BUF[ ADJ2_ADDR9])
                   + decode(LCD_BUF[ ADJ2_ADDR10])+ decode(LCD_BUF[ ADJ2_ADDR11])+ decode(LCD_BUF[ ADJ2_ADDR12]);
           
            checksumhi=(char)(checksum >> 8);
           CHECKSUM_BUF[CHECKSUM2_H]=encode((char)checksumhi);
           CHECKSUM_BUF[CHECKSUM2_L]=encode((char)(checksum & 0x00FF));
#else           
           checksum =CHECKSUM_BASE;
            for (i=DATA_MAX_SIZE;i<DATA_MAX_SIZE2;i++)
               checksum+=decode(LCD_BUF[ i]);
           
           checksumhi=(char)(checksum >> 8);
           CHECKSUM_BUF[CHECKSUM2_H]=encode((char)checksumhi);
           CHECKSUM_BUF[CHECKSUM2_L]=encode((char)(checksum & 0x00FF));
          ////////////////////////////////////////////////////////////////////// 
            checksum = (PC_H3[2] <<8) +PC_H3[0] ;
           
          checksum =checksum + decode(LCD_BUF[ ADJ2_ADDR1])+decode( LCD_BUF[ ADJ2_ADDR2]) + decode(LCD_BUF[ ADJ2_ADDR3])
                  + decode(LCD_BUF[ ADJ2_ADDR4])+ decode(LCD_BUF[ ADJ2_ADDR5])+ decode(LCD_BUF[ ADJ2_ADDR6]);
           checksum =checksum - decode(LCD_BUF[ ADJ1_ADDR1])-decode( LCD_BUF[ ADJ1_ADDR2]) - decode(LCD_BUF[ ADJ1_ADDR3])
                   - decode(LCD_BUF[ ADJ1_ADDR4])- decode(LCD_BUF[ ADJ1_ADDR5])-decode(LCD_BUF[ ADJ1_ADDR6]);
           
           checksum =checksum + decode(LCD_BUF[ ADJ2_ADDR7])+decode( LCD_BUF[ ADJ2_ADDR8]) + decode(LCD_BUF[ ADJ2_ADDR9])
                   + decode(LCD_BUF[ ADJ2_ADDR10])+ decode(LCD_BUF[ ADJ2_ADDR11])+ decode(LCD_BUF[ ADJ2_ADDR12]);
           
            checksumhi=(char)(checksum >> 8);
           CHECKSUM_BUF[CHECKSUM4_H]=encode((char)checksumhi);
           CHECKSUM_BUF[CHECKSUM4_L]=encode((char)(checksum & 0x00FF));
#endif           
           ///////////////////////////////////////////////////////
           checksum=CHECKSUM_BASE;
           
           for (i=DATA_MAX_SIZE2;i<(DataSize);i++)
           {
              checksum+=decode(LCD_BUF[ i]);
           }
        //   checksum = (decode(LCD_BUF[DataSize+1]) <<8) +decode(LCD_BUF[DataSize-1]) ;
         //  checksum -=( decode(LCD_BUF[ ADJ2_ADDR1])+decode( LCD_BUF[ ADJ2_ADDR2]) + decode(LCD_BUF[ ADJ2_ADDR3])
          //         + decode(LCD_BUF[ ADJ2_ADDR4])+ decode(LCD_BUF[ ADJ2_ADDR5])+ decode(LCD_BUF[ ADJ2_ADDR6]));
           //checksum -=( MID_BUF[ ADJ2_ADDR1]+ MID_BUF[ ADJ2_ADDR2] + MID_BUF[ ADJ2_ADDR3]
           //      + MID_BUF[ ADJ2_ADDR4]+ MID_BUF[ ADJ2_ADDR5]+ MID_BUF[ ADJ2_ADDR6]);
           checksumhi=(char)(checksum >> 8);
           CHECKSUM_BUF[CHECKSUM3_H]=encode((char)checksumhi);
           CHECKSUM_BUF[CHECKSUM3_L]=encode((char)(checksum & 0x00FF));
           
           
              checksum= tailDataSize;   
      //  checksum= TAIL_CHECKSUM_BASE+(DataSize-6); //// Why ????
        //   checksum=(decode(PC_BUF[DataSize+1])<<8+decode(PC_BUF[DataSize-1]))-0x28A;
        checksumhi=(char)(checksum >> 8);
        LCD_BUF1[TAIL_ADDR1]=encode((char)checksumhi);
        LCD_BUF1[TAIL_ADDR2]=encode((char)(checksum & 0x00FF));
        checksum2=TAIL_BASE+(checksum & 0x00FF)+ checksumhi    ;
        checksumhi=(char)(checksum2 >> 8);
        LCD_BUF1[TAIL_ADDR3]=encode((char)(checksum2 & 0x00FF));
        LCD_BUF1[TAIL_ADDR4]=encode((char)checksumhi);
        
        CHECKSUM_BUF[DATASIZE]=encode(((DataSize-DATA_MAX_SIZE+1)));
       } 
   }
   else ///////////// MultiFlag
   {    
                checksum=BASE1+ (BASEH << 8);
                unsigned char range1=DataSize%LCD_SIZE;
                unsigned char range2=(DataSize/LCD_SIZE)*LCD_SIZE;
                unsigned char range3=RANG1-range2;
                if (!range1)
                {
                    range2=(DataSize/LCD_SIZE-1)*LCD_SIZE;
                    range3=RANG1-range2;
                }   
                if (range2)
                {    
                  for (i=range3;i<RANG1;i++)
                      checksum +=( LCD_BUF[i]);
                  checksum-=range2;
                
                }
                for (i=0;i<range1;i++)
                {
                    checksum +=( LCD_BUF[RANG1+i]);
                    checksum -=EMPTY1;

                }
                
                if (!range1)
                {   
                     for (i=0;i<LCD_SIZE;i++)
                    {
                         checksum +=( LCD_BUF[RANG1+i]);
                          checksum -=EMPTY1;

                    }
                }    
                 checksumhi=(char)(checksum >> 8);

 #ifdef USE_EEPROM
       DATAEE_WriteByte(CHECKSUMH , encode ( checksumhi));
       DATAEE_WriteByte(CHECKSUM1,encode ((char)(checksum & 0x00FF) ));
#else          
         LCD_BUF0[CHECKSUMH] = encode ( checksumhi);
        LCD_BUF0[CHECKSUM1] = encode ((char)(checksum & 0x00FF) );
#endif 
              
       
                UUID_index=0;
                while(getUUID ());
   } 
 

}
unsigned char displayP (void)
{
    if( !sysReady) return 0;
 
  if(!LCD_index)
  {
     ClearLCD();
    EPD_index=0;
  }
             mode++;
         if (mode >2) mode=0;
#ifndef TEST_MULTI    
    if(( LCD_index-Line_index) >(MAX_EPD_ROW*MAX_EPD_COL)) 
    {
        EPD_print();
		EPD_flash();
        return 0; //20170913
    }
#endif 
  idle=0;
  idle2=0; 
  lcdOnFlag=1;
 if (MultiFlag) 
 {
         for (i=0;i<DataSize;i++)
                putcLCD(decode(LCD_BUF[i+ LCD_SIZE*mode]));

       return 0;
       /*
        if (DataSize < ( LCD_index+LCD_SIZE))
        {
            #ifndef USE_EPD  
            LCD_Set_Cursor(0,0);
           #else
#ifndef USE_MAX_EPD
           LCD_Set_Cursor(EPD_index++,0);
#endif 
           #endif
          //  for (i=LCD_index;i<(LCD_index+DataSize );i++)
            for (i=LCD_index;i<(DataSize );i++)
             putcLCD(decode(LCD_BUF[i]));
            LCD_index=0;
			#ifdef USE_EPD   
#ifndef USE_MAX_EPD
             EPD_print();
			EPD_flash();
#endif            
		   #endif  
		   return 0;
        }  
        else
        if (DataSize < ( LCD_index+LCD_SIZE+LCD_SIZE))
        {
           #ifndef USE_EPD  
            LCD_Set_Cursor(0,0);
           #else
#ifndef USE_MAX_EPD
           LCD_Set_Cursor(EPD_index++,0);
#endif           
            #endif 
            for (i=LCD_index;i<(LCD_index+ LCD_SIZE );i++)
             putcLCD(decode(LCD_BUF[i]));
            #ifndef USE_EPD  
            
            LCD_Set_Cursor(1,0);
           #else
#ifndef USE_MAX_EPD
            EPD_print();
           LCD_Set_Cursor(EPD_index++,0);
#endif           
            #endif
            LCD_index+= LCD_SIZE;  
          ///  for (i=LCD_index;i<(LCD_index+ DataSize );i++)
            for (i=LCD_index;i<(DataSize );i++)
             putcLCD(decode(LCD_BUF[i]));  
            LCD_index=0;
			#ifdef USE_EPD 
#ifndef USE_MAX_EPD
               EPD_print();
				EPD_flash();
#endif                
		    #endif  
		    return 0;
        }   
        else
        {
            #ifndef USE_EPD  
            LCD_Set_Cursor(0,0);
           #else
#ifndef USE_MAX_EPD
           LCD_Set_Cursor(EPD_index++,0);
#endif           
            #endif 
            for (i=LCD_index;i<(LCD_index+ LCD_SIZE );i++)
            putcLCD(decode(LCD_BUF[i]));
             #ifndef USE_EPD  
            LCD_Set_Cursor(1,0);
           #else
#ifndef USE_MAX_EPD
            EPD_print();
           LCD_Set_Cursor(EPD_index++,0);
#endif           
            #endif 
            LCD_index+= LCD_SIZE;  
            for (i=LCD_index;i<(LCD_index+ LCD_SIZE );i++)
             putcLCD(decode(LCD_BUF[i])); 
            #ifdef USE_EPD
#ifndef USE_MAX_EPD
                        EPD_print();
#endif                       
            #endif            
            LCD_index+= LCD_SIZE; 
        }
        */
 }
 else ///////////// Not MultiFlag
 {
    unsigned char range1=DataSize%LCD_SIZE;
    unsigned char range2=((DataSize/LCD_SIZE))*LCD_SIZE;
    unsigned char range3=RANG1-range2;
  
    if ( !range1)
    {
     range1=LCD_SIZE;
     range2=((DataSize/LCD_SIZE-1))*LCD_SIZE;
     range3=RANG1-range2;
    }
    if (DataSize <= ( LCD_index+LCD_SIZE))
    {
          #ifndef USE_EPD  
            LCD_Set_Cursor(0,0);
           #else
#ifndef USE_MAX_EPD
           LCD_Set_Cursor(EPD_index++,0);
#endif           
          #endif
         for (i= (RANG1);i<(RANG1+range1);i++)
#ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else             
         putcLCD(decode(LCD_BUF0[i]));
#endif           
          LCD_index=0;
		  #ifdef USE_EPD  
#ifndef USE_MAX_EPD
          EPD_print();
			EPD_flash();
#endif            
		 #endif  
		 return 0;
    }
    else
     if (DataSize <=( LCD_index+LCD_SIZE+LCD_SIZE))
     {
         unsigned char range3=(RANG1-LCD_SIZE);
    
             #ifndef USE_EPD  
            LCD_Set_Cursor(0,0);
           #else
#ifndef USE_MAX_EPD
           LCD_Set_Cursor(EPD_index++,0);
#endif           
            #endif
           for (i=(range3);i<(LCD_SIZE+range3);i++)
 #ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else             
         putcLCD(decode(LCD_BUF0[i]));
#endif    
             LCD_index+= LCD_SIZE;  
           #ifndef USE_EPD  
            LCD_Set_Cursor(1,0);
           #else
#ifndef USE_MAX_EPD
            EPD_print();
           LCD_Set_Cursor(EPD_index++,0);
#endif           
            #endif
          for (i=RANG1;i<(RANG1+range1);i++) 
 #ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else             
         putcLCD(decode(LCD_BUF0[i]));
#endif    
           LCD_index=0;
		   #ifdef USE_EPD 
#ifndef USE_MAX_EPD
              EPD_print();
				EPD_flash();
#endif                
           #endif  
		   return 0;
     }
     else
     {
         #ifndef USE_EPD  
            LCD_Set_Cursor(0,0);
           #else
#ifndef USE_MAX_EPD
           LCD_Set_Cursor(EPD_index++,0);
#endif           
            #endif
        for (i=(range3+LCD_index);i<(LCD_index+ LCD_SIZE+range3 );i++)
#ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else             
         putcLCD(decode(LCD_BUF0[i]));
#endif    
           #ifndef USE_EPD  
            LCD_Set_Cursor(1,0);
           #else
#ifndef USE_MAX_EPD
            EPD_print();
           LCD_Set_Cursor(EPD_index++,0);
#endif           
            #endif 
        LCD_index+= LCD_SIZE;  
        for (i=(LCD_index+range3);i<(LCD_index+ LCD_SIZE+range3 );i++)
#ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else             
         putcLCD(decode(LCD_BUF0[i]));
#endif     
#ifdef USE_EPD
        #ifndef USE_MAX_EPD
        EPD_print();
        #endif
#endif        
        LCD_index+= LCD_SIZE; 
		
		return 1;
     }
 
 }    
  return 0;
} 
void displayN (void)
{
    ClearLCD();
 if (MultiFlag) 
 {
       // if (LCD_index <= ( LCD_SIZE))
       // {
       //     LCD_Set_Cursor(0,0); 
          //  for (i=LCD_index;i<(LCD_index+DataSize );i++)
       //     for (i=LCD_index;i<(LCD_SIZE );i++)
       //      putcLCD(decode(LCD_BUF[i]));
       //     LCD_index=0;
       // }  
       // else
        if (LCD_index <= ( LCD_SIZE+LCD_SIZE))
        {
            LCD_index=0;
              LCD_Set_Cursor(0,0); 
            for (i=LCD_index;i<(LCD_index+ LCD_SIZE );i++)
             putcLCD(decode(LCD_BUF[i]));
               LCD_Set_Cursor(1,0); 
            LCD_index+= LCD_SIZE;  
          ///  for (i=LCD_index;i<(LCD_index+ DataSize );i++)
            for (i=LCD_index;i<(LCD_index+ LCD_SIZE+ LCD_SIZE );i++)
             putcLCD(decode(LCD_BUF[i]));  
            LCD_index=0;
        }   
        else
        {
              LCD_index-= LCD_SIZE; 
              LCD_index-= LCD_SIZE; 

               LCD_Set_Cursor(0,0); 
            for (i=LCD_index;i<(LCD_index+ LCD_SIZE );i++)
             putcLCD(decode(LCD_BUF[i]));
               LCD_Set_Cursor(1,0); 
            LCD_index+= LCD_SIZE;  
            for (i=LCD_index;i<(LCD_index+ LCD_SIZE );i++)
             putcLCD(decode(LCD_BUF[i]));  
            LCD_index-= LCD_SIZE; 
        }
 }
 else ///////////// Not MultiFlag
 {
    unsigned char range1=DataSize%LCD_SIZE;
    unsigned char range2=(DataSize/LCD_SIZE)*LCD_SIZE;
    unsigned char range3=RANG1-range2;
   if ( !range1)
    {
     range1=LCD_SIZE;
     range2=((DataSize/LCD_SIZE-1))*LCD_SIZE;
     range3=RANG1-range2;
    }
    if ((LCD_index <= ( LCD_SIZE)) && (DataSize<=LCD_SIZE))
    {
         LCD_Set_Cursor(0,0);
         for (i= (RANG1);i<(RANG1+range1);i++)
 #ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else             
         putcLCD(decode(LCD_BUF0[i]));
#endif    
          LCD_index=0;
    }
    else
     if (LCD_index < (LCD_SIZE+LCD_SIZE)&& (DataSize<=(LCD_SIZE+LCD_SIZE)))
     {
          unsigned char range3=(RANG1-LCD_SIZE);
           LCD_Set_Cursor(0,0);
           for (i=(range3);i<(LCD_SIZE+range3);i++)
 #ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else             
         putcLCD(decode(LCD_BUF0[i]));
#endif    
             LCD_index-= LCD_SIZE;  
           LCD_Set_Cursor(1,0); 
          for (i=RANG1;i<(RANG1+range1);i++) 
#ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else             
         putcLCD(decode(LCD_BUF0[i]));
#endif    
           LCD_index=0;
     }
     else
     {
        if (LCD_index >= (LCD_SIZE+LCD_SIZE))  
        {
         LCD_index-= LCD_SIZE; 
         LCD_index-= LCD_SIZE; 
        }
        else
         LCD_index=0;   
        LCD_Set_Cursor(0,0); 
        for (i=(range3+LCD_index);i<(LCD_index+ LCD_SIZE+range3 );i++)
#ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else             
         putcLCD(decode(LCD_BUF0[i]));
#endif    
           LCD_Set_Cursor(1,0); 
        LCD_index+= LCD_SIZE;  
        for (i=(LCD_index+range3);i<(LCD_index+ LCD_SIZE+range3 );i++)
#ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else             
         putcLCD(decode(LCD_BUF0[i]));
#endif    
        LCD_index-= LCD_SIZE; 
     }
 
 }    
  idle=0;
  idle2=0; 
    lcdOnFlag=1;
}   


unsigned char Hex2ASC (unsigned char aa )
{
    switch (aa)
    {
        case 0:
             return 0x30;
        case 1:
             return 0x31;
        case 2:
             return 0x32;
        case 3:
             return 0x33;     
        case 4:
             return 0x34;
        case 5:
             return 0x35;
        case 6:
             return 0x36;
        case 7:
             return 0x37;
        case 8:
             return 0x38;
        case 9:
             return 0x39;
        case 0x0A:
             return 0x41;
        case 0x0B:
             return 0x42;     
        case 0x0C:
             return 0x43;     
        case 0x0D:
             return 0x44;     
        case 0x0E:
             return 0x45;     
        case 0x0F:
             return 0x46;     
     
    }
  return 0;  
}
unsigned char ASCII2UUID (unsigned char cc)
{
    
     UUID_Buf[UUID_index2++]=Hex2ASC ((cc >> 4) );
     UUID_Buf[UUID_index2++]=Hex2ASC ((cc & 0x0F) );
   //  Hex2ASC ((cc >> 4) )
   //  Hex2ASC ((cc & 0x0F ))     
     return 0;
}

unsigned char getUUID2 (void)
{
  UUID_index=0;
  UUID_index2=0;
  while(getUUID ());
  i=0;
  return 0;
}
unsigned char getUUID (void)
{
  
  idle=0;
  idle2=0; 
  lcdOnFlag=1;

 if (MultiFlag) 
 {
          for (i=0;i<DataSize;i++)
           {
             //UUID_Buf[UUID_index++]=decode(LCD_BUF0[i]);
              if(UUID_index>LOTID_MIN) 
              ASCII2UUID (decode(LCD_BUF[i]));
              UUID_index++;
              if(UUID_index>LOTID_MAX) return 0;
           }
 }
 else ///////////// Not MultiFlag
 {
    unsigned char range1=DataSize%LCD_SIZE;
    unsigned char range2=((DataSize/LCD_SIZE))*LCD_SIZE;
    unsigned char range3=RANG1-range2;
  
    if ( !range1)
    {
     range1=LCD_SIZE;
     range2=((DataSize/LCD_SIZE-1))*LCD_SIZE;
     range3=RANG1-range2;
    }
    if (DataSize <= ( UUID_index+LCD_SIZE))
    {
          
         for (i= (RANG1);i<(RANG1+range1);i++)
         {
          // UUID_Buf[UUID_index++]=decode(LCD_BUF0[i]);
            if(UUID_index>LOTID_MIN) 
#ifdef USE_EEPROM
                  ASCII2UUID (decode(DATAEE_ReadByte(i)));
#else               
             ASCII2UUID (decode(LCD_BUF0[i]));
#endif            
           UUID_index++;
           if(UUID_index>LOTID_MAX) return 0;
         }
          UUID_index=0;
		 return 0;
    }
    else
     if (DataSize <=(UUID_index+LCD_SIZE+LCD_SIZE))
     {
         unsigned char range3=(RANG1-LCD_SIZE);
    
            
           for (i=(range3);i<(LCD_SIZE+range3);i++)
           {
             //UUID_Buf[UUID_index++]=decode(LCD_BUF0[i]);
              if(UUID_index>LOTID_MIN) 
 #ifdef USE_EEPROM
                  ASCII2UUID (decode(DATAEE_ReadByte(i)));
#else               
             ASCII2UUID (decode(LCD_BUF0[i]));
#endif
           UUID_index++;
             if(UUID_index>LOTID_MAX) return 0;
            }
          for (i=RANG1;i<(RANG1+range1);i++) 
          {
           // UUID_Buf[UUID_index++]=decode(LCD_BUF0[i]);
             if(UUID_index>LOTID_MIN) 
 #ifdef USE_EEPROM
                  ASCII2UUID (decode(DATAEE_ReadByte(i)));
#else               
             ASCII2UUID (decode(LCD_BUF0[i]));
#endif
           UUID_index++;
            if(UUID_index>LOTID_MAX) return 0;
           }
		   return 0;
     }
     else
     {
       for (i=(range3+UUID_index);i<(UUID_index+ LCD_SIZE+range3 );i++)
        {
          // UUID_Buf[UUID_index++]=decode(LCD_BUF0[i]);
           if(UUID_index>LOTID_MIN) 
#ifdef USE_EEPROM
                  ASCII2UUID (decode(DATAEE_ReadByte(i)));
#else               
             ASCII2UUID (decode(LCD_BUF0[i]));
#endif
           UUID_index++;
           if(UUID_index>LOTID_MAX) return 0;
         }
        
        for (i=(UUID_index+range3);i<(UUID_index+ LCD_SIZE+range3 );i++)
        {
          // UUID_Buf[UUID_index++]=decode(LCD_BUF0[i]);
           if(UUID_index>LOTID_MIN) 
#ifdef USE_EEPROM
                  ASCII2UUID (decode(DATAEE_ReadByte(i)));
#else               
             ASCII2UUID (decode(LCD_BUF0[i]));
#endif
           UUID_index++;
           if(UUID_index>LOTID_MAX) return 0;
         }
		
		return 1;
     }
 
 }    
return 0;
} // GetUUID
unsigned int GetDataSize (void)
{
      int checksum=0;
      int checksumpc=0;
 unsigned char checksumhi;
 
     unsigned char addHead;
       addHead =((TAG_SIZE-DataSize)/16)*16;
     
       for (i=0;i<addHead;i++)
       {
#ifdef USE_EEPROM
              DATAEE_WriteByte(i,0x04);
#else           
           LCD_BUF0[i]=0x04;
#endif           
        
       }
       if (tsmcFlag)
       {
            for (i=0;i<(DataSize);i++)
            {
              //   checksum+=LCD_BUF[i];
#ifdef USE_EEPROM
              DATAEE_WriteByte((i+addHead),decode(LCD_BUF[i]));
#else                   
                LCD_BUF0[i+addHead]=decode(LCD_BUF[i]);
#endif                
            }
       }
       else
       {
            for (i=0;i<(DataSize);i++)
            {
              //   checksum+=LCD_BUF[i];
#ifdef USE_EEPROM
              DATAEE_WriteByte((i+addHead),decode(LCD_BUF[i]));
#else                   
                LCD_BUF0[i+addHead]=decode(LCD_BUF[i]);
#endif    
            }
       }
       
   //    getEEPROM();
  //     checksum+=0x90;
     //  LCD_BUF0[DataSize-3]=0x04;
    //   LCD_BUF0[DataSize-2]=0x04;
     //  LCD_BUF0[DataSize-1]=0x04;
       checksumpc=(LCD_BUF[(DataSize)] <<8)+LCD_BUF[(DataSize-2)];
       stockerFirst=0;
       checksumpc +=((TAG_SIZE-DataSize)*0x20);
       
         checksum =checksumpc+0xF0+0x647-0x60;//+0x0A+0x20;
       if(!tsmcFlag)
           checksum+=0x0c;
       else
           checksum+=0x20;
       checksumhi=(char)(checksum >> 8);
       CHECKSUM_BUF[CHECKSUM6_L]=encode((char)(checksum & 0x00FF));
       CHECKSUM_BUF[CHECKSUM6_H] =encode((char)checksumhi); 
	   
       checksum=checksumpc;
        checksumhi=(char)(checksum >> 8);
       CHECKSUM_BUF[CHECKSUM5_L]=encode((char)(checksum & 0x00FF));
      CHECKSUM_BUF[CHECKSUM5_H] =encode((char)checksumhi); 
	  
	    tagSize=DataSize+addHead; //20171010
		
	       for(i=0;i< (TAG_SIZE-tagSize); i++) // for E-RACK 20171010
                 checksum-=0x20;
	    checksumhi=(char)(checksum >> 8);
         CHECKSUM_BUF[CHECKSUM3_L]=encode((char)(checksum & 0x00FF));
        CHECKSUM_BUF[CHECKSUM3_H] =encode((char)checksumhi); 	 

      // Header0[1]=encode((char)(DataSize+32));
       checksum=checksumpc+0x180; /// For ACONFIG
       
         checksumhi=(char)(checksum >> 8);
       CHECKSUM_BUF[CHECKSUM4_L]=encode((char)(checksum & 0x00FF));
      CHECKSUM_BUF[CHECKSUM4_H] =encode((char)checksumhi); 
        
 
    
  
    if ( tsmcFlag)
    {
        eeWrite();
       return DataSize;    
    }
    else
    {
       unsigned char D0_index=D0_LOC;
       unsigned int dd;
       D0_index--;
       while ((D0_index >RANG1) && ( LCD_BUF[D0_index] == EMPTY_DATA)) D0_index--;  //20170621
        times16=(D0_BASE-LCD_BUF[D0_LOC]);
  
        dd=(D0_index-RANG1+1)+ times16;
  
         DataSize=dd; //20171204 why ?????
          eeWrite();
            return dd;
    }
}


#ifndef _32BIT_
void button(void)
{
    DisplayFlag=1;
 //   display();
}
void display (void)
{
    ClearLCD();
 if (MultiFlag) 
 {
        if (DataSize < ( LCD_index+LCD_SIZE))
        {
            LCD_Set_Cursor(0,0); 
          //  for (i=LCD_index;i<(LCD_index+DataSize );i++)
            for (i=LCD_index;i<(DataSize );i++)
             putcLCD(decode(LCD_BUF[i]));
            LCD_index=0;
        }  
        else
        if (DataSize < ( LCD_index+LCD_SIZE+LCD_SIZE))
        {
              LCD_Set_Cursor(0,0); 
            for (i=LCD_index;i<(LCD_index+ LCD_SIZE );i++)
             putcLCD(decode(LCD_BUF[i]));
               LCD_Set_Cursor(1,0); 
            LCD_index+= LCD_SIZE;  
          ///  for (i=LCD_index;i<(LCD_index+ DataSize );i++)
            for (i=LCD_index;i<(DataSize );i++)
             putcLCD(decode(LCD_BUF[i]));  
            LCD_index=0;
        }   
        else
        {
               LCD_Set_Cursor(0,0); 
            for (i=LCD_index;i<(LCD_index+ LCD_SIZE );i++)
             putcLCD(decode(LCD_BUF[i]));
               LCD_Set_Cursor(1,0); 
            LCD_index+= LCD_SIZE;  
            for (i=LCD_index;i<(LCD_index+ LCD_SIZE );i++)
             putcLCD(decode(LCD_BUF[i]));  
            LCD_index+= LCD_SIZE; 
        }
 }
 else ///////////// Not MultiFlag
 {
    unsigned char range1=DataSize%LCD_SIZE;
    unsigned char range2=(DataSize/LCD_SIZE)*LCD_SIZE;
    unsigned char range3=RANG1-range2;
    
    if (DataSize < ( LCD_index+LCD_SIZE))
    {
         LCD_Set_Cursor(0,0);
         for (i= (RANG1);i<(RANG1+range1);i++)
#ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else                   
                putcLCD(decode(LCD_BUF0[i]));
#endif    
         
          LCD_index=0;
    }
    else
     if (DataSize < ( LCD_index+LCD_SIZE+LCD_SIZE))
     {
           LCD_Set_Cursor(0,0);
           for (i=(LCD_index+range3);i<(LCD_index+LCD_SIZE+range3);i++)
 #ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else                   
                putcLCD(decode(LCD_BUF0[i]));
#endif 
             LCD_index+= LCD_SIZE;  
           LCD_Set_Cursor(1,0); 
          for (i=RANG1;i<(RANG1+range1);i++) 
 #ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else                   
                putcLCD(decode(LCD_BUF0[i]));
#endif 
           LCD_index=0;
     }
     else
     {
        LCD_Set_Cursor(0,0); 
        for (i=(range3+LCD_index);i<(LCD_index+ LCD_SIZE+range3 );i++)
#ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else                   
                putcLCD(decode(LCD_BUF0[i]));
#endif 
           LCD_Set_Cursor(1,0); 
        LCD_index+= LCD_SIZE;  
        for (i=(LCD_index+range3);i<(LCD_index+ LCD_SIZE+range3 );i++)
 #ifdef USE_EEPROM
              putcLCD(decode(DATAEE_ReadByte(i)));
#else                   
                putcLCD(decode(LCD_BUF0[i]));
#endif   
        LCD_index+= LCD_SIZE; 
     }
 
 }    
 
    lcdOnFlag=1;
}   


#ifdef EPDD
void buttonUp_LCD(void)
#else
void buttonUp(void)
#endif
{
  if(! sysReady) return; 
 #ifdef BLE_MODE 
    sendBLE ();
#endif     
 //   DisplayFlag=1;
   Line_index=0;
 #ifdef USE_EPD
    #ifdef  DATA_LOGGER2  
 EPD_PowerOn();
 for (i;i<1000;i++);
                        EPD_print();
                       EPD_flash();
                       idle=0;
                       idle2=0;
                      // DisplayFlag=1;
                   
                      for (i=0;i<60000;i++);
                      
                         
    //  DATAEE_WriteByte(S2EEPFOM_H,(s2EEPFOM_lnd >>8 ));
   //   DATAEE_WriteByte (S2EEPFOM_L,(s2EEPFOM_lnd & 0x00FF));
   //   if ( (s2EEPFOM_lnd+S2EEPFOM_START)> S2EEPFOM_MAX ) s2EEPFOM_lnd=S2EEPFOM_MAX;
   //   for (i=0;i<s2EEPFOM_lnd; i++)
   //       EUSART2_Write( (const uint8_t )DATAEE_ReadByte((i+S2EEPFOM_START)));    
     #else
      EPD_PowerOn();
      for (i;i<1000;i++);
      EPD_Display();
     #endif
#else   
    displayP();
    
#endif  
   
  //  if (Line_index)
       
   // else
    //    Line_index+=64;
}

void uartWakeUp(void)
{
          idle=0;
		  idle2=0; 
}
#ifdef EPDD
void buttonDown_LCD(void)
#else
void buttonDown(void)
#endif
{
     if(! sysReady) return; 
 //   DisplayFlag=1;
 #ifndef USE_EPD    
    displayN();
#else
     #ifdef  DATA_LOGGER2  
     //   s2EEPFOM_lnd=0;
      //  DATAEE_WriteByte(S2EEPFOM_H,0);
      //  DATAEE_WriteByte (S2EEPFOM_L,0);
        EPD_PowerOn();
        for (i;i<1000;i++);
        DataCount=0;
        ClearLCD();
        EPD_print();
        EPD_flash();
        idle=0;
        idle2=0;
           for (i=0;i<60000;i++);
                    
     #else
       Line_index=64;
       EPD_PowerOn();
       for (i;i<1000;i++);
       EPD_Display();
       
     #endif
  
#endif     
   //   ClearLCD();
    //showTitle();
}
void sendBLE (void)
{ 
  

    for (i=0;i<BLE_MAX; i++)
    {
         if (MultiFlag) 
           EUSART2_Write( (const uint8_t )decode(LCD_BUF[i]));
         else
           EUSART2_Write( (const uint8_t )UUID_Buf[i]);    
    }
    EUSART2_Write( (const uint8_t )0x0a);
}

#else
#ifdef EPDD
void buttonUp_LCD(void)
#else
void buttonUp(void)
#endif
{
    
 #ifdef BLE_MODE 
    sendBLE ();
#endif     
 //   DisplayFlag=1;
 #ifdef USE_EPD
      EPD_Display();
#else   
    displayP();
    
#endif  
   //   ClearLCD();
}

void uartWakeUp(void)
{
          idle=0;
		  idle2=0; 
}
#ifdef EPDD
void buttonDown_LCD(void)
#else
void buttonDown(void)
#endif
{
 //   DisplayFlag=1;
 #ifndef USE_EPD    
    displayN();
#else
    
     displayP();
#endif     
   //   ClearLCD();
    //showTitle();
}


unsigned  int showBattery(void)
{
    unsigned  int conversion;
unsigned  int zz,xx;


//   while (1)
   {    
      
        ADC1_Initialize();
        ADC1_ChannelSelect(ADC1_CHANNEL_AN4);
        ADC1_Start();
        //Provide Delay
        for( i=0;i <1000;i++)
        {
        }
        ADC1_Stop();
        while(!ADC1_IsConversionComplete())
        {
            ADC1_Tasks();   
        }
        conversion = ADC1_ConversionResultGet();
   
      //   for( zz=0;zz <10;zz++)
   //     {
    //        for( xx=0;xx <60000;xx++)
     //   {
             
     //   }  
      //  }
   }       
return conversion;
}
void SLEEP(void)
{
   // PowerSaveSleep(); 
    
  /*
// Code example to put the Device in Idle and then
// wake the device when the ADC completes a conversion
SYSKEY = 0x0; // Write invalid key to force lock
SYSKEY = 0xAA996655; // Write Key1 to SYSKEY
SYSKEY = 0x556699AA; // Write Key2 to SYSKEY
//OSCCONCLR = 0x10; // Set the power-saving mode to an idle mode
 OSCCONSET = 0x10;//sleep mode
SYSKEY = 0x0; // Write invalid key to force lock
asm volatile ( "wait" ); // Put device in selected power-saving mode
// Code execution will resume here after wake and
// the ISR is complete
    */
   
    BatteryLevel=showBattery();
 
    if(BatteryLevel >BATTERY_LOW)
    {    

        IO_RC9_SetHigh();
        #ifdef LCD_OFF  
          LcdOff();
        #endif 
        U1MODEbits.WAKE = 1; // Bit7 No Wake up (since we don't sleep here)
      
       //  RCON=0x80;
        OSCCONSET = 0x10;//sleep mode
     //   asm volatile ( "wait" );
        PowerSaveSleep(); 
        LcdOn();
      //  IO_RC9_SetLow();
    }
    else
    { 
        //   IO_RC9_SetLow();
            LcdOn();
            sprintf (adcBuf,"Battery Low " );
            ClearLCD();
    //        delay_ms(3000);
            LCD_Set_Cursor(0,0);
     //       delay_ms(3000);
            putsLCD(adcBuf);
           for (j=0;j<MAX_WAIT_TIME;j++)
           {
             delay_ms(60000);
           }
           #ifdef LCD_OFF 
           // LcdOff(); 
          #endif  
    }
    
}

void sendBLE (void)
{ 
  

    for (i=0;i<BLE_MAX; i++)
    {
         if (MultiFlag) 
           UART2_Write( (const uint8_t )decode(LCD_BUF[i]));
         else
           UART2_Write( (const uint8_t )UUID_Buf[i]);    
    }
    UART2_Write( (const uint8_t )0x0a);
}
#endif


void sendTitle(void)
{
     putdata ('i');
     putdata ('-');
     putdata ('M');
     putdata ('o');
     putdata ('b');
     putdata ('i');
     putdata ('l');
     putdata ('e');
}

void EPD_PowerOn(void)
{
    IO_RC1_SetLow();
      for(i=0;i<500;i++); //20171021
   IO_RC0_SetLow();
  
   IO_RD2_SetHigh();
  // IO_RD2_SetLow();
    IO_RD5_SetLow();
    
    
    for(i=0;i<1000;i++); //20171021
    setFont8();
    setMaxLine();
    
}
void EPD_PowerOff(void)
{
   IO_RC0_SetHigh(); //VDD_SW
   IO_RC1_SetHigh(); // VEPD_SW //20170814 remove
   IO_RD2_SetHigh();  // +3V3_SW
   IO_RD5_SetHigh(); // VLCM_SW
}
void showTitle(void)
{
 EPD_Wait=  EPD_WAIT;
     ClearLCD();
   // delay_ms(30000);
  #ifdef  DATA_LOGGER2  
          LCD_Set_Cursor(0,0);
  #else    
          LCD_Set_Cursor(3,0);
  #endif
  //  delay_ms(30000);
    putcLCD('I');
    putcLCD('-');
    putcLCD('T');
    putcLCD('a');
    putcLCD('g');
    putcLCD('1');
    putcLCD('0');
    putcLCD('0');
    putcLCD(' ');
    putcLCD(V0); 
    putcLCD(V1);
    putcLCD(V2);
    putcLCD(V3);
    putcLCD(V4);
    

    EPD_print();
    EPD_flash();
 
      
}

#define SPI_RX_IN_PROGRESS 0x0
uint8_t SPI2_Read8bit(void)
{
    // Clear the Write Collision flag, to allow writing
    SSP2CON1bits.WCOL = 0;

    while(SSP2STATbits.BF == SPI_RX_IN_PROGRESS)
    {
    }

    return (SSP2BUF);
}
uint8_t SPI2_8bit(uint8_t data) //JasonSPI
{
    // Clear the Write Collision flag, to allow writing
    SSP2CON1bits.WCOL = 0;

    SSP2BUF = data;
    
    while(SSP2STATbits.BF == SPI_RX_IN_PROGRESS)
    {
    }
   // while( !SSPSTATbits.BF );  
   // for(i=0;i<1000;i++);
  //  SSP2CON2bits.
  //  SSP2STATbits.BF
    return (SSP2BUF);
}
void UART1_WaitEmpty (void)
{
    
     while ( PIE1bits.TX1IE)
     {
           for (j=0;j< UART_DELAY;j++);
     } //Wait for 
    while ( EUSART1_DataReady)
     {
                    unsigned char rc;
        
                    IO_RC2_Toggle();
        
                    rc=EUSART1_Read();
                     for (j=0;j< UART_DELAY;j++);
    }       
}
void UART1_Clear (void)
{
    
}

void EPD_print(void)
{
#ifdef  USE_SPI
 SPI2_8bit(0xFC);
 SPI2_8bit('@'); 
#else    
 EUSART2_Write(0xFC);
EUSART2_Write('@'); 
#endif
}

 void   EPD_flash(void)
 {
     if (0)//mode==1)
     {
         #ifdef  USE_SPI
  SPI2_8bit(0xFC);
  SPI2_8bit('_'); 
    i=0;
     while(i<SPI_DUMMY_SIZE)
   //  for(i=0;i<10;i++); // wait EPD
    {
    //   for(j=0;j<60000;j++);
       //for(j=0;j<60000;j++);
      
       SPI2_8bit(0xAA);
      // EPD_flash();
        i++;
    }
#else 
  EUSART2_Write(0xFC);
  EUSART2_Write('_');  
#endif
     }   
     else
     {
 
 #ifdef  USE_SPI
  SPI2_8bit(0xFC);
  SPI2_8bit('|'); 
    i=0;
     while(i<SPI_DUMMY_SIZE)
   //  for(i=0;i<10;i++); // wait EPD
    {
    //   for(j=0;j<60000;j++);
       //for(j=0;j<60000;j++);
      
       SPI2_8bit(0xAA);
      // EPD_flash();
        i++;
    }
#else 
  EUSART2_Write(0xFC);
  EUSART2_Write('|');  
#endif
      }
}
void ClearLCD(void)
{
 #ifdef  USE_SPI
    SPI2_8bit(0xFC);
    SPI2_8bit('#');
#else    
EUSART2_Write(0xFC);
EUSART2_Write('#');
#endif
} 

void setMode(void)
{
 #ifdef  USE_SPI
    SPI2_8bit(0xFC);
    SPI2_8bit('%');
    SPI2_8bit(mode);
#else    
EUSART2_Write(0xFC);
EUSART2_Write('%');
SPI2_8bit(mode);
#endif
}
void setFont8(void)
{
    #ifdef FONTS8
   // for (j=0;j<60000;j++);
    SPI2_8bit(0xFC); //FONTS
    SPI2_8bit('*');
     SPI2_8bit(8);
     setMaxLine();
    #endif    
    
}
void setMaxLine(void)
{
    #ifdef FONTS8
   // for (j=0;j<60000;j++);
    SPI2_8bit(0xFC); //FONTS
    SPI2_8bit('&');
     SPI2_8bit(36);
     SPI2_8bit(16);
	#else
	  SPI2_8bit(0xFC); //FONTS
      SPI2_8bit('&');
      SPI2_8bit(MAX_EPD_COL);
      SPI2_8bit(MAX_EPD_ROW);
    #endif    
    
}
void EPD_Show(void)
{
 #ifdef  USE_SPI

    setFont8();
    setMaxLine();
    SPI2_8bit(0xFC); //Init
    SPI2_8bit('S');
    i=0;
     while(i<SPI_DUMMY_SIZE)
   //  for(i=0;i<10;i++); // wait EPD
    {
    //   for(j=0;j<60000;j++);
       //for(j=0;j<60000;j++);
      
       SPI2_8bit(0xAA);
      // EPD_flash();
        i++;
    }
#else    
EUSART2_Write(0xFC);
EUSART2_Write('S');
#endif
} 
void EPD_Init(void)
{
 #ifdef  USE_SPI
    SPI2_8bit(0xFC);
    SPI2_8bit('I');
#else    
EUSART2_Write(0xFC);
EUSART2_Write('I');
#endif
} 
void putcLCD(unsigned char LCD_Char)
{
#ifdef  USE_SPI
   SPI2_8bit(LCD_Char);
#else   
	EUSART2_Write(LCD_Char);
#endif   
}
void LCD_Set_Cursor(unsigned char CurY, unsigned char CurX)
{
#ifdef  USE_SPI
    SPI2_8bit(0xFC);
    SPI2_8bit('$');
    SPI2_8bit(CurY);
    SPI2_8bit(CurX);
#else      
    EUSART2_Write(0xFC);
    EUSART2_Write('$');
    EUSART2_Write(CurY);
    EUSART2_Write(CurX);
#endif      
    
}
void RS232_Disable (void)
{

}
void RS232_WakeUp (void)
{
    
   
}    
//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
