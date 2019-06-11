/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
        Device            :  PIC18LF46K22
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "mcc_generated_files/mcc.h"

#include "motion_sensor_custom.h"   
#include "motion_sensor_spi.h"

#define PWM1_INITIALIZE_DUTY_VALUE    249  // see epwm1.c

uint8_t g_vison=0x00;
uint8_t str1[160];
extern uint8_t tmr1_flag_1s;
extern uint8_t tmr1_flag_100ms;

extern uint8_t status1_value;
extern uint8_t status2_value;
extern uint8_t status3_value;
extern uint8_t status4_value;
extern uint8_t status5_value;
extern uint8_t status6_value;

extern long int gx, gy, gz;
extern int x_value[MAX_COUNT], y_value[MAX_COUNT], z_value[MAX_COUNT];

void putrs2USART(const char *data);
void g_sensor_initial(void);
void wake_up_initial(void);
static void UART_Show_Version(void);

#define LED_OFF    M_INDICATOR_SetHigh
#define LED_ON     M_INDICATOR_SetLow
#define LED_FLASH  M_INDICATOR_Toggle

uint8_t pause=0;
int gx1=0, gy1=0, gz1=0;
int gx2=0, gy2=0, gz2=0;
int dx=0, dy=0, dz=0;

int light=5;
int duty[11] = { 499, 495, 480, 459, 409, 359, 299, 249, 199, 99, 0 };
void pwm_light(int idx) {
    if (idx < 0) idx = 0;
    else if (idx > 10) idx = 10;
    if (idx == 0) duty[0] = 501; //  A BUG
    EPWM1_LoadDutyValue(duty[idx]);
}
void UART_Demo_Command_INT(void)
{
    uint8_t temp;
    if(EUSART2_is_rx_ready())
    {
        temp=EUSART2_Read();  // read a byte for RX
        EUSART2_Write(temp);  // send a byte to TX  (from Rx)

        switch(temp)    // check command  
        {
            case 'H':
            case 'h':
            {
                ++light;
                if (light >= 10) light = 10;
                pwm_light(light);
                printf("+ : duty %d  light %d\r\n", duty[light], light);
                break;
            }
            case 'L':
            case 'l':
            {
                --light;
                if (light < 0) light = 0;
                pwm_light(light);
                printf("- : duty %d  light %d\r\n", duty[light], light);
                break;
            }
            case 'P':
            case 'p':
            {
                pause = pause?0:1;
                break;
            }
            case 'V':
            case 'v':
            {
                UART_Show_Version();
                break;
            }
            default:
            {
                printf("? : other key '%c'", temp);
                break;
            }
        }
    }
}
static void UART_Show_Version(void)
{
    printf("\r\nRTC from Neil, modified by wade 12\r\n");
    printf("\r\nPWM - 1\r\n");
    printf("UART Communications 8-bit Rx and Tx\r\n");
    printf("Keyboard Type H : LED ON   Type L: LED OFF \r\n");
}

int last_sensor = 0;
uint16_t light_level;

void main(void)
{
//    uint8_t light_level;
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptHighEnable();
    INTERRUPT_GlobalInterruptLowEnable();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();

    g_sensor_initial();//sleep ok
    wake_up_initial();
    UART_Show_Version();

    /* ?????????????? TMR1 ???, ????
    TRISCbits.TRISC0 = 0;
    LATCbits.LATC0 =0;
    __delay_ms(10);
    LATCbits.LATC0 =1;
    __delay_ms(10);
    LATCbits.LATC0 =0;
    NOP();
    NOP();     
     */

    motion_sensor_write_data(0xC8,MOTION_SENSOR_INT_CTRL_REG1);	//enable all INT1 interrupt funs
    motion_sensor_write_data(0x00,MOTION_SENSOR_ATH);	//enable all INT1 interrupt funs    

    while (1) {
        UART_Demo_Command_INT();
        if (acc_sensor_get_acc()) {
            acc_sensor_read_status();
            acc_sensor_clear_interrupt_status(); // it can read G-sensor INT 
            if(tmr1_flag_100ms) {
                tmr1_flag_100ms = 0;
                gx1 = (int)gx;
                gy1 = (int)gy;
                gz1 = (int)gz;
                dx = (gx2-gx1); dx *= dx;
                dy = (gy2-gy1); dy *= dy;
                dz = (gz2-gz1); dz *= dz;
                last_sensor = (last_sensor*3 + dx+dy+dz) / 4;
                gx2 = gx1; gy2 = gy1; gz2 = gz1;
//                if (dx > 0 || dy > 0 || dz > 0) {
                if (last_sensor>10)
                {
                    light_level=10;
                }else 
                {
//                    light_level=499;
                    light_level+=10;
                    if (light_level>=499) light_level = 499;
//                    if (light_level<450 && light_level>0)
//                    {
//                        light_level=light_level+3;
//                    }else
//                    {
//                        light_level=499;
//                    
                }
                EPWM1_LoadDutyValue(light_level);
//                    pwm_light(last_sensor);
                    if (!pause) printf("sensor = %d (%d, %d, %d)\r\n", last_sensor, dx, dy, dz);
//                }
            }
        }
    }
}
void putrs2USART(const char *data)
{
  do
  {  // Transmit a byte
//    while(Busy2USART());
    EUSART2_Write(*data);               // ?? enusart1.c ??? (from MCC)
  } while( *data++ );
}

static void g_sensor_vision(void)
{

       __delay_ms(2);
         M_G_SSI_SetLow();
         SPI1_Exchange8bit(MOTION_SENSOR_WHO_AM_I | 0x80 );
         g_vison=SPI1_Exchange8bit(NULL);
         M_G_SSI_SetHigh();
         NOP();
         NOP();     
}

void g_sensor_initial(void)
{
        acc_sensor_pwr_down();
        acc_sensor_init();
        acc_sensor_interrupt_initial();
        acc_sensor_wuf_initial();
        acc_sensor_clear_interrupt_status();
        motion_sensor_write_data(0x89, MOTION_SENSOR_WHO_AM_I);             
        g_sensor_vision();
}

void wake_up_initial(void)
{
    INTCONbits.INT0IE =  1;
    INTCONbits.INT0IF  = 0;
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT1IF = 0;
    INTCON3bits.INT2IE = 1;
    INTCON3bits.INT2IF = 0;


}

#ifndef WADE
void ready_to_sleep(void)
{

    //////////// it can off all
    PIE1bits.ADIE = 0;
    PIE1bits.RC1IE = 0;
    PIE3bits.RC2IE = 0;

    LATE = 0x00;    
    LATD = 0x00;    
    LATA = 0x00;    
    LATB = 0x00;    
    LATC = 0x00;    
////    M_INDICATOR_SetHigh() ; 21uA-28uA;
//////    //it is not for board
//////    TRISE = 0x00;
//////    TRISA = 0x00;
//////    TRISB = 0x00;
//////    TRISC = 0x00;
//////    TRISD = 0x00;
//////
//////    ANSELC = 0x00;
//////    ANSELB = 0x00;
//////    ANSELD = 0x00;
//////    ANSELE = 0x00;
//////    ANSELA = 0x00;
//////     //it is not for board
//    //////////it need to open,  for board`
     M_INDICATOR_SetHigh();//it need to open, it 30uA for board
     LATCbits.LATC0 = 1;//it need to open,  it 4mA for board   
     ///
     LATDbits.LATD5 = 1;//it need to open,  it 4mA for board   
     LATDbits.LATD2 = 1;//it need to open,  it 4mA for board   
     LATCbits.LATC1 = 1;//it need to open,  it 4mA for board   
       
     OSCCON=0x00;  
       //////////it need to open,  for board
   // PIE1bits.ADIE = 0; ///ADD 2017_12_20
//    if(BAUDCON2bits.RCIDL ==1)
//    {
//        BAUDCON2bits.WUE =1; // if it need wake up again, it need to add this in isr
//        OSCCONbits.IDLEN=1;
        Sleep();
        __delay_ms(30);
        SYSTEM_Initialize();
        wake_up_initial();
      TRISCbits.TRISC0 = 0;//    power_24_on_SetHigh() ;
    LATCbits.LATC0 =0;//    power_24_on_SetHigh() ; TMR1_StartTimer();
       __delay_ms(40);
    LATCbits.LATC0 =1;//    power_24_on_SetHigh() ; TMR1_StartTimer();
     __delay_ms(40);
    LATCbits.LATC0 =0;//    power_24_on_SetHigh() ; TMR1_StartTimer();
    NOP();
     NOP();
     com_bluetoooth_set();
//        __delay_ms(5);
//        BAUDCON2bits.WUE =0;
    
}

uint8_t temp=0;
unsigned char 	EEPROM_Data1 , EEPROM_Data2,EEPROM_Data3 ;
unsigned char	SPI_Status ;
unsigned char cc=0;

void mcu18_spi_24_send(void)
{

 SPI2_Exchange8bit(cc++);/////ex: 0x34  0x34 
  EEPROM_Data1=SPI2_Exchange8bit(0x00);/////ex: 0x34  0x34 
//     EEPROM_Data2 =SPI2_Exchange8bit(0x45);/////ex: 0x34  0x34 
//     EEPROM_Data3 =SPI2_Exchange8bit(0x56);/////ex: 0x34  0x34  

        #ifdef test
       sprintf(str1, "EEPROM_Data1=%x EEPROM_Data2=%x EEPROM_Data3=%x  \r\n",EEPROM_Data1 , EEPROM_Data2,EEPROM_Data3);
       putrs2USART(str1);  
        #endif
       NOP();
       NOP();
}
void	SPI_Delay(void)
{
	int	LoopSPI ;
	for (LoopSPI = 0 ; LoopSPI < 100 ; LoopSPI ++ ) ;

}
void g_sensor_int_value(void)

{
      #ifdef test
            acc_sensor_clear_interrupt_status(); // it can read G-sensor INT 
            sprintf(str1, "INT_STATUS=%x, \r\n",status2_value);
            putrs2USART(str1);  
            motion_sensor_write_data(int_value_temp,MOTION_SENSOR_ATH);	//ath    
            ////if you want to change , it need to modify this .
//            motion_sensor_write_data(0x0C,0x32);	//etilt_angle_ll default
//            motion_sensor_write_data(0x2A,0x33);	//etilt_angle_HL default
//            motion_sensor_write_data(0x1C,0x32);	//etilt_angle_ll
//            motion_sensor_write_data(0x3A,0x33);	//etilt_angle_HL
            sersonr_temp=motion_sensor_read_data(MOTION_SENSOR_ATH);
            sprintf(str, "int_value_temp=%x,sersonr_temp=%x\r\n", int_value_temp,sersonr_temp);
            putrs2USART(str);
            ///
                     ///
         /////debug g_gensor_value
//         sprintf(str1, "INS1=%x,INT_STATUS=%x,INS2=%x,INS3=%x,TSPP= %x,INT_REL=%x \r\n",status1_value,status2_value,status3_value,status5_value,status7_value);
//         putrs2USART(str1);  
          #endif
}
#endif
/**
 End of File
*/