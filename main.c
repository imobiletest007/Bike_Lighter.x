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
//    wake_up_initial();
    UART_Show_Version();

    motion_sensor_write_data(0xC8,MOTION_SENSOR_INT_CTRL_REG1);	//enable all INT1 interrupt funs
    motion_sensor_write_data(0x00,MOTION_SENSOR_ATH);	//enable all INT1 interrupt funs    

    while (1) 
    {
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
//                    light_level=10;
                    light_level-=2;
                    if (light_level<=10) light_level = 10;
                }else 
                {
//                    light_level=499;
                    light_level+=10;
                    if (light_level>=499) light_level = 499;
                }
                EPWM1_LoadDutyValue(light_level);
                if (!pause) printf("sensor = %d (%d, %d, %d)\r\n", last_sensor, dx, dy, dz);
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

//       __delay_ms(2);
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

/*void wake_up_initial(void)
{
    INTCONbits.INT0IE =  1;
    INTCONbits.INT0IF  = 0;
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT1IF = 0;
    INTCON3bits.INT2IE = 1;
    INTCON3bits.INT2IF = 0;


}*/


/**
 End of File
*/