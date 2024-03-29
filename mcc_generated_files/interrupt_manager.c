/**
  @Generated Interrupt Manager File

  @Company:
    Microchip Technology Inc.

  @File Name:
    interrupt_manager.c

  @Summary:
    This is the Interrupt Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  @Description:
    This header file provides implementations for global interrupt handling.
    For individual peripheral handlers please see the peripheral driver for
    all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
        Device            :  PIC18LF46K22
        Driver Version    :  1.02
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

#include "interrupt_manager.h"
#include "mcc.h"

void  INTERRUPT_Initialize (void)
{
    // Enable Interrupt Priority Vectors
    RCONbits.IPEN = 1;

    // Assign peripheral interrupt priority vectors

    // TXI - high priority
    IPR1bits.TX1IP = 1;

    // RCI - high priority
    IPR1bits.RC1IP = 1;

    // TMRI - high priority
    IPR1bits.TMR1IP = 1;

    // RBI - high priority
    INTCON2bits.RBIP = 1;

    // TXI - high priority
    IPR3bits.TX2IP = 1;

    // RCI - high priority
    IPR3bits.RC2IP = 1;

   // INT1I - low priority
    INTCON3bits.INT1IP = 0;  
    
   // INT2I - low priority
    INTCON3bits.INT2IP = 0;  
    
    // ADI - low priority
    IPR1bits.ADIP = 0;
}

void __interrupt(high_priority) INTERRUPT_InterruptManagerHigh (void)
{
   // interrupt handler
    if(PIE1bits.TX1IE == 1 && PIR1bits.TX1IF == 1)
    {
        EUSART1_Transmit_ISR();
    }
    else if(PIE1bits.RC1IE == 1 && PIR1bits.RC1IF == 1)
    {
        EUSART1_Receive_ISR();
    }
    else if(PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF == 1)
    {
        TMR1_ISR();
    }
    else if(INTCONbits.INT0IE == 1 && INTCONbits.INT0IF == 1) 
    {
        PIN_MANAGER_IOC();
    }
      else if(INTCON3bits.INT1IE == 1 && INTCON3bits.INT1IF == 1) 
    {
        PIN_MANAGER_IOC();
    }
      else if(INTCON3bits.INT2IE == 1 && INTCON3bits.INT2IF == 1) 
    {
        PIN_MANAGER_IOC();
    }
}

void __interrupt(low_priority) INTERRUPT_InterruptManagerLow (void)
{
    // interrupt handler
     if(INTCON3bits.INT2IE == 1 && INTCON3bits.INT2IF == 1)
    {
        INT2_ISR();
    }
    else if(INTCON3bits.INT1IE == 1 && INTCON3bits.INT1IF == 1)
    {
        INT1_ISR();
    }
    else  if(PIE1bits.ADIE == 1 && PIR1bits.ADIF == 1)
    {
        ADC_ISR();
    }
}
/**
 End of File
*/
