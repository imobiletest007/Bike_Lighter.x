/*****************************************************************************
 *
 * Filename:
 * ---------
 *    Motion_sensor_custom.c
 *
 * Project:
 * --------
 *    Motion Sensor KX023 SPI Interface
 *
 * Description:
 * ------------
 *   This Module defines Serial Interface.
 *
 * Author:
 * -------
 * -------
 *
 *============================================================================
 ****************************************************************************/
#define MOTION_SENSOR_SUPPORT
#define uint16_t unsigned int
#define uint8_t unsigned char
#ifdef MOTION_SENSOR_SUPPORT

#include "motion_sensor_custom.h"   
#include "motion_sensor_spi.h"   
#include <xc.h>
#include "mcc_generated_files/mcc.h"
/* ---------------------------------------------------------------------------- */
/*
* FUNCTION                                                            
*	motion_sensor_check_softwave_ready
*
* DESCRIPTION                                                           
*   check sensor softwave function completion
*
* PARAMETERS
*  received data
*	
* RETURNS
*	None
*/    
void motion_sensor_check_softwave_ready(void)
{    
	do
	{
		__delay_ms(50);
	} while ((motion_sensor_read_data(MOTION_SENSOR_CTRL_REGB) & 0x80));
}


/*
* FUNCTION                                                            
*	acc_sensor_pwr_up
*
* DESCRIPTION                                                           
*   	This function is to power up acceleration module
*
* CALLS  
*
* PARAMETERS
*  None
*	
* RETURNS
*	None
*/    
//test// //char temp1=0;
//test// //char temp2=0;
//test// //char temp3=0;
//test// //char temp4=0;
void acc_sensor_pwr_up(void)
{
	__delay_ms(30);	//delay for power on reset	
	motion_sensor_enable_bits(0x80, MOTION_SENSOR_CTRL_REGB);	//KX023 reset
	motion_sensor_check_softwave_ready();						//wait & check reset completion
	
	motion_sensor_write_data(0x00, MOTION_SENSOR_CTRL_REGA);	//set to low current, 2g
	motion_sensor_write_data(0x02, MOTION_SENSOR_ODCNTL_REGC);	//set ODR 50Hz
	motion_sensor_write_data(0x10,MOTION_SENSOR_INT_CTRL_REG1);	//disable all INT1 interrupt funs
	motion_sensor_write_data(0x10,MOTION_SENSOR_INT_CTRL_REG5);	//disable all INT2 interrupt funs
	motion_sensor_enable_bits(0x80, MOTION_SENSOR_CTRL_REGA);	//set to work mode
}


/*
* FUNCTION                                                            
*	acc_sensor_pwr_down 
*
* DESCRIPTION                                                           
*   	This function is to power down acceleration
*
* CALLS  
*
* PARAMETERS
*	None
*	
* RETURNS
*	None
*/ 
void acc_sensor_pwr_down(void)
{   
	motion_sensor_disable_bits(0x80, MOTION_SENSOR_CTRL_REGA);	// into standby mode
}


/*
* FUNCTION                                                            
*	acc_sensor_init
*
* DESCRIPTION                                                           
*   	This function is to initialize acceleration module
*
* CALLS  
*
* PARAMETERS
*	None
*	
* RETURNS
*	None
*/ 
void acc_sensor_init(void)
{  
	motion_senosr_interface_init();
}


/*
* FUNCTION                                                            
*	acc_sensor_interrupt_initial
*
* DESCRIPTION                                                           
*   Set & enable interrupt function
*
* CALLS  
*
* PARAMETERS
*  None
*	
* RETURNS
*	None
*/    
void acc_sensor_interrupt_initial(void)
{
	motion_sensor_disable_bits(0x80, MOTION_SENSOR_CTRL_REGA);		// into standby mode
	motion_sensor_write_data(0x00, MOTION_SENSOR_INT_CTRL_REG1);
	motion_sensor_enable_bits(0x30, MOTION_SENSOR_INT_CTRL_REG1);	// interrupt-pin (5) enable, active high, latch, no WUF
    motion_sensor_write_data(0x0C,0x32);	//etilt_angle_ll
    motion_sensor_write_data(0x2A,0x33);	//etilt_angle_HL
    //ADD INT interrupt
    motion_sensor_enable_bits(0x01, MOTION_SENSOR_INT_CTRL_REG4);	// interrupt-pin (5) enable, active high, latch, no WUF
    //ADD INT interrupt
	motion_sensor_enable_bits(0x80, MOTION_SENSOR_CTRL_REGA);		// set to work mode 
}


/*
* FUNCTION                                                            
*	acc_sensor_WUF_initial
*
* DESCRIPTION                                                           
*   Set & enable the WUF function
*
* CALLS  
*
* PARAMETERS
*  None
*	
* RETURNS
*	None
*/    
void acc_sensor_wuf_initial(void)
{
	motion_sensor_disable_bits(0x80, MOTION_SENSOR_CTRL_REGA);	// into standby mode
	motion_sensor_write_data(0x00, MOTION_SENSOR_WUFC);			// WUF_TIMER
	motion_sensor_write_data(0x08, MOTION_SENSOR_ATH);			//  WUF_THRESH
	motion_sensor_enable_bits(0x02, MOTION_SENSOR_CTRL_REGA); 	// enable WUF function  
	motion_sensor_enable_bits(0x80, MOTION_SENSOR_CTRL_REGA);	// set to work mode 
}


/*
* FUNCTION                                                            
*	acc_sensor_clear_int_status
*
* DESCRIPTION                                                           
*   Release the latch mode of interrupt
*
* CALLS  
*
* PARAMETERS
*  None
*	
* RETURNS
*	None
*/
void acc_sensor_clear_interrupt_status(void)
{
	uint8_t i;
	i = motion_sensor_read_data(MOTION_SENSOR_INT_REL);
}


/*
* FUNCTION                                                            
*	acc_sensor_read_status
*
* DESCRIPTION                                                           
*   Read the interrupt status
*
* CALLS  
*
* PARAMETERS
*  *status1_value : status 1 value 
*  *status2_value : status 2 value 
*	 
* RETURNS
*	None
*/

uint8_t status1_value=0;
uint8_t status2_value=0;
uint8_t status3_value=0;
uint8_t status4_value=0;
uint8_t status5_value=0;
uint8_t status6_value=0;
void acc_sensor_read_status(void)
{
//    motion_sensor_disable_bits(0x80, MOTION_SENSOR_CTRL_REGA);		// into standby mode READ
	status1_value = motion_sensor_read_data(MOTION_SENSOR_INS1);
	status2_value = motion_sensor_read_data(MOTION_SENSOR_INT_STATUS);
    status3_value = motion_sensor_read_data(MOTION_SENSOR_INS2);
    status4_value = motion_sensor_read_data(MOTION_SENSOR_INS3);
    status5_value = motion_sensor_read_data(MOTION_SENSOR_TSPP);
    status6_value = motion_sensor_read_data(MOTION_SENSOR_INT_CTRL_REG4);

//    	motion_sensor_enable_bits(0x80, MOTION_SENSOR_CTRL_REGA);	// set to work mode 
    NOP();
     NOP();
}


/*Customization functin*/ 
/*
* FUNCTION                                                            
*	acc_sensor_get_acc
*
* DESCRIPTION                                                           
*   	This function is to get ADC value in every axis
*
* CALLS  
*
* PARAMETERS
*	x_adc: ADC value in X-axis
*	y_adc: ADC value in Y-axis
*	z_adc: ADC value in Z-axis
*
* RETURNS
*	None
*/ 

int x_value[MAX_COUNT]={0};
int y_value[MAX_COUNT]={0};
int z_value[MAX_COUNT]={0};
long int gx=0, gy=0, gz=0;
uint8_t g_index=0;

int acc_sensor_get_acc()
{	
	static uint8_t temp_data[6];
	motion_sensor_read_multi_data(MOTION_SENSOR_XOUT_L, temp_data, 6);
	
	x_value[g_index] = ((temp_data[1] << 8) | temp_data[0]);		/* x-axis */      
	y_value[g_index] = ((temp_data[3] << 8) | temp_data[2]);		/* y-axis */      
	z_value[g_index] = ((temp_data[5] << 8) | temp_data[4]);		/* z-axis */   
    ++g_index;
    if (g_index >= MAX_COUNT) {
        gx = gy = gz = 0;
        for (int i=0; i<MAX_COUNT; ++i) {
            gx += x_value[i];
            gy += y_value[i];
            gz += z_value[i];
        }
        gx /= 8196;
        gy /= 8196;
        gz /= 8196;
        g_index=0;
        return 1;
    } else return 0;
}

#endif
