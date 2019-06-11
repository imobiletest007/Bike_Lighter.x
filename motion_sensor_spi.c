/*****************************************************************************
 *
 * Filename:
 * ---------
 *    Motion_sensor_spi.c
 *
 * Project:
 * --------
 *    Motion Sensor SPI Interface
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
#ifdef MOTION_SENSOR_SUPPORT

#include	"motion_sensor_spi.h"  
#define uint16_t_t unsigned int
#define uint8_t unsigned char
#include "mcc_generated_files/mcc.h"
/*************************************************************************
* FUNCTION
*	MS_Delay
*
* DESCRIPTION
*	Using for delay time.
*
* PARAMETERS
*  	time : system executed cycle (1 cycle = 1usec)
*
* RETURNS
*	None 
* 
* GLOBALS AFFECTED
*	None
*
*************************************************************************/
void MS_Delay(uint16_t time)
{
	uint16_t i, j;
	for(i=0;i<time;i++)
	{
		for(j=0;j<1000;j++);
	}
}

/*************************************************************************
* FUNCTION
*	motion_senosr_interface_init
*
* DESCRIPTION
*	Initialization SPI interface.
*
* PARAMETERS
*  	None
*
* RETURNS
*	None 
* 
* GLOBALS AFFECTED
*	None
*
*************************************************************************/
void motion_senosr_interface_init(void) 
{       
	SET_MS_CLK_LOW;
	SET_MS_MOSI_HIGH;
	SET_MS_CS_HIGH;
	
//	SET_MS_CLK_OUTPUT;
//	SET_MS_MOSI_OUTPUT;	
//	SET_MS_CS_OUTPUT;
//	SET_MS_MISO_INPUT;
}

/*************************************************************************
* FUNCTION
*	motion_sensor_send_byte
*
* DESCRIPTION
*	send a byte data through software SPI interface
*
* PARAMETERS
*	send_byte : send a byte
*
* RETURNS
*	None 
*
* GLOBALS AFFECTED
*	None
*
*************************************************************************/

void motion_sensor_send_byte(uint8_t send_byte)
{
 int i;
char j;
	for (i=7;i>=0;i--)
	{	
		if (send_byte & (1<<i))
		{
			SET_MS_MOSI_HIGH;
		}
		else
		{
			SET_MS_MOSI_LOW;
		}
		SET_MS_CLK_HIGH;
		SET_MS_CLK_LOW;
	}
  NOP();
}


/*************************************************************************
* FUNCTION
*	motion_sensor_get_byte
*
* DESCRIPTION
*	get a byte data from software SPI interface
*
* PARAMETERS
*	None
*
* RETURNS
*	value : receiver a byte data
*
* GLOBALS AFFECTED
*	None
*
*************************************************************************/
uint8_t motion_sensor_get_byte(void)
{
	uint8_t i; 
	uint8_t value = 0;
		
    	for(i=0;i<8;i++)
	{	
		SET_MS_CLK_HIGH;
		
		if (GET_MS_DATA) 
		{
			value |= (1 << i);
		}
		
		SET_MS_CLK_LOW;
	}
	
	return value;
}	

/*************************************************************************
* FUNCTION
*	motion_sensor_write_data
*
* DESCRIPTION
*	send a byte data to sensor's register via SPI interface.
*
* PARAMETERS
* 	data : 	register value
*	addr :  register address
*
* RETURNS
*	None
*   
* GLOBALS AFFECTED
*	None
*
*************************************************************************/
void motion_sensor_write_data(uint8_t data, uint8_t addr)
{
	SET_MS_CS_LOW;
    SPI1_Exchange8bit(addr & 0x7F);
	SPI1_Exchange8bit(data);	
	SET_MS_CS_HIGH;
    NOP();
    SET_MS_CS_LOW;
    SPI1_Exchange8bit(addr & 0x7F);
	SPI1_Exchange8bit(data);	
	SET_MS_CS_HIGH;
   
}

/*************************************************************************
* FUNCTION
*	motion_sensor_read_data
*
* DESCRIPTION
*	Read a byte data from sensor's register via SPI interface.
*
* PARAMETERS
*  	addr : register address
*
* RETURNS
*  	value : receiver a byte data
*   
* GLOBALS AFFECTED
*	None
*
*************************************************************************/

uint8_t motion_sensor_read_data(uint8_t addr)
{
	uint8_t value = 0;
	
	SET_MS_CS_LOW;
    SPI1_Exchange8bit(addr | 0x80);
	value = SPI1_Exchange8bit(NULL);

	SET_MS_CS_HIGH;
	
	return value;
}

/*************************************************************************
* FUNCTION
*	motion_sensor_read_multi_data
*
* DESCRIPTION
*	Read a byte data from sensor's register via SPI interface.
*
* PARAMETERS
*   addr : register address
*  *data : data store (point)
*	 num : receiver number
*
* RETURNS
*  	None
*   
* GLOBALS AFFECTED
*	None
*
*************************************************************************/
void motion_sensor_read_multi_data(uint8_t addr, uint8_t *data, uint8_t num)
{
	uint8_t cn;
	cn = num;
	
	SET_MS_CS_LOW;
	SPI1_Exchange8bit(addr | 0x80);
	
	for (uint8_t i=0; i<=cn; i++) 
	{
		data[i] = SPI1_Exchange8bit(NULL);
	}		
	
	SET_MS_CS_HIGH;
}

/*************************************************************************
* FUNCTION
*	motion_sensor_disable_bits
*
* DESCRIPTION
*	Storage old value & mixed with new value (and)
*
* PARAMETERS
* 	data : 	register value
*	addr :  register address
*
* RETURNS
*  None
* 
* GLOBALS AFFECTED
*	None
*
*************************************************************************/
void motion_sensor_disable_bits(uint8_t data, uint8_t addr)
{    
	uint8_t temp_data;
	
	temp_data = motion_sensor_read_data(addr);
	temp_data &= (~data);
	motion_sensor_write_data(temp_data, addr);
}

/*************************************************************************
* FUNCTION
*	motion_sensor_enable_bits
*
* DESCRIPTION
*	Storage old value & mixed with new value (or)
*
* PARAMETERS
* 	data : 	register value
*	addr :  register address
*
* RETURNS
*  None
* 
* GLOBALS AFFECTED
*	None
*
*************************************************************************/
void motion_sensor_enable_bits(uint8_t data, uint8_t addr)
{    
	uint8_t temp_data;
	
	temp_data = motion_sensor_read_data(addr);
	temp_data |= data;
	motion_sensor_write_data(temp_data, addr);
}

#endif

