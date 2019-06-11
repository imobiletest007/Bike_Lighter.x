/*****************************************************************************
 *
 * Filename:
 * ---------
 *    Motion_sensor_spi.h
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

#ifndef MOTION_SENSOR_SPI_H
#define MOTION_SENSOR_SPI_H
//#define a0                LATCbits.LATC0
//#define res               LATCbits.LATC1
//#define cs                LATCbits.LATC2
//#define scl               LATCbits.LATC3
//#define sdo               LATCbits.LATC5
#define MS_SPI_CLK_PIN   		LATCbits.LATC3	//define GPIO7 for spi CLK Singal
#define MS_SPI_MOSI_PIN  		LATCbits.LATC5  	//define GPIO6 for spi MOSI Singal (data out)
#define MS_SPI_MISO_PIN  		LATCbits.LATC4   	//define GPIO5 for spi MISO Singal (data in)
#define MS_SPI_CS_PIN 	 		LATBbits.LATB3  	//define GPIO4 for spi CS Singal

//#define SET_MS_CLK_OUTPUT		GPIO_InitIO(OUTPUT,MS_SPI_CLK_PIN);
//#define SET_MS_MOSI_OUTPUT		GPIO_InitIO(OUTPUT,MS_SPI_MOSI_PIN);
//#define SET_MS_MISO_INPUT		GPIO_InitIO(INPUT,MS_SPI_MISO_PIN);
//#define SET_MS_CS_OUTPUT		GPIO_InitIO(OUTPUT,MS_SPI_CS_PIN);

#define SET_MS_CLK_HIGH			LATCbits.LATC3=1;
#define SET_MS_CLK_LOW			LATCbits.LATC3=0;
#define SET_MS_MOSI_HIGH		LATCbits.LATC5=1;
#define SET_MS_MOSI_LOW			LATCbits.LATC5=0;
#define SET_MS_CS_HIGH			LATBbits.LATB3=1;
#define SET_MS_CS_LOW			LATBbits.LATB3=0;

#define GET_MS_DATA				MS_SPI_MISO_PIN


#endif

#define uint16_t unsigned int
#define uint8_t unsigned char
void MS_Delay(uint16_t value);
void motion_senosr_interface_init(void);
void motion_sensor_send_byte(uint8_t send_byte);
uint8_t motion_sensor_get_byte(void);
void motion_sensor_write_data(uint8_t data, uint8_t addr);
uint8_t motion_sensor_read_data(uint8_t addr);
void motion_sensor_read_multi_data(uint8_t addr, uint8_t *data, uint8_t num);
void motion_sensor_disable_bits(uint8_t data, uint8_t addr);
void motion_sensor_enable_bits(uint8_t data, uint8_t addr);

