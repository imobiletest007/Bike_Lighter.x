/*****************************************************************************
 *
 * Filename:
 * ---------
 *    Motion_sensor_custom.c
 *
 * Project:
 * --------
 *    Motion Sensor KX023 I2C Interface
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
#ifndef _MOTION_SENSOR_CUSTOM_H
#define _MOTION_SENSOR_CUSTOM_H
#define uint16_t unsigned int
#define uint8_t unsigned char
/* Gsensor slave address and read write bit */
#define MOTION_SENSOR_ADDRESS    		0x1F 	// when "PIN-ADDR" connect to "IO_VDD"
// #define MOTION_SENSOR_ADDRESS    	0x1E 	// when "PIN-ADDR" connect to "GND"

/* Gsensor 8bit register */
#define  MOTION_SENSOR_XOUT_L    		0x06
#define  MOTION_SENSOR_XOUT_H    		0x07
#define  MOTION_SENSOR_YOUT_L    		0x08
#define  MOTION_SENSOR_YOUT_H    		0x09
#define  MOTION_SENSOR_ZOUT_L    		0x0A
#define  MOTION_SENSOR_ZOUT_H    		0x0B
#define  MOTION_SENSOR_DCST_RESP   		0x0C
#define  MOTION_SENSOR_WHO_AM_I   		0x0F
#define  MOTION_SENSOR_TSPP				0x11
#define  MOTION_SENSOR_INS1				0x12
#define  MOTION_SENSOR_INS2				0x13
#define  MOTION_SENSOR_INS3				0x14
#define  MOTION_SENSOR_INT_STATUS		0x15
#define  MOTION_SENSOR_INT_REL	 		0x17
#define  MOTION_SENSOR_CTRL_REGA 		0x18
#define  MOTION_SENSOR_CTRL_REGB 		0x19
#define  MOTION_SENSOR_CTRL_REGC 		0x1A
#define  MOTION_SENSOR_ODCNTL_REGC 		0x1B
#define  MOTION_SENSOR_INT_CTRL_REG1 	0x1C
#define  MOTION_SENSOR_INT_CTRL_REG2 	0x1D
#define  MOTION_SENSOR_INT_CTRL_REG3 	0x1E
#define  MOTION_SENSOR_INT_CTRL_REG4 	0x1F
#define  MOTION_SENSOR_INT_CTRL_REG5 	0x20
#define  MOTION_SENSOR_INT_CTRL_REG6 	0x21
#define  MOTION_SENSOR_WUFC		 		0x23
#define  MOTION_SENSOR_TDTRC			0x24
#define  MOTION_SENSOR_TDTC	 			0x25	
#define  MOTION_SENSOR_TTH			 	0x26	
#define  MOTION_SENSOR_TTL			 	0x27	
#define  MOTION_SENSOR_FTD			 	0x28	
#define  MOTION_SENSOR_STD			 	0x29	
#define  MOTION_SENSOR_TLT				0x2A	
#define  MOTION_SENSOR_TWS				0x2B	
#define  MOTION_SENSOR_ATH		 		0x30
#define  MOTION_SENSOR_SELF_TEST		0x60

/* interrupt configuration */
#define MOTION_SENSOR_INT_SUPPORT   FALSE
#define MOTION_SENSOR_INT           4
#define MOTION_SENSOR_INT_LEVEL     LEVEL_HIGH
#define MOTION_SENSOR_LOW_G_MASK    0x1
#define MOTION_SENSOR_HIGH_G_MASK   0x2

#define MAX_COUNT 8
#define _g_sens  16384 // 9.8?

/* function */
void motion_sensor_check_softwave_ready(void);
void acc_sensor_pwr_up(void);
void acc_sensor_pwr_down(void);
void acc_sensor_init(void);
void acc_sensor_interrupt_initial(void);
void acc_sensor_wuf_initial(void);
void acc_sensor_clear_interrupt_status(void);
void acc_sensor_read_status(void);
int acc_sensor_get_acc();

#endif
