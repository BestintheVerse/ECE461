#include <MKL25Z4.H>
#include "mma8451.h"
#include "gpio_defs.h"
#include "i2c.h"
#include "delay.h"

int16_t acc_X=0, acc_Y=0, acc_Z=0;
volatile int i2c_is_complete;
volatile uint8_t isr_data[6];

//initializes mma8451 sensor
//i2c has to already be enabled
int init_mma()
{
	uint8_t data[1];

	//set active mode, 14 bit samples, 2g full scale and 800 Hz ODR 
	data[0] = 0x01;
	i2c_write_bytes(MMA_ADDR, REG_CTRL1, data, 1);
	return 1;
}

void read_full_xyz()
{
	int i;
	uint8_t data[6];
	int16_t temp[3];
	
	if(CHECK_BIT(CONFIG1_POS)){ //Use Blocking I2C Code
		i2c_read_bytes(MMA_ADDR, REG_XHI, data, 6);
	}
	
	if(CHECK_BIT(CONFIG2_POS)){ //Use FSM I2C Code
		while(!i2c_read_bytes_fsm(MMA_ADDR, REG_XHI, data, 6)){
			CLEAR_BIT(DEBUG1_POS)
			ShortDelay(8);
			SET_BIT(DEBUG1_POS)
		}
	}
	
	if(CHECK_BIT(CONFIG3_POS)){ //Use ISR I2C Code
		i2c_is_complete = 0;
		i2c_start();
		CLEAR_BIT(DEBUG1_POS)
		while(!i2c_is_complete){}
		for(i=0; i < 6; i++){
			data[i] = isr_data[i];
		}
	}
		
	for ( i=0; i<3; i++ ){
			temp[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
	}	
		


	// Align for 14 bits
	acc_X = temp[0]/4;
	acc_Y = temp[1]/4;
	acc_Z = temp[2]/4;
		
}
