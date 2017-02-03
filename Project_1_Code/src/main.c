/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "mma8451.h"
#include "delay.h"

#define FLASH_DELAY 10
#define ACC_SENSITIVITY 100

void Init_Debug_Signals(void) {
	//Set debug pins to GPIO
	
	//Init Debug Signal 1: Set high when I2C code is running
	//Debug 1 board location: J10 pin 4
	PORTB->PCR[DEBUG1_POS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[DEBUG1_POS] |= PORT_PCR_MUX(1);
	//Init Debug Signal 2: Set high when I2C message is being sent or received on the bus (between start and stop conditions)
	//Debug 2 board location: J10 pin 6
	PORTB->PCR[DEBUG2_POS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[DEBUG2_POS] |= PORT_PCR_MUX(1);
	//Init Debug Signal 3: Set high when executing a busy-wait loop in I2C communications
	//Debug 3 board location: J10 pin 8
	PORTB->PCR[DEBUG3_POS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[DEBUG3_POS] |= PORT_PCR_MUX(1);
	
	
	//Set debug bits as outputs
	PTB->PDDR |= MASK(DEBUG1_POS) | MASK(DEBUG2_POS) | MASK(DEBUG3_POS);
	
}

void Init_Config_Signals(void) {
	//Enable port E clock
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	//Set config pins to GPIO
	
	//Config Signal 1 (Port E bit 3): When low use ISR I2C communications
	//Config Signal 1 board location: J9 pin 11
	PORTE->PCR[CONFIG1_POS] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	//config Signal 2 (Port E bit 4): When low use FSM polling I2C communications
	//Config signal 2 board location: J9 pin 13
	PORTE->PCR[CONFIG2_POS] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[CONFIG2_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;;
	//config Signal 3 (Port E bit 5): When low use default blocking I2C communications
	//Debug 3 board location: J9 pin 15
	PORTE->PCR[CONFIG3_POS] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[CONFIG3_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;;
	
	//Set configure signals as input
	PTE->PDDR &= ~MASK(CONFIG1_POS) | ~MASK(CONFIG2_POS) | ~MASK(CONFIG3_POS);
	
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	int16_t prev_acc_X, prev_acc_Y, prev_acc_Z;
	int n;
	
	Init_RGB_LEDs();
	Init_Debug_Signals();
	Init_Config_Signals();
	
	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	i2c_init();																/* init i2c	*/
	Delay(200);
	
	if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
		while (1)																/* not able to initialize mma */
			;
	}
	Control_RGB_LEDs(0, 0, 0);	

	if(CHECK_BIT(CONFIG3_POS)){
		init_i2c_isr();
	}	

	Delay(50);
	
	while (1) {
		Delay(1);
		prev_acc_X = acc_X;
		prev_acc_Y = acc_Y;
		prev_acc_Z = acc_Z;
		
		read_full_xyz();
		
		if ((abs(prev_acc_X - acc_X) > ACC_SENSITIVITY) || 
			(abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY) || 
			(abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY)) {
			// Flash LEDs
				for (n=0; n<2; n++) {
					Control_RGB_LEDs(1, 1, 1);
					Delay(FLASH_DELAY);
					Control_RGB_LEDs(0, 0, 0);							
					Delay(FLASH_DELAY*2);		
				}
			}		
	}
}

