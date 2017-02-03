#include	 <MKL25Z4.H>
#include	 "i2c.h"
#include 	"gpio_defs.h"
#include  "mma8451.h"

extern volatile int i2c_is_complete;
extern volatile uint8_t isr_data[6];
int lock_detect = 0;
int i2c_lock = 0;

//init i2c0
void i2c_init( void )
{
 //clock i2c peripheral and port E
	SIM->SCGC4		 |= SIM_SCGC4_I2C0_MASK;
	SIM->SCGC5		 |= SIM_SCGC5_PORTE_MASK;

	//set pins to I2C function
	PORTE->PCR[ 24 ] |= PORT_PCR_MUX( 5 );
	PORTE->PCR[ 25 ] |= PORT_PCR_MUX( 5 );

	//set baud rate
	//baud = bus freq/(scl_div+mul)
	I2C0->F				= ( I2C_F_ICR( 0x11 ) | I2C_F_MULT( 0 ) );

	//enable i2c and set to master mode
	I2C0->C1		 |= ( I2C_C1_IICEN_MASK );

	// Select high drive mode
	I2C0->C2		 |= ( I2C_C2_HDRS_MASK );
}


void i2c_wait(void) {
	SET_BIT(DEBUG3_POS)
	while(((I2C0->S & I2C_S_IICIF_MASK)==0)) {
		;
		}
  I2C0->S |= I2C_S_IICIF_MASK;
	CLEAR_BIT(DEBUG3_POS)
}

int i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	uint8_t dummy, num_bytes_read=0, is_last_read=0;
	
	SET_BIT(DEBUG1_POS)

	I2C_TRAN;													//	set to transmit mode
	SET_BIT(DEBUG2_POS)
	I2C_M_START;											//	send start
	I2C0->D = dev_adx;								//	send dev address (write)
	i2c_wait();												//	wait for completion

	I2C0->D = reg_adx;								//	send register address
	i2c_wait();												//	wait for completion

	SET_BIT(DEBUG2_POS)
	I2C_M_RSTART;											//	repeated start
	I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)
	i2c_wait();												//	wait for completion

	I2C_REC;													//	set to receive mode
	while (num_bytes_read < data_count) {
		is_last_read = (num_bytes_read == data_count-1)? 1: 0;
		if (is_last_read){
			NACK;													// tell HW to send NACK after read
		} else {
			ACK;													// tell HW to send ACK after read
		}

		dummy = I2C0->D;								//	dummy read
		i2c_wait();											//	wait for completion

		if (is_last_read){
			I2C_M_STOP;										//	send stop
			CLEAR_BIT(DEBUG2_POS);
		}
		data[num_bytes_read++] = I2C0->D; //	read data
	}
	CLEAR_BIT(DEBUG1_POS)
	return 1;
}

int i2c_read_bytes_fsm(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	
	uint8_t dummy;
	static uint8_t num_bytes_read = 0, is_last_read = 0;
	static enum {ST_DEV_WR, ST_WAIT_DEV, ST_SEND_REG, ST_WAIT_REG, ST_DEV_RD, ST_DEV_RD_WAIT, ST_DUMMY_READ, ST_WAIT_BYTES, ST_READ_BYTES, ST_IDLE} next_state = ST_DEV_WR;
	
	switch (next_state){
		case ST_DEV_WR:
			I2C_TRAN;													//	set to transmit mode
			SET_BIT(DEBUG2_POS)
			I2C_M_START;											//	send start
			I2C0->D = dev_adx;								//	send dev address (write)
			next_state = ST_WAIT_DEV;
			return 0;
		case ST_WAIT_DEV:
			if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0)){
				I2C0->S |= I2C_S_IICIF_MASK;
				next_state = ST_SEND_REG;
			}
			return 0;
		case ST_SEND_REG:
			I2C0->D = reg_adx;								//	send register address
			next_state = ST_WAIT_REG;
			return 0;
		case ST_WAIT_REG:
			if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0)){
				I2C0->S |= I2C_S_IICIF_MASK;
				next_state = ST_DEV_RD;
			}
			return 0;
		case ST_DEV_RD:
			SET_BIT(DEBUG2_POS)
			I2C_M_RSTART;											//	repeated start
			I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)
			next_state = ST_DEV_RD_WAIT;
			return 0;
		case ST_DEV_RD_WAIT:
			if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0)){
				I2C0->S |= I2C_S_IICIF_MASK;
				I2C_REC;													//	set to receive mode
				next_state = ST_DUMMY_READ;
			}
			return 0;
		case ST_DUMMY_READ:
			is_last_read = (num_bytes_read == data_count-1)? 1: 0;
			if (is_last_read){
				NACK;													// tell HW to send NACK after read
			} else {
				ACK;													// tell HW to send ACK after read
			}
			dummy = I2C0->D;								//	dummy read
			next_state = ST_WAIT_BYTES;
			return 0;
		case ST_WAIT_BYTES:
			if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0)){
				I2C0->S |= I2C_S_IICIF_MASK;
				next_state = ST_READ_BYTES;
			}
			return 0;
		case ST_READ_BYTES:
			if (is_last_read){
				I2C_M_STOP;										//	send stop
				CLEAR_BIT(DEBUG2_POS)
				next_state = ST_DEV_WR;
				data[num_bytes_read++] = I2C0->D; //	read data
				num_bytes_read = 0;
				is_last_read = 0;
				return 1;
			}
			else{
				data[num_bytes_read++] = I2C0->D; //	read data
				next_state = ST_DUMMY_READ;
				return 0;
			}
			default:
				next_state = ST_DEV_WR;
				return 0;
	}
	
}

void init_i2c_isr(){
	SET_BIT(DEBUG1_POS)
	//enable i2c interrupt
	I2C0->C1 |= I2C_C1_IICIE_MASK;
	NVIC_SetPriority(I2C0_IRQn, 2);
	NVIC_ClearPendingIRQ(I2C0_IRQn);
	NVIC_EnableIRQ(I2C0_IRQn);
	__enable_irq();
	CLEAR_BIT(DEBUG1_POS)
	
}

void i2c_start(){
		SET_BIT(DEBUG1_POS)
		I2C_TRAN;													//	set to transmit mode
		SET_BIT(DEBUG2_POS)
		I2C_M_START;											//	send start
		I2C0->D = MMA_ADDR;								//	send dev address (write)
		CLEAR_BIT(DEBUG1_POS)
}

void I2C0_IRQHandler(void){
	
	uint8_t dummy;
	static enum {SEND_REG, SEND_DEV, END} address_cycle = SEND_REG;
	static int byte_count = 0;
	
	SET_BIT(DEBUG1_POS)
	
	I2C0->S |= I2C_S_IICIF_MASK;	// Clear interrupt flag
	if(I2C0->C1 & I2C_C1_TX_MASK){ //if transmit mode
		if((I2C0->S & I2C_S_RXAK_MASK) == 0){
			switch (address_cycle){
				case SEND_REG: //Send Device Register
					I2C0->D = REG_XHI;
					address_cycle = SEND_DEV;
					break;
				case SEND_DEV:
					I2C_M_RSTART; //Send repeated start
					I2C0->D = MMA_ADDR | 0x01;
					address_cycle = END;
					break;
				case END:
					I2C_REC;	//	set to receive mode
					dummy = I2C0->D;
					ACK; //Needed for some reason
					address_cycle = SEND_REG;
					break;
				default:
					address_cycle = SEND_REG;
					break;
			}
		}
	}
	else{ //else receive mode
	
		if(byte_count == 5){ //last byte to be received
			CLEAR_BIT(DEBUG2_POS)
			I2C_M_STOP; //Send stop
			i2c_is_complete = 1; //Set completion flag
		}
		else if(byte_count == 4){
			NACK; 
		}
		else{
			ACK;
		}
		
		isr_data[byte_count++] = I2C0->D; //Record accelerometer data
		if(byte_count > 5){
			byte_count = 0; //Reset byte count
		}
	}
	
	CLEAR_BIT(DEBUG1_POS)
	
}

int i2c_write_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	uint8_t num_bytes_written=0;

	I2C_TRAN;													//	set to transmit mode
	I2C_M_START;											//	send start
	I2C0->D = dev_adx;								//	send dev address (write)
	i2c_wait();												//	wait for completion

	I2C0->D = reg_adx;								//	send register address
	i2c_wait();												//	wait for completion

	while (num_bytes_written < data_count) {
		I2C0->D = data[num_bytes_written++]; //	write data
		i2c_wait();											//	wait for completion
	}
	I2C_M_STOP;												//		send stop
	CLEAR_BIT(DEBUG2_POS)
	return 1;
}
