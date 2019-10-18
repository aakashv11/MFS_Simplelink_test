/*
 * CC1310_I2C.c
 *
 *  Created on: Mar 21, 2016
 *      Author: Nicholas Metcalf
 */

#include "CC1310_I2C.h"
#include "globals.h"
//#include "LP_Board.h"
#include "DRONE_Board.h"
#include <stdio.h>
#include <xdc/std.h>
/* I2C Globals */

I2C_Handle i2c_base;
I2C_Params i2c_params;

char CC1310_I2C_init() {
	// Open I2C periph in blocking mode with default bitrate set to 100KHz
	UInt peripheral_num = 0; /* Such as I2C0: CC1310 only has support for I2C0 */
	I2C_Params_init(&i2c_params);

	i2c_params.transferMode = I2C_MODE_BLOCKING;
	i2c_params.transferCallbackFxn = NULL;
	
	i2c_params.bitRate = I2C_100kHz;
	// i2c_params.bitRate = I2C_400kHz; // Use if 400KHz is needed or preferred

	i2c_base = I2C_open(peripheral_num, &i2c_params);
	if (i2c_base == NULL) {
	 /* Error opening I2C */
		printf("Error in opening I2C connection\n");
		return((char)(-1));
	}
	return(0);
}

unsigned char CC1310_I2C_call(I2C_Transaction *i2c_txn) {
	Bool transfer_stat = false;

	transfer_stat = I2C_transfer(i2c_base, i2c_txn); /* Perform I2C transfer */
	if (!transfer_stat) {
	 /* I2C bus fault */
	    PIN_setOutputValue(pinHandle, Board_LED0, !PIN_getOutputValue(Board_LED0));
		return((char)-1);
	}
	return(0);
}
