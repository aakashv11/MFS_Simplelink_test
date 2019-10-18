/*
 * CC1310_I2C.h
 *
 *  Created on: Mar 21, 2016
 *      Author: Nicholas Metcalf
 */

#ifndef CC1310_I2C_H_
#define CC1310_I2C_H_


#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
// Initialize I2C module on CC1310
char CC1310_I2C_init();

// Pass an I2C_Transaction * which has already been filled with data
// This is transmitted and then passed back to caller
unsigned char CC1310_I2C_call(I2C_Transaction *i2c_txn);

#endif /* CC1310_I2C_H_ */
