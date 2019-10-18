/*
 * bme_task.c
 *
 *  Created on: Mar 21, 2016
 *      Author: Nicholas Metcalf
 */

#include "bme_task.h"
#include "components/bme/bme280.h"
#include "components/global/CC1310_I2C.h"
#include "components/global/globals.h"
#include "components/global/DRONE_Board.h"
//#include "LP_Board.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <xdc/runtime/System.h>
#define BME_SAMPLE_DELAY				100000		// 10 Hz
#define BME_LAUNCHER_SAMPLE_DELAY		1000000		// 1 Hz


I2C_Transaction i2c_txn;
volatile unsigned char data_to_send[9];
struct bme280_t sensor;
unsigned long int usecs;
int i;

u32 ticks_start;
u32 ticks_end;
u32 ticks_diff = 0;

static void enter_critical_section(void);
static void exit_critical_section(void);

static s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static u8 bme280_I2C_routine(void);
static void BME280_delay_msec(u16 msec);

Swi_Struct swi0Struct;
Swi_Handle swi0Handle;
/*
void swi0Fxn(UArg count, UArg idx)
{


    bme280_read_pressure_temperature_humidity((u32 *)&(bme_msgs[idx].press_data[count]),
                                              (s32 *)&(bme_msgs[idx].temp_data[count]),
                                              (u32 *)&(bme_msgs[idx].humid_data[count]));

}
*/
void bme280_task_init(void) {
	bme280_I2C_routine();
	bme280_init(&sensor);								// Init bme sensor
	bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);
	bme280_set_oversamp_pressure(BME280_OVERSAMP_1X);
	bme280_set_oversamp_temperature(BME280_OVERSAMP_1X);
	bme280_set_power_mode(BME280_SLEEP_MODE);
}

void bme280_task_run(void)
{
  /*
    Swi_Params swiParams;

    Swi_Params_init(&swiParams);
    swiParams.arg0 = 0;
    swiParams.arg1 = 0;
    swiParams.priority = 1;
  //  swiParams.trigger = 0;

    swi0Handle = Swi_create((Swi_FuncPtr)swi0Fxn, &swiParams, NULL);

    if (swi0Handle == NULL) {
     System_abort("Swi create failed");

    }
    */
//	enter_critical_section();
	bme280_task_init();
//	exit_critical_section();

	int count = 0;
	unsigned char idx = 1;

	while (1)
	{
	    idx ^= 1; // Alternate idx between 0 and 1
		while (count < BME_SAMPLE_SIZE)
		{
			ticks_start = Clock_getTicks();		// Get start time of measurement

			// Lock to ensure sensor measurement occurs atomically

			PIN_setOutputValue(pinHandle, Board_TESTPOINT_2, 1);
/*

		//	Swi_Params swiParams2;
			Swi_getAttrs(swi0Handle, NULL, &swiParams);
			swiParams.arg0 = count;
			swiParams.arg1 = idx;
		//	swiParams.priority = 1;
			Swi_setAttrs(swi0Handle, NULL, &swiParams);

*/
			bme280_set_power_mode(BME280_FORCED_MODE);
			cc1310_usleep(BME_WAKE_DELAY, 0); // sleep for 5 ms..can i do this?
		//	Swi_post(swi0Handle); // get data
			enter_critical_section();
			bme280_read_pressure_temperature_humidity((u32 *)&(bme_msgs[idx].press_data[count]),
			                                              (s32 *)&(bme_msgs[idx].temp_data[count]),
			                                              (u32 *)&(bme_msgs[idx].humid_data[count]));
			exit_critical_section();
			bme280_set_power_mode(BME280_SLEEP_MODE);
			 ++count;
			ticks_end = Clock_getTicks();	// Get end time of measurement

			// get difference in ticks. If wrapped around, use prevoius difference
			ticks_diff = (ticks_end < ticks_start) ? ticks_diff : (ticks_end - ticks_start);
			PIN_setOutputValue(pinHandle, Board_TESTPOINT_2, 0);
			cc1310_usleep(BME_SAMPLE_DELAY, ticks_diff); // Should be replaced by hardware interrupts

		}
		// Let RF task know that data is ready
		Semaphore_post(data_rdy_lock_handle);
		Task_yield();
		//PIN_setOutputValue(pinHandle, Board_LED0, !PIN_getOutputValue(Board_LED0));

		count = 0;
	}
}

// I2c write wrapper function
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	data_to_send[0] = reg_addr;				// Set first block to register address
	for (i = 1; i < cnt+1; ++i) {
		data_to_send[i] = reg_data[i-1];	// Fill in register data
	}

	i2c_txn.slaveAddress = dev_addr; /* 7-bit peripheral slave address */
	i2c_txn.writeBuf = (void *)data_to_send; /* Buffer to be written */
	i2c_txn.writeCount = cnt + 1; /* Number of bytes to be written */
	i2c_txn.readBuf = NULL; /* Buffer to be read */
	i2c_txn.readCount = 0; /* Number of bytes to be read */

	return CC1310_I2C_call(&i2c_txn);
}

// I2c read wrapper function
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	data_to_send[0] = reg_addr;		// Set first block to register address

	i2c_txn.slaveAddress = dev_addr; /* 7-bit peripheral slave address */
	i2c_txn.writeBuf = (void *)data_to_send; /* Buffer to be written */
	i2c_txn.writeCount = 1; /* Number of bytes to be written */
	i2c_txn.readBuf = (void *)reg_data; /* Buffer to be read */
	i2c_txn.readCount = cnt; /* Number of bytes to be read */
	return CC1310_I2C_call(&i2c_txn);
}

u8 bme280_I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp180 the following structure parameter can be accessed
 *	Bus write function pointer: BMP180_WR_FUNC_PTR
 *	Bus read function pointer: BMP180_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	sensor.bus_write = BME280_I2C_bus_write;
	sensor.bus_read = BME280_I2C_bus_read;
	sensor.dev_addr = BME280_I2C_ADDRESS1; // Use with embedded adventure boards
//	sensor.dev_addr = BME280_I2C_ADDRESS2; // Use with adafruit boards
	sensor.delay_msec = BME280_delay_msec;

	return BME280_INIT_VALUE;
}

void BME280_delay_msec(u16 msec) {
	usecs = msec*1000;
	cc1310_usleep(usecs, 0);
}

static void enter_critical_section(void)
{
#if RF_TASK
	Task_setPri(rf_task_handle, PRIORITY_OFF);
#endif
#if BMI_TASK
	Task_setPri(bmi_task_handle, PRIORITY_OFF);
#endif
#if GPS_TASK
	Task_setPri(gps_task_handle, PRIORITY_OFF);
#endif
}

static void exit_critical_section(void)
{
#if RF_TASK
	Task_setPri(rf_task_handle, RF_PRIORITY);
#endif

#if GPS_TASK
	Task_setPri(gps_task_handle, GPS_PRIORITY);
#endif
}
