/*
 * globals.c
 *
 *  Created on: Jun 20, 2016
 *      Author: Nicholas Metcalf
 */

#include "globals.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

Semaphore_Struct data_rdy_lock;
Semaphore_Handle data_rdy_lock_handle;

Semaphore_Struct IMU_rdy_lock;
Semaphore_Handle IMU_rdy_lock_handle;

Semaphore_Struct sampling_lock;
Semaphore_Handle sampling_lock_handle;

Semaphore_Struct gps_rdy_lock;
Semaphore_Handle gps_rdy_lock_handle;

char recalib_pressure = 1;

struct data_hdr_t msg_hdrs[NUM_MSGS];
struct bme_msg_t bme_msgs[NUM_MSGS];
struct bme_launcher_msg_t bme_launcher_msgs[NUM_MSGS];
struct imu_msg_t imu_msgs[NUM_MSGS];
struct gps_msg_t gps_msgs[NUM_MSGS];
struct gps_msg_full_t gps_full_msgs[NUM_MSGS];
//previous data arrays
struct data_hdr_t msg_hdrs_old[NUM_OLD_DATA] = {0};
struct bme_msg_t bme_msgs_old[NUM_OLD_DATA] = {0};
struct imu_msg_t imu_msgs_old[NUM_OLD_DATA] = {0};
struct gps_msg_full_t gps_full_msgs_old[NUM_OLD_DATA] = {0};


PIN_Handle pinHandle;

Task_Handle bme_task_handle = NULL;
Task_Handle bmi_task_handle = NULL;
Task_Handle gps_task_handle = NULL;
Task_Handle rf_task_handle = NULL;


char accel_sampled[NUM_MSGS] = {0,0};
char mag_sampled[NUM_MSGS] = {0,0};

unsigned long curr_time = 0;

void cc1310_usleep(uint32_t usecs, uint32_t ticks_offset)
{
//	static uint32_t sleep_time = 0;
	static uint32_t raw_sleep_time = 0;
	raw_sleep_time = (usecs / Clock_tickPeriod);
	// Make sure that if sleep_time will result in negative value, set to 0
//	sleep_time = raw_sleep_time < ticks_offset ? 0 : raw_sleep_time - ticks_offset;
	//Task_sleep(usecs / Clock_tickPeriod);
	//if (raw_sleep_time < sleep_time) {
		Task_sleep(raw_sleep_time);
	//}
	//else {
	//	Task_sleep(sleep_time);
	//}
}

// initialize mutexes
void init_locks(void)
{
    /* Construct BIOS objects */
    Semaphore_Params bme_params;
    Semaphore_Params gps_params;
    Semaphore_Params imu_params;
    Semaphore_Params sampling_params;

    /* Construct a Semaphore object to be use as a resource lock, inital count 1 */
    Semaphore_Params_init(&bme_params);
    bme_params.mode = Semaphore_Mode_COUNTING;
    Semaphore_Params_init(&gps_params);
    gps_params.mode = Semaphore_Mode_COUNTING;
    Semaphore_Params_init(&imu_params);
    imu_params.mode = Semaphore_Mode_COUNTING;
    Semaphore_Params_init(&sampling_params);
    sampling_params.mode = Semaphore_Mode_COUNTING;

    Semaphore_construct(&data_rdy_lock, 0, &bme_params);
    Semaphore_construct(&IMU_rdy_lock, 0, &imu_params);
    Semaphore_construct(&gps_rdy_lock, 0, &gps_params);
    Semaphore_construct(&sampling_lock, 3, &sampling_params);

    /* Obtain instance handle */
    data_rdy_lock_handle = Semaphore_handle(&data_rdy_lock);
    IMU_rdy_lock_handle = Semaphore_handle(&IMU_rdy_lock);
    gps_rdy_lock_handle = Semaphore_handle(&gps_rdy_lock);
    sampling_lock_handle = Semaphore_handle(&sampling_lock);
}


void itoa(char *out, char start, char end, int num)
{
	int new_num = num;
	while (start != end) {
		out[end] = '0' + abs(new_num%10);
		new_num /= 10;
		--end;
	}
	if (num < 0) {
		out[start] = '-';
	}
	else {
		out[start] = ' ';
	}
}

void uitoa(char *out, char start, char end, unsigned int num)
{
	int new_num = num;
	while (start <= end) {
		out[end] = '0' + new_num%10;
		new_num /= 10;
		--end;
	}
}
