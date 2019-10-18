/*
 * globals.h
 *
 *  Created on: Jun 20, 2016
 *      Author: Nicholas Metcalf
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>

#define BME_SAMPLE_SIZE			 	10  // Number of BME samples per message
#define BME_LAUNCHER_SAMPLE_SIZE	1	// Number of BME samples per message
#define IMU_SAMPLE_SIZE			 	25  // Number of IMU samples per message
#define GPS_MAX_DATA_LEN		 	100	// Number of GPS samples per message
#define NUM_MSGS			 	 	2	// Number of messages to alternate through
#define NUM_OLD_DATA                2   // Number of previous data sets to repeat along with current data set

#define PRIORITY_OFF				-1
#define GPS_PRIORITY				2
#define BME_PRIORITY				3
#define BMI_PRIORITY				-1
#define RF_PRIORITY					3 //2

#define BME_TASK                    1
#define RF_TASK                     1
#define BMI_TASK                    0
#define GPS_TASK                    1

#define BASE_FQ         915000000


// Global semaphores for synchronization
extern Semaphore_Handle data_rdy_lock_handle;
extern Semaphore_Handle IMU_rdy_lock_handle;
extern Semaphore_Handle sampling_lock_handle;
extern Semaphore_Handle gps_rdy_lock_handle;

extern char recalib_pressure;
extern PIN_Handle pinHandle;

// Global task handles for atomicity
extern Task_Handle bme_task_handle;
// Use seperate accelerometer, gyro, magnetometer in place of IMU
extern Task_Handle adxl_task_handle;
extern Task_Handle gyro_task_handle;
extern Task_Handle mdxl_task_handle;
extern Task_Handle gps_task_handle;
extern Task_Handle rf_task_handle;

extern char accel_sampled[NUM_MSGS];
extern char gyro_sampled[NUM_MSGS];
extern char mag_sampled[NUM_MSGS];

extern unsigned long curr_time;


//struct gps_msg_t;

// header for sensor messages
struct data_hdr_t {
	uint8_t data_hdr_vers;				// version number... Must be correct
	uint8_t data_hdr_type;				// Type of message dictates length of data
	uint16_t data_hdr_seqn;	            // seqn number for message
	uint16_t address;			        // id number of sender
	uint32_t data_crc;			        // CRC for payload
};

// data format for bme messages (array of values)
struct bme_msg_t
{
	int32_t temp_data[BME_SAMPLE_SIZE];		// temperature data array
	uint32_t press_data[BME_SAMPLE_SIZE];	// pressure data array
	uint32_t humid_data[BME_SAMPLE_SIZE];	// humidity data array
};

// data format for bme messages (single values)
struct bme_launcher_msg_t
{
	int32_t temp_data;		// temperature data array
	uint32_t press_data;	// pressure data array
	uint32_t humid_data;	// humidity data array
	uint32_t ground_press;
};

// magnetometer data struct
struct mag_t
{
    //6 bytes
	int16_t x;
	int16_t y;
	int16_t z;
};

// data format for imu messages (Update 9/20/2019 - no longer using single imu, struct is out of date)
/*
struct imu_msg_t
{
	struct bmi160_gyro_t gyro_data[IMU_SAMPLE_SIZE];	// gyroscope data array
	struct bmi160_accel_t accel_data[IMU_SAMPLE_SIZE];	// accelerometer data array
	struct bmi160_mag_t mag_data[IMU_SAMPLE_SIZE];		// magnetometer data array
};
*/


// gps message format
struct gps_msg_t
{
    //18 bytes
	double lat_; // 8 bytes
	double long_; // 8 bytes
	char north; // 1 byte
	char east; // 1 byte
};


struct gps_msg_full_t
{
    //35 bytes
    char time[9];
    char latitude[8];
    char N_S[1];
    char longitude[9];
    char E_W[1];
    char quality_indicator;
    char sat_num[2];
    char horizontal_resolution[2];
    char altitude[4];
};

// Instantiate each data message array
// NUM_MSGS allows for sampling using one message and transmitting another
extern struct data_hdr_t msg_hdrs[NUM_MSGS];
extern struct bme_msg_t bme_msgs[NUM_MSGS];
extern struct imu_msg_t imu_msgs[NUM_MSGS];
extern struct gps_msg_t gps_msgs[NUM_MSGS];
extern struct gps_msg_full_t gps_full_msgs[NUM_MSGS];
//previous data arrays
extern struct data_hdr_t msg_hdrs_old[NUM_OLD_DATA];
extern struct bme_msg_t bme_msgs_old[NUM_OLD_DATA];
extern struct imu_msg_t imu_msgs_old[NUM_OLD_DATA];
extern struct gps_msg_full_t gps_full_msgs_old[NUM_OLD_DATA];


// sleep for usec micro-seconds
extern void cc1310_usleep(uint32_t usecs, uint32_t ticks_offset);

// initialize mutexes
extern void init_locks(void);

// IMU CRC functions
unsigned int gen_IMU_CRC(struct imu_msg_t *data);
char check_IMU_CRC(struct imu_msg_t *data, unsigned int CRC);

//	Converts integer into an array of ascii chars 
void itoa(char *out, char start, char end, int num);
//	Converts an unsigned integer into an array of ascii chars 	
void uitoa(char *out, char start, char end, unsigned int num);

#endif /* GLOBALS_H_ */
