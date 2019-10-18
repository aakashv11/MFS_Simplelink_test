
#ifndef RF_TASK_H_
#define RF_TASK_H_

#include "components/easylink/EasyLink.h"

#define RF_TASK_STACK_SIZE                1024

#define MSG_TYPE_UNDEF					  255
#define MSG_TYPE_BME					    0
#define MSG_TYPE_IMU					    1
#define MSG_TYPE_GPS					    2
#define MSG_TYPE_ALL 				        9
#define MSG_TYPE_RELEASE	 	 		    3	// Release drone
#define MSG_TYPE_RECALIB	 	 		    4	// Recalibrate base pressure
#define MSG_TYPE_LAUNCHER_DATA              5 // Recalibrate base pressure
#define MSG_TYPE_BME_REQUEST                6 // Recalibrate base pressure
#define MSG_TYPE_GPS_FULL                   7

#define DRONE_VERS						    0xAE
#define SEQN_UNINIT						    0x00
#define CURRENT_ADDR                        0x12
#define LAUNCHER_ADDR                       0xBB
#define RECEIVER1_ADDR                      0xAA
#define RECEIVER2_ADDR                      0xCC

// Initialize all parameters for rf task
EasyLink_Status rf_task_init(void);

// Main rf loop to transmit to LCM
void rf_task_run(void);

#endif /* RF_TASK_H_ */
