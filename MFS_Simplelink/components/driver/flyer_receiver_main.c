/*
	Main file for flyer_receiver code that
	will receive data from the current version
	of a FLYER_V2R2.
	
  Author: Dennis Kanarsky
	Date: 2/26/2017
	
	(C) 2017 Sensor Network Laboratory. 
	University of Michigan. All Right Reserved.
*/

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "components/launcher/LP_Board.h" // CC1310 LaunchPad
#include "components/global/globals.h"
#include "components/tasks/rf_task.h"
#include "components/global/CC1310_UART.h"

#include <stdio.h>

/* EasyLink API Header files */
#include "components/easylink/EasyLink.h"

/***** Defines *****/

/* Undefine to remove address filter and async mode */
#define RFEASYLINKRX_ADDR_FILTER

#define RFEASYLINKEX_TASK_STACK_SIZE 1024
#define PROCESS_TASK_STACK_SIZE 512
#define RFEASYLINKEX_TASK_PRIORITY   2

#define BUFF_SIZE		10

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] = {
	Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//	Board_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//	Board_LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/***** Variable declarations *****/
static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */
static uint8_t rxTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];

static Task_Params process_TaskParams;
Task_Struct process_Task;    /* not static so you can see in ROV */
static uint8_t process_TaskStack[PROCESS_TASK_STACK_SIZE];

struct data_hdr_t msg_hdrs_recv;
struct bme_msg_t bme_msgs_recv;
struct gps_msg_t gps_msgs_recv;

Semaphore_Struct process_lock;
Semaphore_Handle process_lock_handle;

char address_str[] = {"Address: xxx\r\n"};	// len = 14
char temp_str[] = {"T xxxx.xx C\r\n"};	// len = 13
char press_str[] = {"P XXXX.XX mbar\r\n"};	// len = 16
char humid_str[] = {"H xXX.XXX %\r\n"};	// len = 13
char gps_str[] = {"XXXX.XXXX,X,XXXXX.XXXX,X\r\n"};	// len = 26
char Ax_str[] = {"Ax xxxxxx\r\n"};	// len = 11
char Ay_str[] = {"Ay xxxxxx\r\n"};	// len = 11
char Az_str[] = {"Az xxxxxx\r\n"};	// len = 11
char Gx_str[] = {"Gx XXXXXX\r\n"};	// len = 11
char Gy_str[] = {"Gy XXXXXX\r\n"};	// len = 11
char Gz_str[] = {"Gz XXXXXX\r\n"};	// len = 11
char Mx_str[] = {"Mx XXXXXXXXXXX\r\n"};	// len = 16
char My_str[] = {"My XXXXXXXXXXX\r\n"};	// len = 16
char Mz_str[] = {"Mz XXXXXXXXXXX\r\n"};	// len = 16

/***** Function declarations *****/
static void rfEasyLinkRxFnx(UArg arg0, UArg arg1);
void rxTask_init(PIN_Handle ledPinHandle);
void process_messages_task();
void process_bme_message(void);
void process_gps_message(void);
void process_imu_message(void);

/*
 *  ======== main ========
 */
int main(void)
{
	/* Call board init functions. */
	Board_initGeneral();
	CC1310_UART_init();

	/* Open LED pins */
	ledPinHandle = PIN_open(&ledPinState, pinTable);
	if(!ledPinHandle) {
			System_abort("Error initializing board LED pins\n");
	}

	/* Clear LED pins */
	PIN_setOutputValue(ledPinHandle, Board_LED1, 0);
	PIN_setOutputValue(ledPinHandle, Board_LED2, 0);
//    PIN_setOutputValue(ledPinHandle, Board_LED3, 0);
//    PIN_setOutputValue(ledPinHandle, Board_LED4, 0);

	rxTask_init(ledPinHandle);

	/* Start BIOS */
	BIOS_start();

	return (0);
}

/***** Function definitions *****/
void rxTask_init(PIN_Handle ledPinHandle) {
	/* Construct BIOS objects */
	Semaphore_Params mutex_params;

	/* Construct a Semaphore object to be use as a resource lock, inital count 1 */
	Semaphore_Params_init(&mutex_params);
	Semaphore_construct(&process_lock, 0, &mutex_params);

	/* Obtain instance handle */
	process_lock_handle = Semaphore_handle(&process_lock);

	pinHandle = ledPinHandle;

	Task_Params_init(&rxTaskParams);
	rxTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
	rxTaskParams.priority = RFEASYLINKEX_TASK_PRIORITY;
	rxTaskParams.stack = &rxTaskStack;
	rxTaskParams.arg0 = (UInt)1000000;

	Task_Params_init(&process_TaskParams);
	process_TaskParams.stackSize = PROCESS_TASK_STACK_SIZE;
	process_TaskParams.priority = RFEASYLINKEX_TASK_PRIORITY;
	process_TaskParams.stack = &process_TaskStack;
	process_TaskParams.arg0 = (UInt)1000000;

	Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
	Task_construct(&process_Task, process_messages_task, &process_TaskParams, NULL);
}

static void rfEasyLinkRxFnx(UArg arg0, UArg arg1)
{
	int i;
	char curr_idx = 0;
	/* Initialize EasyLink */
	EasyLink_RxPacket rxPacket = {0};
  EasyLink_init(EasyLink_Phy_50kbps2gfsk);
	/* Set Freq to 915MHz */
	EasyLink_setFrequency(915000000);
	/* Set output power to 14dBm */
	EasyLink_setRfPwr(14);

  uint8_t addrFilter = 0xaa;
	EasyLink_enableRxAddrFilter(&addrFilter, 1, 1); //WHAT IS THIS?

	while(1) 
	{
		rxPacket.absTime = 0;
		EasyLink_Status result = EasyLink_receive(&rxPacket);
		if (result == EasyLink_Status_Success)
		{
			
			/* On success, we copy data from received buffer into data structures */
			/* First copy header data into data_hdr_t */
			memcpy(&(msg_hdrs_recv), rxPacket.payload, sizeof(struct data_hdr_t));
			
			if (msg_hdrs_recv.data_hdr_type == MSG_TYPE_BME)
			{
				memcpy(&(bme_msgs_recv), rxPacket.payload + sizeof(struct data_hdr_t), sizeof(struct bme_msg_t));

				process_message(); // This should be a task..
				/* Toggle LED2 to indicate RX */
				PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));

			}
			if (msg_hdrs_recv.data_hdr_type == MSG_TYPE_GPS)
			{
				memcpy(&(gps_msgs_recv), rxPacket.payload + sizeof(struct data_hdr_t), sizeof(struct gps_msg_t));

				process_gps_message(); // This should be a task...
				/* Toggle LED2 to indicate RX */
				PIN_setOutputValue(pinHandle, Board_LED0,!PIN_getOutputValue(Board_LED0));
			}
			else
			{
				/* Toggle LED1 and LED2 to indicate error */
				PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
				PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));
			}
		} //end of EasyLink status change
	} // end of while(1)
} // end of rfEasyLinkRxFnx

void process_messages_task() {
	int i = 0;
	while (1) 
	{
		Semaphore_pend(process_lock_handle, BIOS_WAIT_FOREVER); // What does this do??
		process_bme_message();
		process_gps_message();
		//TODO: Need some sort of buffer because otherwise we will loose messages.............. :-'(
	}
}


void process_bme_message(void) {
	int i = 0;

	address_str[11] = '0'+msg_hdrs_recv.address%10;
	address_str[10] = '0'+(msg_hdrs_recv.address/10)%10;
	address_str[9] = '0'+(msg_hdrs_recv.address/100)%10;

	UART_printf(address_str, 14);

	for (i = 0; i < BME_SAMPLE_SIZE; ++i) {
		temp_str[8] = '0'+bme_msgs_recv.temp_data[i]%10;
		temp_str[7] = '0'+(bme_msgs_recv.temp_data[i]/10)%10;
		temp_str[5] = '0'+(bme_msgs_recv.temp_data[i]/100)%10;
		temp_str[4] = '0'+(bme_msgs_recv.temp_data[i]/1000)%10;
		temp_str[3] = '0'+(bme_msgs_recv.temp_data[i]/10000)%10;
		if (bme_msgs_recv.temp_data[i] < 0) {
			temp_str[2] = '-';
		}
		else {
			temp_str[2] = ' ';
		}
		press_str[2] = '0'+(bme_msgs_recv.press_data[i]/100000)%10;
		press_str[3] = '0'+(bme_msgs_recv.press_data[i]/10000)%10;
		press_str[4] = '0'+(bme_msgs_recv.press_data[i]/1000)%10;
		press_str[5] = '0'+(bme_msgs_recv.press_data[i]/100)%10;
		press_str[7] = '0'+(bme_msgs_recv.press_data[i]/10)%10;
		press_str[8] = '0'+(bme_msgs_recv.press_data[i])%10;

		humid_str[8] = '0'+bme_msgs_recv.humid_data[i]%10;
		humid_str[7] = '0'+(bme_msgs_recv.humid_data[i]/10)%10;
		humid_str[6] = '0'+(bme_msgs_recv.humid_data[i]/100)%10;
		humid_str[4] = '0'+(bme_msgs_recv.humid_data[i]/1000)%10;
		humid_str[3] = '0'+(bme_msgs_recv.humid_data[i]/10000)%10;
		humid_str[2] = '0'+(bme_msgs_recv.humid_data[i]/100000)%10;

		UART_printf(temp_str, 13);
		UART_printf(press_str, 16);
		UART_printf(humid_str, 13);
	}
}

void process_gps_message(void) {
	int i = 0;

	address_str[11] = '0'+msg_hdrs_recv.address%10;
	address_str[10] = '0'+(msg_hdrs_recv.address/10)%10;
	address_str[9] = '0'+(msg_hdrs_recv.address/100)%10;

	UART_printf(address_str, 14);

	unsigned long int lat_ = gps_msgs_recv.lat_*10000;
	unsigned long int long_ = gps_msgs_recv.long_*10000;

	gps_str[8] = '0'+lat_%10;
	gps_str[7] = '0'+(lat_/10)%10;
	gps_str[6] = '0'+(lat_/10)%10;
	gps_str[5] = '0'+(lat_/10)%10;
	gps_str[3] = '0'+(lat_/100)%10;
	gps_str[2] = '0'+(lat_/1000)%10;
	gps_str[1] = '0'+(lat_/10000)%10;
	gps_str[0] = '0'+(lat_/100000)%10;

	gps_str[10] = gps_msgs_recv.north;

	gps_str[21] = '0'+long_%10;
	gps_str[20] = '0'+(long_/10)%10;
	gps_str[19] = '0'+(long_/10)%10;
	gps_str[18] = '0'+(long_/10)%10;
	gps_str[16] = '0'+(long_/100)%10;
	gps_str[15] = '0'+(long_/1000)%10;
	gps_str[14] = '0'+(long_/10000)%10;
	gps_str[13] = '0'+(long_/100000)%10;
	gps_str[12] = '0'+(long_/1000000)%10;

	gps_str[23] = gps_msgs_recv.east;

	UART_printf(gps_str, 26);
}