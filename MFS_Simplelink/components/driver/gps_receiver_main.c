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
#include "components/launcher/LP_Board.h"
#include "components/global/globals.h"
#include "components/tasks/rf_task.h"
#include "components/global/CC1310_UART.h"


/* EasyLink API Header files */
#include "components/easylink/EasyLink.h"

/***** Defines *****/

/* Undefine to remove address filter and async mode */
#define RFEASYLINKRX_ADDR_FILTER

#define RFEASYLINKEX_TASK_STACK_SIZE 1024
#define PROCESS_TASK_STACK_SIZE 512
#define RFEASYLINKEX_TASK_PRIORITY   2

#define NUM_HOPS 		4
#define BUFF_SIZE		4

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

struct data_hdr_t msg_hdrs_recv[BUFF_SIZE];
struct gps_msg_full_t gps_msgs_recv[BUFF_SIZE];

Semaphore_Struct process_lock;
Semaphore_Handle process_lock_handle;

char address_str[] = {"Address: xxx\r\n"};	// len = 14

/***** Function declarations *****/
static void rfEasyLinkRxFnx(UArg arg0, UArg arg1);
void rxTask_init(PIN_Handle ledPinHandle);
void process_message();
void get_commands_task();
void itoa(char *out, char start, char end, int num);
void uitoa(char *out, char start, char end, unsigned int num);
unsigned int findchr(const char *input, char to_find);

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

    //    Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
    Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
    Task_construct(&process_Task, process_message, &process_TaskParams, NULL);
}

static void rfEasyLinkRxFnx(UArg arg0, UArg arg1)
{
	int i;
	char curr_idx = 0;
	EasyLink_RxPacket rxPacket = {0};
    EasyLink_init(EasyLink_Phy_50kbps2gfsk);
    /* Set Freq to 915MHz */
    EasyLink_setFrequency(915000000);
    /* Set output power to 14dBm */
    EasyLink_setRfPwr(14);

    uint8_t addrFilter = 0xaa;
//    EasyLink_enableRxAddrFilter(&addrFilter, 1, 1);

    while(1) {
    	rxPacket.absTime = 0;
    	EasyLink_Status result = EasyLink_receive(&rxPacket);

    	if (result == EasyLink_Status_Success)
    	{
        	/* On success, we copy data from received buffer into data structures */
    		memcpy((void *)&(msg_hdrs_recv[curr_idx]), rxPacket.payload, sizeof(struct data_hdr_t));
    		memcpy((void *)&(gps_msgs_recv[curr_idx]), rxPacket.payload + sizeof(struct data_hdr_t), sizeof(struct gps_msg_full_t));

           	/* Toggle LED2 to indicate RX */
           	PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));
        	PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));

        	Semaphore_post(process_lock_handle);
        	curr_idx = (curr_idx + 1)%BUFF_SIZE;
    	}
       	else
       	{
       		/* Toggle LED1 and LED2 to indicate error */
          	PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
           	PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));
        }
    }
}

void process_message() {
	int i = 0;
	char curr_idx = 0;
	unsigned int len = 0;
	while (1) {
		Semaphore_pend(process_lock_handle, BIOS_WAIT_FOREVER);

		uitoa(address_str, 9, 11, msg_hdrs_recv[curr_idx].address);
		UART_printf(address_str, 14);
		len = findchr((char *)gps_msgs_recv[curr_idx].longitude, '\n');
		UART_printf((char *)gps_msgs_recv[curr_idx].longitude, len);

		curr_idx = (curr_idx + 1)%BUFF_SIZE;
	}
}

void itoa(char *out, char start, char end, int num) {
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

void uitoa(char *out, char start, char end, unsigned int num) {
	int new_num = num;
	while (start <= end) {
		out[end] = '0' + new_num%10;
		new_num /= 10;
		--end;
	}
}

unsigned int findchr(const char *input, char to_find) {
	unsigned int pos = 0;
	while (input[pos++] != to_find);
	return pos;
}
