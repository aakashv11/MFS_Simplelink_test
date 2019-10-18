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


/* EasyLink API Header files */
#include "components/easylink/EasyLink.h"

/***** Defines *****/

#define RFEASYLINKEX_TASK_STACK_SIZE 1024
#define RFEASYLINKEX_TASK_PRIORITY   2

/* Pin driver handles */
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;
static PIN_State ledPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_LED0 is on.
 *   - LEDs Board_LED1 is off.
 */
PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

/***** Variable declarations *****/
Task_Struct txTask;    /* not static so you can see in ROV */
static uint8_t txTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];

Semaphore_Struct btn_sem;
Semaphore_Handle btn_sem_handle;

/***** Function declarations *****/
void init_tasks(void);
void send_msg_task(void);
void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions. */
    Board_initGeneral();

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    /* Clear LED pins */
    PIN_setOutputValue(ledPinHandle, Board_LED1, 0);
    PIN_setOutputValue(ledPinHandle, Board_LED2, 0);

    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if(!buttonPinHandle) {
        System_abort("Error initializing button pins\n");
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
        System_abort("Error registering button callback function");
    }

    init_tasks();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

/***** Function definitions *****/
void init_tasks(void) {
	Task_Params txTaskParams;

	//init semaphores
	Semaphore_Params mutex_params;

	/* Construct a Semaphore object to be use as a resource lock, inital count 1 */
	Semaphore_Params_init(&mutex_params);
	Semaphore_construct(&btn_sem, 0, &mutex_params);

	/* Obtain instance handle */
	btn_sem_handle= Semaphore_handle(&btn_sem);


	Task_Params_init(&txTaskParams);
	txTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
	txTaskParams.priority = RFEASYLINKEX_TASK_PRIORITY;
	txTaskParams.stack = &txTaskStack;

	Task_construct(&txTask, (Task_FuncPtr)send_msg_task, &txTaskParams, NULL);
}

void send_msg_task() {

	char msg[] = {"Hello world!\r\n"}; // length = 14
	char len = 14;
	EasyLink_TxPacket txPacket =  { {0}, 0, 0, {0} };

	EasyLink_init(EasyLink_Phy_50kbps2gfsk);
	/* Set Freq to 915MHz */
	EasyLink_setFrequency(915000000);
	/* Set output power to 14dBm */
	EasyLink_setRfPwr(14);

	while (1) {
		// Wait for btn to be pressed
		Semaphore_pend(btn_sem_handle, BIOS_WAIT_FOREVER);

		memcpy(txPacket.payload, (void *)&(msg), (size_t)len);

		txPacket.len = (size_t)len;

		EasyLink_Status result = EasyLink_transmit(&txPacket);
	}
}

/*
 *  ======== buttonCallbackFxn ========
 *  Increment semaphore count.
 */
void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId) {
	  Semaphore_post(btn_sem_handle);
}

