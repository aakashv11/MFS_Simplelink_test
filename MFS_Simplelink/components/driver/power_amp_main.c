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
#include "components/global/Power_amp_Board.h"
#include "components/global/globals.h"
#include "components/tasks/rf_task.h"


/* EasyLink API Header files */
#include "components/easylink/EasyLink.h"

/***** Defines *****/

/* Undefine to remove address filter and async mode */
#define RFEASYLINKRX_ADDR_FILTER

#define RFEASYLINKEX_TASK_STACK_SIZE 1024
#define RFEASYLINKEX_TASK_PRIORITY   2

#define RECEIVER_SLEEP_AMOUNT        1000000
#define RX_TIMEOUT                   100000
/* Pin driver handle */
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] = {
  Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_HGM | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_LNA | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_PA | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

/***** Variable declarations *****/
Task_Struct txTask;    /* not static so you can see in ROV */
static uint8_t txTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];

/***** Function declarations *****/
static void init_tasks(void);
static void send_msg_task(void);
static void pa_et_rx(void);
static void pa_set_tx(void);
static void pa_set_high_gain(char on);

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions. */
    Board_initGeneral();

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    /* Clear LED pins */
    PIN_setOutputValue(ledPinHandle, Board_LED1, 1);
    PIN_setOutputValue(ledPinHandle, Board_LED2, 1);
    // Initialize power amplifier to 0 state
    PIN_setOutputValue(ledPinHandle, Board_HGM, 0);
    PIN_setOutputValue(ledPinHandle, Board_LNA, 0);
    PIN_setOutputValue(ledPinHandle, Board_PA, 0);

    init_tasks();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

/***** Function definitions *****/
static void init_tasks(void) {
	Task_Params txTaskParams;

	Task_Params_init(&txTaskParams);
	txTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
	txTaskParams.priority = RFEASYLINKEX_TASK_PRIORITY;
	txTaskParams.stack = &txTaskStack;

	Task_construct(&txTask, (Task_FuncPtr)send_msg_task, &txTaskParams, NULL);
}

static void send_msg_task(void) {

	struct data_hdr_t msg_hdr;
	unsigned short next_seqn = 0;
	EasyLink_Status result = EasyLink_Status_Success;
	EasyLink_TxPacket txPacket =  { {0}, 0, 0, {0} };

	msg_hdr.data_hdr_vers = DRONE_VERS;
	msg_hdr.data_hdr_type = MSG_TYPE_UNDEF;
	msg_hdr.data_hdr_seqn = SEQN_UNINIT;
	msg_hdr.address = RECEIVER1_ADDR;

	EasyLink_init(EasyLink_Phy_50kbps2gfsk);
	/* Set Freq to 915MHz */
	EasyLink_setFrequency(915000000);
	/* Set output power to 14dBm */
	// 14 is highest possible value, but can be set to others
	EasyLink_setRfPwr(14);

	// Set mode of power amplifier to tx
	pa_set_tx();
	// If desired set high gain mode
	// pa_set_high_gain(1);

	while (1) {
		msg_hdr.data_hdr_seqn = next_seqn++;
        msg_hdr.data_hdr_type = MSG_TYPE_RELEASE;

        	// copy header into txpacket payload
	    memcpy(txPacket.payload, &(msg_hdr), sizeof(struct data_hdr_t));

	    txPacket.dstAddr[0] = LAUNCHER_ADDR; // Address of receiver
	    txPacket.len = sizeof(struct data_hdr_t);

	    // Transmit message
	    result = EasyLink_transmit(&txPacket);


	    PIN_setOutputValue(ledPinHandle, Board_LED1, !PIN_getOutputValue(Board_LED1));
	    PIN_setOutputValue(ledPinHandle, Board_LED2, !PIN_getOutputValue(Board_LED2));

	    // Sleep for 1 sec
	    cc1310_usleep(1000000, 0);
	}
}

/*
 Function sets the power amplifier to be in receiver mode
*/
static void pa_set_rx(void) {
	PIN_setOutputValue(ledPinHandle, Board_LNA, 1);
	PIN_setOutputValue(ledPinHandle, Board_PA, 0);
}

/*
 Function sets the power amplifier to be in transmit mode
*/
static void pa_set_tx(void) {
	PIN_setOutputValue(ledPinHandle, Board_LNA, 0);
	PIN_setOutputValue(ledPinHandle, Board_PA, 1);
}

/*
 Function sets the power amplifier to be in high gain mode if on is non-zero
 and low gain mode if on is zero.
*/
static void pa_set_high_gain(char on) {
	on = on ? 1 : 0;
	PIN_setOutputValue(ledPinHandle, Board_HGM, on);
}
