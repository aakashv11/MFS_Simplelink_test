/*
 * 	V0.1 Launcher
 * 	Mostly the same as drone code, but made to run on cc1310 launchpd.
 * 	Only measures pressure at 2 HZ.
 * 	Accecpts 2 commands: launch drone, and reset ground pressure
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
#include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>

#include "components/tasks/bme_task.h"
#include "components/tasks/rf_task.h"
#include "components/global/globals.h"
#include "components/global/CC1310_I2C.h"

/* Board Header files */
#include "components/launcher/LP_Board.h"

/* EasyLink API Header files */
#include "components/easylink/EasyLink.h"

/***** Defines *****/

#define LAUNCHER_TASK_PRIORITY   1
#define LAUNCHER_STACK_SIZE     1024

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
    PIN_TERMINATE
};

/***** Variable declarations *****/
Task_Struct launcher_task_struct;
char launcher_task_stack[LAUNCHER_STACK_SIZE];

static struct data_hdr_t msg_hdr_recv;

static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */
static uint8_t rxTaskStack[LAUNCHER_STACK_SIZE];

/* The RX Output struct contains statistics about the RX operation of the radio */
PIN_Handle pinHandle;

#ifdef RFEASYLINKRX_ASYNC
static Semaphore_Handle rxDoneSem;
#endif

/***** Function declarations *****/
void rxTask_init(void);
void rfEasyLinkRxFnx(UArg arg0, UArg arg1);

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions. */
    Board_initGeneral();
    Board_initGPIO();
    Board_initPWM();

    /* Open LED pins */
   // ledPinHandle = PIN_open(&ledPinState, pinTable);
   // if(!ledPinHandle) {
   //     System_abort("Error initializing board LED pins\n");
   // }

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);
    GPIO_write(Board_LED1, Board_LED_ON);

    rxTask_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

/***** Function definitions *****/
void rxTask_init(void) {

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = LAUNCHER_STACK_SIZE;
    rxTaskParams.priority = LAUNCHER_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;

    Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
}

void rfEasyLinkRxFnx(UArg arg0, UArg arg1) {
    PWM_Handle pwm1;
    PWM_Params params;
    uint16_t   pwmPeriod = 20000;      // Period and duty in microseconds
    uint16_t   duty = 1500;

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = duty;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;
    pwm1 = PWM_open(Board_PWM0, &params);

    if (pwm1 == NULL) {
        System_abort("Board_PWM0 did not open");
    }

	EasyLink_RxPacket rxPacket = {0};

    EasyLink_init(EasyLink_Phy_50kbps2gfsk);
    EasyLink_setFrequency(915000000);
    /* Set output power to 14dBm */
    EasyLink_setRfPwr(14);
    while(1) {
        rxPacket.absTime = 0;
        EasyLink_Status result = EasyLink_receive(&rxPacket);

        if (result == EasyLink_Status_Success)
        {
            /* Toggle LED2 to indicate RX */
            /* On success, we copy data from received buffer into data structures */
            memcpy((void *)&(msg_hdr_recv), rxPacket.payload, sizeof(struct data_hdr_t));

            switch (msg_hdr_recv.data_hdr_type) {
                case MSG_TYPE_RELEASE: {
                    PWM_start(pwm1);
                    //PIN_setOutputValue(ledPinHandle, Board_LED1, !PIN_getOutputValue(Board_LED1));
                    GPIO_write(Board_LED1, Board_LED_ON);
                    cc1310_usleep(250000, 0);
                    //PIN_setOutputValue(ledPinHandle, Board_LED1, !PIN_getOutputValue(Board_LED1));
                    GPIO_write(Board_LED1, Board_LED_OFF);
                    break;
                }
                default:
                    break;
            }
        }
    }
}

