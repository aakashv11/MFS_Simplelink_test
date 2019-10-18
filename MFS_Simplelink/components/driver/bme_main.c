
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header files */
#include "components/global/DRONE_Board.h"

#include "components/tasks/bme_task.h"
#include "components/tasks/rf_task.h"
#include "components/global/globals.h"
#include "components/global/CC1310_I2C.h"
#include <stdlib.h>

Task_Struct bme_task_struct;
Char bme_task_stack[BME_TASK_STACK_SIZE];

Task_Struct rf_task_struct;
Char rf_task_stack[RF_TASK_STACK_SIZE];

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//	Board_3V3_EN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE
};

/*
 *  ======== main ========
 */
int main(void)
{
	Task_Params bme_task_params;
    Task_Params rf_task_params;

    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    CC1310_I2C_init();
    init_locks();
    // Board_initSPI();
    // Board_initUART();
    // Board_initWatchdog();

    /* Construct BME Task  thread */
    Task_Params_init(&bme_task_params);
    bme_task_params.arg0 = NULL;
    bme_task_params.stackSize = BME_TASK_STACK_SIZE;
    bme_task_params.stack = &bme_task_stack;
    Task_construct(&bme_task_struct, (Task_FuncPtr)bme280_task_run, &bme_task_params, NULL);

    /* Construct RF Task  thread */
    Task_Params_init(&rf_task_params);
    rf_task_params.arg0 = NULL;
    rf_task_params.stackSize = RF_TASK_STACK_SIZE;
    rf_task_params.stack = &rf_task_stack;
    Task_construct(&rf_task_struct, (Task_FuncPtr)rf_task_run, &rf_task_params, NULL);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    PIN_setOutputValue(ledPinHandle, Board_LED0, 1);
    PIN_setOutputValue(ledPinHandle, Board_LED1, 1);
//    PIN_setOutputValue(ledPinHandle, Board_3V3_EN, 1);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
