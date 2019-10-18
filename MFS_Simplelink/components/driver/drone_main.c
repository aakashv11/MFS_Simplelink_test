
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

/* Board Header files */
#include "components/global/DRONE_Board.h"
//#include "LP_Board.h"

#include "components/tasks/bme_task.h"
#include "components/tasks/GPS_task.h"
#include "components/tasks/rf_task.h"
#include "components/global/globals.h"
#include "components/global/CC1310_I2C.h"
#include <stdlib.h>

#if BME_TASK
Task_Struct bme_task_struct;
char bme_task_stack[BME_TASK_STACK_SIZE];
#endif
#if GPS_TASK
Task_Struct gps_task_struct;
char gps_task_stack[GPS_TASK_STACK_SIZE];
#endif
#if RF_TASK
Task_Struct rf_task_struct;
char rf_task_stack[RF_TASK_STACK_SIZE];
#endif
/* Pin driver handle */
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
static PIN_Config ledPinTable[] = {
	Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//	Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_GPS_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_IMU_INT | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_TESTPOINT_1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_TESTPOINT_2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	//	Board_3V3_EN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE
};

// Clock functions/variables
void increment(UArg);
Clock_Struct clkStruct;

/*
 *  ======== main ========
 */
int main(void)
{
    /* Construct BIOS Objects */
    Clock_Params clkParams;
#if BME_TASK
    Task_Params bme_task_params;
#endif

#if GPS_TASK
Task_Params gps_task_params;
#endif

#if RF_TASK
    Task_Params rf_task_params;
#endif




    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    CC1310_I2C_init();
    init_locks();

    // Initialize the timestamp function for the program
    Clock_Params_init(&clkParams);
    clkParams.period = 5000/Clock_tickPeriod;
    clkParams.startFlag = TRUE;

    /* Construct a periodic Clock Instance */
    // Wakes up every 1 ms
//    Clock_construct(&clkStruct, (Clock_FuncPtr)increment,
//                    1000/Clock_tickPeriod, &clkParams);
#if BME_TASK
    /* Construct BME Task  thread */
    Task_Params_init(&bme_task_params);
    bme_task_params.arg0 = NULL;
    bme_task_params.stackSize = BME_TASK_STACK_SIZE;
    bme_task_params.priority = BME_PRIORITY;
    bme_task_params.stack = &bme_task_stack;
    Task_construct(&bme_task_struct, (Task_FuncPtr)bme280_task_run, &bme_task_params, NULL);
    bme_task_handle = Task_handle(&bme_task_struct);
#endif


#if GPS_TASK
   //  Construct GPS Task  thread
    Task_Params_init(&gps_task_params);
    gps_task_params.arg0 = NULL;
    gps_task_params.stackSize = GPS_TASK_STACK_SIZE;
    gps_task_params.priority = GPS_PRIORITY;
    gps_task_params.stack = &gps_task_stack;
    Task_construct(&gps_task_struct, (Task_FuncPtr)GPS_task_run, &gps_task_params, NULL);
    gps_task_handle = Task_handle(&gps_task_struct);
#endif


#if RF_TASK
    /* Construct RF Task  thread */
    Task_Params_init(&rf_task_params);
    rf_task_params.arg0 = NULL;
    rf_task_params.stackSize = RF_TASK_STACK_SIZE;
    rf_task_params.priority = RF_PRIORITY;
    rf_task_params.stack = &rf_task_stack;
    Task_construct(&rf_task_struct, (Task_FuncPtr)rf_task_run, &rf_task_params, NULL);
    rf_task_handle = Task_handle(&rf_task_struct);
#endif

    /* Open LED pins */
    pinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!pinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    PIN_setOutputValue(pinHandle, Board_LED0, 1);
//    PIN_setOutputValue(pinHandle, Board_LED1, 1);
    PIN_setOutputValue(pinHandle, Board_GPS_POWER, 0);



    /* Start BIOS */
    BIOS_start();
    return (0);
}


void increment(UArg arg0) {
	++curr_time;
}

