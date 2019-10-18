/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty_min.c ========
 */
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
#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header files */
#include "components/launcher/LP_Board.h"

#include "components/global/CC1310_I2C.h"
#include "components/tasks/rf_task.h"
#include "components/global/globals.h"
#include "components/tasks/GPS_task.h"


#define TASKSTACKSIZE   2048
#define BMM_STACKSIZE   512

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

Task_Struct bmm_task_struct;
Char bmm_task_stack[BMM_STACKSIZE];

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
    Board_DIO22 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//	Board_3V3_EN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE
};

/*
 *  ======== main ========
 */
int main(void){
    Task_Params taskParams;
    Task_Params bmm_taskParams;
    Task_Params rf_task_params;

    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    CC1310_I2C_init();
    init_locks();
    // Board_initSPI();
   // Board_initUART();
    // Board_initWatchdog();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;

    Task_Params_init(&bmm_taskParams);
    bmm_taskParams.stackSize = BMM_STACKSIZE;
    bmm_taskParams.stack = &bmm_task_stack;
    Task_construct(&task0Struct, (Task_FuncPtr)GPS_task_run, &taskParams, NULL);
//    Task_construct(&task0Struct, (Task_FuncPtr)bmi160_task_run, &taskParams, NULL);
//    Task_construct(&bmm_task_struct, (Task_FuncPtr)bmm050_task_run, &bmm_taskParams, NULL);

    /* Construct RF Task  thread */
//    Task_Params_init(&rf_task_params);
//    rf_task_params.arg0 = NULL;
//    rf_task_params.stackSize = RF_TASK_STACK_SIZE;
//    rf_task_params.stack = &rf_task_stack;
//    Task_construct(&rf_task_struct, (Task_FuncPtr)rf_task_run, &rf_task_params, NULL);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }
    pinHandle = ledPinHandle;
    PIN_setOutputValue(pinHandle, Board_LED1, 1);
    PIN_setOutputValue(pinHandle, Board_LED0, 0);
    PIN_setOutputValue(pinHandle, Board_DIO22, 0);
//    PIN_setOutputValue(ledPinHandle, Board_3V3_EN, 1);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
