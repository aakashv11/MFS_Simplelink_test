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
//#include "Board.h"
#include "components/launcher/LP_Board.h"
#include "components/global/globals.h"
#include "components/tasks/rf_task.h"
#include "components/global/CC1310_UART.h"
#include <stdio.h>
#include <string.h>


/* EasyLink API Header files */
#include "components/easylink/EasyLink.h"

/***** Defines *****/

/* Undefine to remove address filter and async mode */
#define RFEASYLINKRX_ADDR_FILTER

#define RFEASYLINKEX_TASK_STACK_SIZE 1024
#define RFEASYLINKEX_TASK_PRIORITY   2

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] = {
	Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//	Board_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//	Board_LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/***** Variable declarations *****/
static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */
static uint8_t rxTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];

struct data_hdr_t msg_hdrs_recv;
struct bme_msg_t bme_msgs_recv;
struct gps_msg_t gps_msgs_recv;
struct gps_msg_full_t gps_full_msg_recv;

char address_str[] = {"Address: xxx\r\n"};	// len = 14
char sequence_str[] = {"Sequence: xxxxx\r\n"}; // len = 17
char temp_str[] = {"T xxxx.xx C\r\n"};	// len = 13
char press_str[] = {"P XXXX.XX mbar\r\n"};	// len = 16
char humid_str[] = {"H xXX.XXX %\r\n"};	// len = 13
char gps_str[] = {"XXXX.XXXX,X,XXXXX.XXXX,X\r\n"};	// len = 26


/* The RX Output struct contains statistics about the RX operation of the radio */
PIN_Handle pinHandle;

/***** Function declarations *****/
static void rfEasyLinkRxFnx(UArg arg0, UArg arg1);
void rxTask_init(PIN_Handle ledPinHandle);
void process_message(void);
void process_gps_full_message(void);
void get_commands_task(void);

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
    PIN_setOutputValue(ledPinHandle, Board_LED0, 0);
    PIN_setOutputValue(ledPinHandle, Board_LED1, 0);

    rxTask_init(ledPinHandle);

    /* Start BIOS */
    BIOS_start();

    return (0);
}

/***** Function definitions *****/
void rxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
    rxTaskParams.priority = RFEASYLINKEX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;
    rxTaskParams.arg0 = (UInt)1000000;

    //    Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
    Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
}

static void rfEasyLinkRxFnx(UArg arg0, UArg arg1)
{
	EasyLink_RxPacket rxPacket = {0};

    EasyLink_init(EasyLink_Phy_50kbps2gfsk);
    /* Set Freq to BASE_FQ defined in globals.h */
    EasyLink_setFrequency(BASE_FQ);
    /* Set output power to 14dBm */
    EasyLink_setRfPwr(14);

    uint8_t addrFilter = 0xaa;
    EasyLink_enableRxAddrFilter(&addrFilter, 1, 1);

    while(1) {
        rxPacket.absTime = 0;
        EasyLink_Status result = EasyLink_receive(&rxPacket);
        //TODO: Change logic to a case statement
        if (result == EasyLink_Status_Success)
        {
        		/* On success, we copy data from received buffer into data structures */
    			memcpy(&(msg_hdrs_recv), rxPacket.payload, sizeof(struct data_hdr_t));
    			if (msg_hdrs_recv.data_hdr_type == MSG_TYPE_BME) {
    				memcpy(&(bme_msgs_recv), rxPacket.payload + sizeof(struct data_hdr_t), sizeof(struct bme_msg_t));

    				process_message();
    				/* Toggle LED2 to indicate RX */
    				PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));

    			}

    			if(msg_hdrs_recv.data_hdr_type == MSG_TYPE_GPS_FULL)
    			{
                    memcpy(&(gps_full_msg_recv), rxPacket.payload + sizeof(struct data_hdr_t), sizeof(struct gps_msg_full_t));
                    process_gps_full_message();
                    /* Toggle LED2 to indicate RX */
                    PIN_setOutputValue(pinHandle, Board_LED0,!PIN_getOutputValue(Board_LED0));
    			}

        }
        else
        {
            /* Toggle LED1 and LED2 to indicate error */
            PIN_setOutputValue(pinHandle, Board_LED0,!PIN_getOutputValue(Board_LED0));
            PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
        }
    }
}

void process_message(void) {
	int i = 0;

	address_str[11] = '0'+msg_hdrs_recv.address%10;
	address_str[10] = '0'+(msg_hdrs_recv.address/10)%10;
	address_str[9] = '0'+(msg_hdrs_recv.address/100)%10;

	UART_printf(address_str, 14);
    sequence_str[14] = '0' +(msg_hdrs_recv.data_hdr_seqn)%10;
    sequence_str[13] = '0' +(msg_hdrs_recv.data_hdr_seqn/10)%10;
    sequence_str[12] = '0' +(msg_hdrs_recv.data_hdr_seqn/100)%10;
    sequence_str[11] = '0' +(msg_hdrs_recv.data_hdr_seqn/1000)%10;
	sequence_str[10] = '0' +(msg_hdrs_recv.data_hdr_seqn/10000)%10;

	UART_printf(sequence_str, 17);

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

void process_gps_full_message(void)
{
    int i = 0;

    address_str[11] = '0'+msg_hdrs_recv.address%10;
    address_str[10] = '0'+(msg_hdrs_recv.address/10)%10;
    address_str[9] = '0'+(msg_hdrs_recv.address/100)%10;

    UART_printf(address_str, 14);

    //Print out GPS message

    char time_str[] =     "GPS Time: hhmmss.sss \r\n"; //24
    char lat_str[] =      "GPS Lat:  llll.llll a \r\n"; //25
    char long_str[] =     "GPS Long: yyyyy.yyyy a \r\n";//26
    char alt_str[] =      "GPS Alt:  xxx.x \r\n"; //19
    char sat_num[] =      "GPS Sat#: xx \r\n"; //16
    char quality_str[] =  "GPS Qual: x \r\n"; //15
    char hor_res[] =      "GPS Hor:  x.x \r\n"; //17

    //time
    memcpy(time_str+10, gps_full_msg_recv.time, 6);
    memcpy(time_str+17, gps_full_msg_recv.time+6,3);
    UART_printf(time_str, 23);
    //lat
    memcpy(lat_str+10, gps_full_msg_recv.latitude, 4);
    memcpy(lat_str+15, gps_full_msg_recv.latitude+4,4);
    memcpy(lat_str+20, gps_full_msg_recv.N_S, 1);
    UART_printf(lat_str, 25);
    //lon
    memcpy(long_str+10, gps_full_msg_recv.longitude, 5);
    memcpy(long_str+16, gps_full_msg_recv.longitude+5,4);
    memcpy(long_str+21, gps_full_msg_recv.E_W, 1);
    UART_printf(long_str, 26);
    //alt
    alt_str[10] = gps_full_msg_recv.altitude[0];
    alt_str[11] = gps_full_msg_recv.altitude[1];
    alt_str[12] = gps_full_msg_recv.altitude[2];
    alt_str[14] = gps_full_msg_recv.altitude[3];
    UART_printf(alt_str, 19);
    //sat
    sat_num[10] = gps_full_msg_recv.sat_num[0];
    sat_num[11] = gps_full_msg_recv.sat_num[1];
    UART_printf(sat_num, 16);
    //quality
    quality_str[10] = gps_full_msg_recv.quality_indicator;
    UART_printf(quality_str, 15);
    //Horizontal Accuracy
    hor_res[10] = gps_full_msg_recv.horizontal_resolution[0];
    hor_res[12] = gps_full_msg_recv.horizontal_resolution[1];
    UART_printf(hor_res, 17);




}
