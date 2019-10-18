/*
 * GPS_task.c
 *
 *  Created on: May 26, 2016
 *      Author: Nicholas Metcalf
 */

#include "GPS_task.h"
#include "components/global/DRONE_Board.h"
//#include "LP_Board.h"
#include "components/global/globals.h"

#include <ti/drivers/UART.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <string.h>
#include <stdlib.h>

UART_Handle uart;

#define MMS 100


int GPS_msg_available = 0;

static void enter_critical_section(void);
static void exit_critical_section(void);
static void GPS_GGA_parse_lat_lon(char * search_str, char idx);
static void GPS_GGA_parse_full(char * search_str, char idx);
static void GPS_GGA_parse_full_nico(char* search_str, char idx);
static void GPS_uart_callback(void);
static int GPS_init(void);


void GPS_task_run(void)
{

    char message[MMS];
    char tkn;
    char count = 0;
    char idx = 0;
    GPS_init();

	// Read each line one by one from gps
	while (1)
	{

	 //   PIN_setOutputValue(pinHandle, Board_TESTPOINT_1, 1);
	 //   PIN_setOutputValue(pinHandle, Board_TESTPOINT_2, 1);
	    UART_read(uart, (void *)&tkn, 1);

	    if(GPS_msg_available == 1)
	    {
	        GPS_msg_available = 0;
	        message[count] = tkn;
	            ++count;
	            if (tkn == '\n')
	            {
	                PIN_setOutputValue(pinHandle, Board_TESTPOINT_1, !PIN_getOutputValue(Board_TESTPOINT_1));
                     // Once we finish a line, check if msg is a GPGGA
                    if (message[3] == 'G' && message[4] == 'G' && message[5] == 'A')
                    {
                         //we have the message we need
                    //     GPS_GGA_parse_lat_lon(message, idx);
                         GPS_GGA_parse_full_Nico(message, idx);
                         Semaphore_post(IMU_rdy_lock_handle); // let RF task know
                         idx ^= 1;
                     //    ready = 1;
                      //   cc1310_usleep(100000, 0); //sleep for half a second
                     }
                count = 0;
               } // if end of message
            //}//if(GPS_msg_available)
	         //   Task_yield();
	    }
	//    PIN_setOutputValue(pinHandle, Board_TESTPOINT_1, 0);
	    Task_yield();
	}//while(1)

}//void

static void enter_critical_section(void)
{
#if RF_TASK
	Task_setPri(rf_task_handle, PRIORITY_OFF);
#endif
#if BME_TASK
	Task_setPri(bme_task_handle, PRIORITY_OFF);
#endif
}

static void exit_critical_section(void)
{
#if RF_TASK
	Task_setPri(rf_task_handle, RF_PRIORITY);
#endif
#if BME_TASK
	Task_setPri(bme_task_handle, BME_PRIORITY);
#endif
}

static void GPS_GGA_parse_lat_lon(char * msg, char idx)
{
	static char * comma_ptr = NULL;
	static char * prev_comma_ptr = NULL;

	// Find first comma
	comma_ptr = strchr(msg, ',');
	// Find second - start of actual lat/long data
	prev_comma_ptr = comma_ptr;
	comma_ptr = strchr(prev_comma_ptr+1, ',');
	// Find second - end of lat data
	prev_comma_ptr = comma_ptr;
	comma_ptr = strchr(++prev_comma_ptr, ',');
	// Convert latitude data of string double and store in gps msg
	gps_msgs[idx].lat_ = atof(prev_comma_ptr);
	gps_msgs[idx].north = *(++comma_ptr);
	// Find third - end of longitude data
	prev_comma_ptr = comma_ptr;
	if (*prev_comma_ptr != ',') {
	  ++prev_comma_ptr;
	}
	comma_ptr = strchr(++prev_comma_ptr, ',');
	// Convert longitude data of string double and store in gps msg
	gps_msgs[idx].long_ = atof(prev_comma_ptr);
	gps_msgs[idx].east = *(++comma_ptr);
}

// There are a few things to note about this function.
//  1) It relies on the assumption that there will ALWAYS be a correct number
//     of commas in the output. Otherwise, tokenizing function will have
//     problems.
//
//  2) I changed any possible blank outputs to the string `N/A`. I chose
//     this string because I am not sure of what characters are (and are not)
//     capable of being sent/recieved by the GPS. If there ends up being
//     a single character, such as `?` or `|`, then I would advise changing
//     the code to use one of those characters, for aesthetic and parsing
//     reasons.
//
//  3) The team should really make use of this function `strtok`, it makes
//     dealing with commas as delimiters NOT a nightmare. This dilemma
//     seems to plague a lot of the functions in this file, and may be the
//     cause of who knows how many bugs/errors.
//
//  4) The reason I replaced the blank outputs (,,) with non-null placeholders
//     (,N/A,) was because the tokenizing function will just skip duplicate
//     delimiters. This is problematic if we are trying to keep track of which
//     field on the NMEA string we are evaluating. So the placeholders allow
//     a consistent number of fields in every NMEA string
//
//  5) The function wouldn't be nearly as long if `gps_msg_full_t` struct,
//     whos instance is `gps_full_msgs` would keep its char pointers within
//     an array. That way we could just iterate through that array and store
//     the incremented tokens, as opposed to having to manually type out
//     each store and iteration.
//
//  6) Finally, the main difference that you will notice is that this
//     function does not individually allociate each index of the data, as the
//     previous function did. I believe this was the root cause of the parsing
//     errors. As the outputs became larger (i.e. more digits) the placements
//     would shift in a way that the program hadn't accounted for. Thus 
//     misaligning all of the data. This method of tokenizing (ideally) avoids
//     this problem altogether.
//
// Given the NMEA string, `msg`, makes a copy of the string, substituting 
// blank output for a non-null sequence, such as `N/A`. Then tokenizes the
// fields of the string to be stored in the `gps_full_msgs` struct.
static void GPS_GGA_parse_full_nico(char *msg, char idx)
{
  // TODO: Ensure that the assigned values don't change once the pointers
  //        point to a new spot. They shouldn't, but just to be sure.
  char modified[100] = "";
  char *left = msg;
  char *right = msg; ++right;
  int idx = 0;
  while (*right != '\0')
  {
    modified[idx++] = *left;
    if ((*right == ',') && (*left == ','))
    {
      modified[idx++] = 'N';
      modified[idx++] = '/';
      modified[idx++] = 'A';
    }
    ++left;
    ++right;
    if (*right == '\0')
    { // Copy last character and null character
      modified[idx++] = *left;
      if (*left == 'm')
      {
        modified[idx++] = 'N';
        modified[idx++] = '/';
        modified[idx++] = 'A';
      }
      modified[idx++] = *right;
    }
  } // Blank output replaced with `N/A`

  // Tokenize the newly modified string
  char *token;
  token = strtok(modified, ","); // Skip the GPGGA token
  token = strtok(NULL, ","); // `NULL` indicates to move to the next
                             // token in the previously used string
  gps_full_msgs[idx].time = token;
  token = strtok(NULL, ",");
  gps_full_msgs[idx].latitude = token;
  token = strtok(NULL, ",");
  gps_full_msgs[idx].N_S = token;
  token = strtok(NULL, ",");
  gps_full_msgs[idx].longitude = token;
  token = strtok(NULL, ",");
  gps_full_msgs[idx].E_W = token;
  token = strtok(NULL, ",");
  gps_full_msgs[idx].quality_indicator = token;
  token = strtok(NULL, ",");
  gps_full_msgs[idx].sat_num = token;
  token = strtok(NULL, ",");
  gps_full_msgs[idx].horizontal_resolution = token;
  token = strtok(NULL, ",");
  gps_full_msgs[idx].altitude = token;
}


static void GPS_GGA_parse_full(char * msg, char idx)
{
    // Fill in this struct
    /*
    struct gps_msg_full_t
    {
        //27 bytes
        char time[9];
        char latitude[8];
        char N_S[1];
        char longitude[9];
        char E_W[1];
        char horizontal_resolution[2];
        char altitude[2];
    };
    */
    gps_full_msgs[idx].time[0] = msg[7];
    gps_full_msgs[idx].time[1] = msg[8];
    gps_full_msgs[idx].time[2] = msg[9];
    gps_full_msgs[idx].time[3] = msg[10];
    gps_full_msgs[idx].time[4] = msg[11];
    gps_full_msgs[idx].time[5] = msg[12];
    //skip period
    gps_full_msgs[idx].time[6] = msg[14];
    gps_full_msgs[idx].time[7] = msg[15];
    gps_full_msgs[idx].time[8] = msg[16];
    //skip comma
    gps_full_msgs[idx].latitude[0] = msg[18];
    gps_full_msgs[idx].latitude[1] = msg[19];
    gps_full_msgs[idx].latitude[2] = msg[20];
    gps_full_msgs[idx].latitude[3] = msg[21];
    //skip period
    gps_full_msgs[idx].latitude[4] = msg[23];
    gps_full_msgs[idx].latitude[5] = msg[24];
    gps_full_msgs[idx].latitude[6] = msg[25];
    gps_full_msgs[idx].latitude[7] = msg[26];
    //skip comma is 27
    gps_full_msgs[idx].N_S[0] = msg[28];

    //skip comma is 29
    gps_full_msgs[idx].longitude[0] = msg[30];
    gps_full_msgs[idx].longitude[1] = msg[31];
    gps_full_msgs[idx].longitude[2] = msg[32];
    gps_full_msgs[idx].longitude[3] = msg[33];
    gps_full_msgs[idx].longitude[4] = msg[34];
    // skip period is 35
    gps_full_msgs[idx].longitude[5] = msg[36];
    gps_full_msgs[idx].longitude[6] = msg[37];
    gps_full_msgs[idx].longitude[7] = msg[38];
    gps_full_msgs[idx].longitude[8] = msg[39];
    //skip comma is 40
    gps_full_msgs[idx].E_W[0] = msg[41];
    //skip comma
    gps_full_msgs[idx].quality_indicator = msg[43];
    //skip comma
    gps_full_msgs[idx].sat_num[0] = msg[45];
    gps_full_msgs[idx].sat_num[1] = msg[46];
    //skip comma
    gps_full_msgs[idx].horizontal_resolution[0] = msg[48];
    //skip period
    gps_full_msgs[idx].horizontal_resolution[1] = msg[50];
    //skip comma
    gps_full_msgs[idx].altitude[0] = msg[52];
    gps_full_msgs[idx].altitude[1] = msg[53];
    gps_full_msgs[idx].altitude[2] = msg[54];
    //skip comma
    gps_full_msgs[idx].altitude[3] = msg[56];

}




static void GPS_uart_callback(void)
{
    GPS_msg_available = 1;
 //   Semaphore_post(gps_rdy_lock_handle);

}

static int GPS_init(void)
{
    UART_init();
    UART_Params uart_params;
    /* Create a UART with data processing off. */
    UART_Params_init(&uart_params);
    uart_params.writeDataMode = UART_DATA_BINARY;
    uart_params.readDataMode = UART_DATA_BINARY;
    uart_params.readReturnMode = UART_RETURN_FULL;
    uart_params.readEcho = UART_ECHO_OFF;
    uart_params.baudRate = 4800;
    uart_params.readMode = UART_MODE_CALLBACK;
   // uart_params.readMode = UART_MODE_BLOCKING;
    uart_params.readCallback = &GPS_uart_callback;
    uart = UART_open(Board_UART, &uart_params);

    if (uart == NULL)
    {
        System_abort("Error opening the GPS UART\n");
        return(-1);
    }

    // wake up gps
    //TODO: Figure out how to do this without critical sections
    enter_critical_section();

    cc1310_usleep(500000, 0);
    PIN_setOutputValue(pinHandle, Board_GPS_POWER, 1);
    cc1310_usleep(1000, 0);
    PIN_setOutputValue(pinHandle, Board_GPS_POWER, 0);
    cc1310_usleep(1000, 0);
    PIN_setOutputValue(pinHandle, Board_GPS_POWER, 1);
    cc1310_usleep(1000, 0);
    PIN_setOutputValue(pinHandle, Board_GPS_POWER, 0);
    cc1310_usleep(1000, 0);

    exit_critical_section();

    return 0;
}
