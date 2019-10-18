
#include "rf_task.h"
#include "components/global/globals.h"
//#include "LP_Board.h"
#include "components/global/DRONE_Board.h"

#include <string.h>
#include <stdio.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/GPIO.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Swi.h>
//#define BASE_FQ         915000000



static void send_msg(struct data_hdr_t* hdr_ptr, void* data_ptr, size_t len,
					 uint8_t dest_addr, uint8_t msg_type, uint32_t seqn);

//static void next_channel(void);


// Initialize all parameters for rf task
EasyLink_Status rf_task_init(void)
{

    EasyLink_Status status = EasyLink_Status_Success;

	status = EasyLink_init(EasyLink_Phy_50kbps2gfsk);
    /* Set Freq to BASE_FQ defined in globals.h */
	status = EasyLink_setFrequency(BASE_FQ);
    /* Set output power to 14dBm */
	status = EasyLink_setRfPwr(14);

	int i;
	for (i = 0; i < NUM_MSGS; ++i)
	{
		msg_hdrs[i].data_hdr_vers = DRONE_VERS;
		msg_hdrs[i].data_hdr_type = MSG_TYPE_UNDEF;
		msg_hdrs[i].data_hdr_seqn = SEQN_UNINIT;
		msg_hdrs[i].address = CURRENT_ADDR;
	}

	return status;
}

// Main rf loop to transmit to LCM
void rf_task_run(void)
{
	int i = 0;
	int init_status = 0;
	init_status = rf_task_init();
	//TODO: Why does this function return an error status? This then breaks the code..
//	if(init_status != 0)
//	{
//	    while(1); //error....
//	}
    char msg_idx = 1;	// index of current message
    unsigned short next_seqn = 0;

	// Main rf loop
	while (1)
	{

		msg_idx ^= 1;	// XOR index with 1 to alternate between 0 and 1
#if BME_TASK
		Semaphore_pend(data_rdy_lock_handle, BIOS_WAIT_FOREVER);
#endif
#if GPS_TASK
		Semaphore_pend(gps_rdy_lock_handle, BIOS_WAIT_FOREVER);
#endif

	    // Data is ready, prepare to tx
		//Set TESTPOINT_1 high to signal start of RF transmission
  //      PIN_setOutputValue(pinHandle, Board_TESTPOINT_2, 1);

#if BME_TASK
        // Send BME MSG
		send_msg(&(msg_hdrs[msg_idx]), &(bme_msgs[msg_idx]), sizeof(struct bme_msg_t),
				RECEIVER1_ADDR, MSG_TYPE_BME, next_seqn);
#endif
#if GPS_TASK
		// Send GPS MSG
		send_msg(&(msg_hdrs[msg_idx]), &(gps_full_msgs[msg_idx]), sizeof(struct gps_msg_full_t),
				RECEIVER1_ADDR, MSG_TYPE_GPS_FULL, next_seqn);
#endif
		//Flash LED every time you send RF message
	    PIN_setOutputValue(pinHandle, Board_LED0, !PIN_getOutputValue(Board_LED0));
	    //Set TESTPOINT_1 low to signal end of RF transmission
//	    PIN_setOutputValue(pinHandle, Board_TESTPOINT_2, 0);


	    //begin repeated data transmission for old data
	    for (int j =0 ;j<NUM_OLD_DATA; j++)
	    {
#if BME_TASK
        // Send BME MSG
        send_msg(&(msg_hdrs_old[j]), &(bme_msgs_old[j]), sizeof(struct bme_msg_t),
                RECEIVER1_ADDR, MSG_TYPE_BME, next_seqn);
#endif
        //Flash LED every time you send RF message
        PIN_setOutputValue(pinHandle, Board_LED0, !PIN_getOutputValue(Board_LED0));
	    }

	    //TODO: use efficient swap or pointer handling
        msg_hdrs_old[1] = msg_hdrs_old[0];
        bme_msgs_old[1] = bme_msgs_old[0];
        gps_full_msgs_old[1] = gps_full_msgs_old[0];

        msg_hdrs_old[0] = msg_hdrs[msg_idx];
        bme_msgs_old[0] = bme_msgs[msg_idx];
        gps_full_msgs_old[0] = gps_full_msgs[msg_idx];

	    //end repeated transmission

	    next_seqn++; //increment sequence count
	}
}


// Sneds message with hdr_ptr and data_ptr to dest_addr
// Switches to next channel after transmission
static void send_msg(struct data_hdr_t* hdr_ptr, void* data_ptr, size_t len,
					 uint8_t dest_addr, uint8_t msg_type, uint32_t seqn)
{
    int txStatus = 0;
    static EasyLink_TxPacket tx_packet;

    hdr_ptr->data_hdr_type = msg_type;
    hdr_ptr->data_hdr_seqn = seqn;

	memcpy(tx_packet.payload,  hdr_ptr, sizeof(struct data_hdr_t));
	memcpy(tx_packet.payload + sizeof(struct data_hdr_t), data_ptr, len);

	tx_packet.len = sizeof(struct data_hdr_t) + len;
	tx_packet.dstAddr[0] = dest_addr;	// Address of receiver

	//Send Packet using EasyLink API
	// Caution: This is a blocking RF call
	txStatus = EasyLink_transmit(&tx_packet);
	//check for error
    if(txStatus != 0)
    {
        while(1); // Error Handling..
    }


//    next_channel();
    cc1310_usleep(50000, 0); // Temporary: This is to allow the receiver enough time to process code
}

// Switches to next channel in frequency table
/*
static void next_channel(void) {
	static char chn_idx = 0;
	chn_idx = (++chn_idx)%ALL_HOPS; // increment fq

	EasyLink_setFrequency(BASE_FQ + (CHN_WIDTH * chn_idx));
}
*/
