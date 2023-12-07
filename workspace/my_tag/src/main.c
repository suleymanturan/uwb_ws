/*! ----------------------------------------------------------------------------
 *  @file    ss_twr_initiator.c
 *  @brief   Single-sided two-way ranging (SS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
 *           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
 *           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
 *           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015 - 2021 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <dw3000_hw.h> 
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>

#include <sit/sit_distance.h>
#include <sit/sit_utils.h>
#include <sit/sit.h>
#include <sit/sit_config.h>
#include <sit_led/sit_led.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Example application name */
#define APP_NAME "SS TWR INIT v1.0"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    DWT_SFD_DW_8,     /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

uint8_t this_initiator_node_id  = 1;
uint8_t responder_node_id       = 2;

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 2000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 400

uint8_t my_tx_buf[10] = {0x00, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,0x48,0x49,0x50};
uint32_t regStatus = 0;

int main(void) {
	printk(APP_NAME);
	printk("==================\n");

	int init_ok = sit_init(&config, TX_ANT_DLY, RX_ANT_DLY);
	// INIT LED and let them Blink one Time to see Intitalion Finshed
    sit_led_init();

    if(init_ok < 0){
        sit_set_led(2, 0);
    } else {
        sit_set_led(1, 0);
    }
    
    uint32_t regStatus = sit_get_device_status();
    LOG_INF("statusreg = 0x%08x",regStatus);
    k_sleep(K_SECONDS(2)); // Allow Logging to write out put 

    uint8_t frame_sequenz = 0;
    k_yield();
    
	while (1) {
        k_sleep(K_MSEC(RNG_DELAY_MS));
        my_tx_buf[0]++;
        dwt_writetxdata(10, my_tx_buf, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);
	}
    return 0;
}