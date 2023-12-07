/**
 * Copyright (c) 2019 - Frederic Mes, RTLOC
 * Copyright (c) 2015 - Decawave Ltd, Dublin, Ireland.
 * Copyright (c) 2021 - Home Smart Mesh
 * Copyright (c) 2022 - Sven Hoyer
 *
 * This file is part of Zephyr-DWM1001.
 *
 *   Zephyr-DWM1001 is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Zephyr-DWM1001 is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Zephyr-DWM1001.  If not, see <https://www.gnu.org/licenses/>.
 *
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

#define APP_NAME "my_anchor\n"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void spi_err_cb(const dwt_cb_data_t *cb_data);
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

uint8_t this_initiator_node_id = 1;
uint8_t responder_node_id = 2;

/*rx twr_2_resp after tx twr_1_poll
 protected by responder's mp_request_at(twr_2_resp):POLL_RX_TO_RESP_TX_DLY_UUS
*/
#define POLL_TX_TO_RESP_RX_DLY_UUS 500

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 650
uint8_t my_rx_buf[10];
uint64_t timestamp;
uint64_t old_timestamp;
uint64_t period;
#define PRINT_DETAILED 1
#define NODE_ID 5
bool rx_event = false;
bool error_event = false;
bool spi_error_event = false;
dwt_rxdiag_t diag_info;
k_timeout_t my_timeout;

int main(void)
{
    printk(APP_NAME);
    printk("==================\n");
    my_timeout.ticks = 10000;
    int init_ok = sit_init(&config, TX_ANT_DLY, RX_ANT_DLY);

    // INIT LED and let them Blink one Time to see Intitalion Finshed
    sit_led_init();
    dw3000_hw_interrupt_enable();
    if (init_ok < 0)
    {
        sit_set_led(2, 0);
    }
    else
    {
        sit_set_led(1, 0);
    }
    dwt_setleds(1);
    uint32_t regStatus = sit_get_device_status();
    LOG_INF("statusreg = 0x%08x", regStatus);
    k_sleep(K_SECONDS(2)); // Allow Logging to write output 
    int frame_sequenz = 0;
    printk("TEST\n");
    printk("TEST\n");
    k_sleep(K_SECONDS(1)); // Allow Logging to write output 
    regStatus = sit_get_device_status();
    LOG_INF("sequence(%u) starting ; statusreg = 0x%08x", frame_sequenz, regStatus);
     k_sleep(K_SECONDS(1)); // Allow Logging to write output 
    printk("2\n");
    k_sleep(K_SECONDS(1)); // Allow Logging to write output 
    dwt_setcallbacks(NULL, &rx_ok_cb, &rx_to_cb, &rx_err_cb, &spi_err_cb, NULL, NULL);
    printk("3\n");
    k_sleep(K_SECONDS(1)); // Allow Logging to write output 
    dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXSTO_BIT_MASK,
                     0, DWT_ENABLE_INT);
    printk("4\n");
    k_sleep(K_SECONDS(1)); // Allow Logging to write output 
    port_set_dwic_isr(dwt_isr);
    printk("5\n");
    k_sleep(K_SECONDS(1)); // Allow Logging to write output 
    sit_receive_at(0);
    while (1)
    {   
            timestamp = get_rx_timestamp_u64();
            period = timestamp - old_timestamp;
            old_timestamp = timestamp;
            
            /* A frame has been received, copy it to our local buffer. */
#if PRINT_DETAILED
            dwt_readrxdata(my_rx_buf, 10, 0);
            LOG_INF("rx data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                    my_rx_buf[0], my_rx_buf[1], my_rx_buf[2], my_rx_buf[3], my_rx_buf[4], my_rx_buf[5], my_rx_buf[6], my_rx_buf[7], my_rx_buf[8], my_rx_buf[9]);

            LOG_INF("Timestamp:%llx, period : %llx\n ", timestamp, period);
#endif      
            k_sleep(K_MSEC(RNG_DELAY_MS));
            printk("RXTS:%d:"
                   "%" PRIX64 "\n",
                   NODE_ID, timestamp);
            sit_receive_at(0);
             k_sleep(K_MSEC(4000));
    } 
   
    return 0;
}

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{

    rx_event = true;

    /* TESTING BREAKPOINT LOCATION #1 */
}
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{

    printk("rx timeout\n");
    k_sleep(K_MSEC(RNG_DELAY_MS));
    /* TESTING BREAKPOINT LOCATION #1 */
}
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{

    error_event = true;
    /* TESTING BREAKPOINT LOCATION #1 */
}

static void spi_err_cb(const dwt_cb_data_t *cb_data){
    spi_error_event = true;
}