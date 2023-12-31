/*! ----------------------------------------------------------------------------
 * @file    port.h
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#if CONFIG_SOC_SERIES_NRF52X
#define NRF52840_XXAA
#endif

#define UNUSED(X) (void)X
#define UNUSED_PARAMETER(X) (void)X

typedef void (*port_deca_isr_t)(void);

void Sleep(uint32_t Delay);
void device_init(void);
void reset_DWIC(void);
void port_set_dw_ic_spi_slowrate(void);
void port_set_dw_ic_spi_fastrate(void);
void port_set_dwic_isr(port_deca_isr_t deca_isr);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
