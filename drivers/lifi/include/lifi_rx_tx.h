/*
 * Copyright (C) 2018 Otto-von-Guericke-Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_cc110x
 * @{
 *
 * @file
 * @brief       Functions to related to RX/TX of the CC110x transceiver driver
 *
 * @author      Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 */

#ifndef LIFI_RX_TX_H
#define LIFI_RX_TX_H

#include "net/netdev.h"
#include "lifi.h"

#define FULL_EDGES_DROP_PIN GPIO_PIN(PORT_F, 15)
#define BIT_INTERPRETATION_PIN GPIO_PIN(PORT_G, 14)
#define TIMING_ISSUE_PIN GPIO_PIN(PORT_B, 8)
#define RECEIVE_INTERRUPT GPIO_PIN(PORT_B, 9)
#define CLOCK_PIN GPIO_PIN(PORT_E, 13)
#define DATA_SENDER_PIN GPIO_PIN(PORT_E, 11)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   ISR to be called via @ref netdev_driver_t::isr
 */
void lifi_isr(netdev_t *dev);

void lifi_send_frame(lifi_t* lifi_dev);

/**
 * @brief   Bring transceiver into RX mode
 *
 * @param   dev     The device descriptor of the transceiver
 *
 * @pre     @p dev has been acquired using @ref cc110x_acquire
 * @post    @p dev is still acquired, the caller has to release it
 */
void lifi_enter_rx_mode(lifi_t *dev);

/**
 *
 * @param lifi_dev
 * @param storage_byte
 * @param bit_to_read
 * @return true if bit stored, false if bit discarded
 */
void read_store_bit(lifi_t* lifi_dev, uint8_t* storage_byte, uint8_t bit_to_read);
void lifi_preamble_sync(lifi_t* lifi_dev);
void init_transceiver_state(lifi_t* lifi_dev);

#ifdef __cplusplus
}
#endif

#endif /* CC110X_RX_TX_H */
/** @} */
