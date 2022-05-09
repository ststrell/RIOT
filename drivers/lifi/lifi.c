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
 * @brief       Implementation for the "public" API of the CC1100/CC1101 driver
 *
 * @author      Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 * @}
 */

#include <errno.h>
#include <string.h>
#include <lifi_netdev.h>

#include "lifi.h"

#define ENABLE_DEBUG 0
#include "debug.h"

int lifi_setup(lifi_t *dev, const lifi_params_t *params, uint8_t index)
{
    DEBUG("[lifi] Setting up Lifi\n");
    if (!dev || !params) {
        return -EINVAL;
    }

    /* Zero out everything but RIOT's driver interface, which should be
     * managed by RIOT
     */
    memset((char *)dev + sizeof(netdev_t), 0x00,
           sizeof(lifi_t) - sizeof(netdev_t));
    dev->params = *params;
    dev->netdev.driver = &lifi_driver;
    dev->state = LIFI_STATE_IDLE;
    netdev_register(&dev->netdev, NETDEV_LIFI, index);
    return 0;
}

int lifi_set_baud(lifi_t *dev, uint16_t baud) {
    dev->baud = baud;
    return 0;
}
