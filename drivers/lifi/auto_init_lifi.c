/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *               2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     sys_auto_init_gnrc_netif
 * @{
 *
 * @file
 * @brief       Auto initialization for cc110x network interfaces
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#include <net/gnrc/netif.h>
#include <cc1xxx_common.h>
#include <lifi.h>
#include "auto_init_lifi.h"
#include "lifi_params.h"
#include "net/gnrc/netif/conf.h"    /* <- GNRC_NETIF_MSG_QUEUE_SIZE */

#define ENABLE_DEBUG 0
#include "debug.h"

#ifndef LIFI_EXTRA_STACKSIZE
/**
 * @brief   Additional stack size required by the driver
 *
 * With increasing of GNRC_NETIF_MSG_QUEUE_SIZE the required stack size
 * increases as well. A queue size of 8 messages works with default stack size,
 * so we increase the stack by `sizeof(msg_t)` for each additional element
 */
#define LIFI_EXTRA_STACKSIZE          ((GNRC_NETIF_MSG_QUEUE_SIZE - 8) * sizeof(msg_t))
#endif

/**
 * @brief   Calculate the stack size for the MAC layer thread(s)
 */
#define LIFI_MAC_STACKSIZE            (THREAD_STACKSIZE_DEFAULT + \
                                        LIFI_EXTRA_STACKSIZE + \
                                        DEBUG_EXTRA_STACKSIZE)
#ifndef LIFI_MAC_PRIO
/**
 * @brief   The priority of the MAC layer thread
 */
#define LIFI_MAC_PRIO                 (GNRC_NETIF_PRIO)
#endif

/**
 * @brief   Calculate the number of configured CC1100/CC1101 transceivers
 */
#define LIFI_NUM                      ARRAY_SIZE(lifi_params)

/**
 * @brief   Statically allocate memory for device descriptors
 */
lifi_t lifi_dev;
static gnrc_netif_t lifi_netif;
/**
 * @brief   Statically allocate memory for the MAC layer thread(s)
 */
static char stacks[1][LIFI_MAC_STACKSIZE];

void auto_init_lifi(void)
{
    DEBUG("[LiFi] Auto init\n");
    lifi_setup(&lifi_dev,&lifi_params[0],0);
    gnrc_netif_cc1xxx_create(&lifi_netif, stacks[0], LIFI_MAC_STACKSIZE, LIFI_MAC_PRIO,
                             "lifi", &lifi_dev.netdev);
}
/** @} */
