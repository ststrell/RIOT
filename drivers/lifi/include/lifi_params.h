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
 * @brief       cc110x board specific configuration
 *
 * @author      Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 */

#ifndef LIFI_PARAMS_H
#define LIFI_PARAMS_H

#include "board.h"
#include "lifi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Default parameters for the lifi driver
 *
 * These values are based on the msba2 board
 * @{
 */

#ifndef LIFI_PARAM_OUTPUT
#define LIFI_PARAM_OUTPUT             GPIO_PIN(1, 21) /**< SPI-CS connected to LIFI */
#endif

#ifndef LIFI_PARAMS
/**
 * @brief   Default initialization parameters of the CC110x driver
 */
#define LIFI_PARAMS               { \
        .outPut = LIFI_PARAM_OUTPUT \
}

#endif
/** @} */

/**
 * @brief   CC110X initialization parameters
 */
static const lifi_params_t lifi_params[] = {
    LIFI_PARAMS
};

#ifdef __cplusplus
}
#endif
#endif /* CC110X_PARAMS_H */
/** @} */
