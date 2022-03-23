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
 * @brief       Implementation of RIOT's netdev_driver API for the CC1100/CC1101
 *              transceiver
 *
 * @author      Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 * @}
 */

#include <errno.h>
#include <string.h>
#include <lifi_rx_tx.h>
#include <cc1xxx_common.h>

#include "assert.h"
#include "iolist.h"
#include "irq.h"
#include "mutex.h"
#include "net/eui64.h"
#include "net/netdev.h"
#include "xtimer.h"

#include "lifi.h"
#include "lifi_netdev.h"
#include "lifi_rx_tx.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define PREAMBLE 0b11111111

static int lifi_init(netdev_t *netdev);
static int lifi_recv(netdev_t *netdev, void *buf, size_t len, void *info);
static int lifi_send(netdev_t *netdev, const iolist_t *iolist);
static int lifi_get(netdev_t *netdev, netopt_t opt,
                    void *val, size_t max_len);
static int lifi_set(netdev_t *netdev, netopt_t opt,
                    const void *val, size_t len);


const netdev_driver_t lifi_driver = {
    .init = lifi_init,
    .recv = lifi_recv,
    .send = lifi_send,
    .isr = lifi_isr,
    .get = lifi_get,
    .set = lifi_set,
};

#define TIMEOUT_TICKS xtimer_ticks_from_usec(10000)

void isr_callback_input_pin(void *_dev)
{
    gpio_toggle(RECEIVE_INTERRUPT);
    lifi_t *lifi_dev = (lifi_t *)_dev;
    lifi_transceiver_state_t* transceiver_state = &lifi_dev->transceiver_state;

    // check if we have a timeout on lastReceive in preamble frame part -> reset
    if(transceiver_state->current_frame_part == e_preamble && xtimer_diff(xtimer_now(),transceiver_state->lastReceive).ticks32 > TIMEOUT_TICKS.ticks32){
        transceiver_state->current_frame_part = e_first_receive;
        transceiver_state->currentByte = 0;
        transceiver_state->currentBit = 7;
        transceiver_state->high_bit_counter = 0;
    }

    // check if we have a timeout in e_len/e_payload/e_crc16 mode -> reset
    if((transceiver_state->current_frame_part == e_len || transceiver_state->current_frame_part == e_payload ||transceiver_state->current_frame_part == e_crc16)
    && xtimer_diff(xtimer_now(),transceiver_state->lastHalfEdge).ticks32 > TIMEOUT_TICKS.ticks32 ){
        transceiver_state->current_frame_part = e_first_receive;
        transceiver_state->currentByte = 0;
        transceiver_state->currentBit = 7;
        transceiver_state->high_bit_counter = 0;
    }

    // Sync to sender frequency
    if (transceiver_state->current_frame_part == e_first_receive
        || transceiver_state->current_frame_part == e_preamble) {
        lifi_preamble_sync(lifi_dev);
    }
    // receive data
    else {
        // first call here means, that preamble is done. Preamble finished with half period
        // next interrupt can happen either at next full period or next half period, depends on the first byte to transfer

        // calculate ticks diff between now and last half period receive, used to check if edge has to be dropped
        xtimer_ticks32_t timediff = xtimer_diff(xtimer_now(), transceiver_state->lastHalfEdge);

        if (timediff.ticks32 >= transceiver_state->min_tolerated_half_clock.ticks32 &&
            timediff.ticks32 <= transceiver_state->max_tolerated_half_clock.ticks32) {
            // Drop edges on full period
            gpio_toggle(FULL_EDGES_DROP_PIN);
        }
        // edge seems to be on half period
        else if (timediff.ticks32 >= transceiver_state->min_tolerated_half_clock.ticks32 * 2 &&
                 timediff.ticks32 <= 2 * transceiver_state->max_tolerated_half_clock.ticks32) {
            transceiver_state->lastHalfEdge = xtimer_now();
            // edge is in expected half period timing

            // check current frame part and currentByte to decide where to store the bit
            if(transceiver_state->current_frame_part == e_len) {
                read_store_bit(lifi_dev,&lifi_dev->input_buf.len , transceiver_state->currentBit--);
            } else if(transceiver_state->current_frame_part == e_payload){
                read_store_bit(lifi_dev,&lifi_dev->input_buf.payload[transceiver_state->currentByte-1] , transceiver_state->currentBit--);
            } else {
                read_store_bit(lifi_dev,&(((uint8_t*)&lifi_dev->input_buf.crc_16)[transceiver_state->currentByte - lifi_dev->input_buf.len -1]) , transceiver_state->currentBit--);
            }
            if (transceiver_state->currentBit <= -1 ) {
                if (transceiver_state->currentByte == 0){
                    transceiver_state->current_frame_part = e_payload;
                } else if(transceiver_state->currentByte == lifi_dev->input_buf.len){
                    transceiver_state->current_frame_part = e_crc16;
                }
                transceiver_state->currentBit = 7;
                transceiver_state->currentByte++;
            }
            if (transceiver_state->currentByte == -1 + lifi_dev->input_buf.len + sizeof(lifi_dev->input_buf.len)+ sizeof(lifi_dev->input_buf.crc_16) && transceiver_state->currentBit == 0) {
                for (int byte = 0; byte < lifi_dev->input_buf.len; ++byte) {
                    printf("char: %c \n", lifi_dev->input_buf.payload[byte]);
                }
                puts("resetting");
                printf("crc %u\n",lifi_dev->input_buf.crc_16);
                transceiver_state->current_frame_part = e_first_receive;
                transceiver_state->currentByte = 0;
                transceiver_state->currentBit = 7;
            }
        }
        else {
            gpio_toggle(TIMING_ISSUE_PIN);
        }

    }
}


static int lifi_init(netdev_t *netdev)
{
    DEBUG("[LiFi] netdev_driver_t::init() called lifi_init fun\n");
    lifi_t *lifi_dev = (lifi_t *)netdev;
    const uint32_t frequency = 38000;
    const uint16_t resolution = 255;
    const pwm_t device = lifi_dev->params.output_pwm_device;
    const uint8_t channel = lifi_dev->params.output_pwm_device_channel;
    const pwm_mode_t mode = PWM_LEFT;

    const gpio_flank_t detectedFlanks = GPIO_BOTH;
    const gpio_t inputPin = lifi_dev->params.input_pin;

    bool error = false;
    uint8_t retval = 0;

    gpio_init(TIMING_ISSUE_PIN,GPIO_OUT);
    gpio_init(FULL_EDGES_DROP_PIN,GPIO_OUT);
    gpio_init(BIT_INTERPRETATION_PIN,GPIO_OUT);
    gpio_init(RECEIVE_INTERRUPT,GPIO_OUT);

    init_transceiver_state(lifi_dev);


    // Todo: any mutexes for ISR like in CC1101?
    if (0 == pwm_init(device, mode, frequency, resolution)) {
        DEBUG("[lifi] netdev_driver_t::init(): Failed to setup pwm device\n");
        retval = -EIO;
        error = true;
    }
    if (!error) {
        pwm_set(device, channel, 0);
    }

    if (!error) {
        if (gpio_init_int(inputPin, GPIO_IN, detectedFlanks, isr_callback_input_pin,
                          lifi_dev) != 0) {
            DEBUG("[lifi] netdev_driver_t::init(): Failed to init input pin gpio interrupt\n");
            retval = -EIO;
            error = true;
        }
    }

    if (!error) {
        DEBUG("[lifi] netdev_driver_t::init(): Success\n");
    }
    // todo update status of device
    // todo setup address?
    return retval;
}

static int lifi_recv(netdev_t *netdev, void *buf, size_t len, void *info)
{
    (void)netdev;
    (void)buf;
    (void)len;
    (void)info;
    return 1;
//    cc110x_t *dev = (cc110x_t *)netdev;
//
//    /* Call to cc110x_enter_rx_mode() will clear dev->buf.len, so back up it first */
//    int size = dev->buf.len;
//
//    cc110x_acquire(dev);
//
//    /* Copy RX info on last frame (if requested) */
//    if (info != NULL) {
//        *((cc1xxx_rx_info_t *)info) = dev->rx_info;
//    }
//
//    if (!buf) {
//        /* Get the size of the frame; if len > 0 then also drop the frame */
//        if (len > 0) {
//            /* Drop frame requested */
//            cc110x_enter_rx_mode(dev);
//        }
//        cc110x_release(dev);
//        return size;
//    }
//
//    if (len < (size_t)size) {
//        /* Drop frame and return -ENOBUFS */
//        cc110x_enter_rx_mode(dev);
//        cc110x_release(dev);
//        return -ENOBUFS;
//    }
//
//    memcpy(buf, dev->buf.data, (size_t)size);
//
//    cc110x_enter_rx_mode(dev);
//    cc110x_release(dev);
//    return size;
}
static int lifi_send(netdev_t *netdev, const iolist_t *iolist)
{
    lifi_t *lifi_dev = (lifi_t *)netdev;

    assert(netdev && iolist && (iolist->iol_len == sizeof(cc1xxx_l2hdr_t)));
    lifi_dev->output_buf.preamble = PREAMBLE;
    /* Copy data to send into frame buffer */
    size_t size = sizeof(cc1xxx_l2hdr_t);
    memcpy(lifi_dev->output_buf.payload, iolist->iol_base, sizeof(cc1xxx_l2hdr_t));

    for (const iolist_t *iol = iolist->iol_next; iol; iol = iol->iol_next) {
        if (iol->iol_len) {
            if(iol->iol_len == 8 || iol->iol_len == 4){
                // Todo remove after debugging
                return -1;
            }
            if (size + iol->iol_len > CC110X_MAX_FRAME_SIZE) {
                DEBUG("[liFi] netdev_driver_t::send(): Frame size of %uB "
                      "exceeds maximum supported size of %uB\n",
                      (unsigned)(size + iol->iol_len),
                      (unsigned)CC110X_MAX_FRAME_SIZE);
                return -1;
            }
            DEBUG("[lifi] netdev_driver_t::send(): iolist entry has size of: %u\n",iol->iol_len);
            memcpy(lifi_dev->output_buf.payload + size, iol->iol_base, iol->iol_len);
            size += iol->iol_len;
        }
    }

    lifi_dev->output_buf.len = (uint8_t)size;
    printf("Sending %u bytes\n", size);
    lifi_send_frame(lifi_dev);
    return 0;
}


static int lifi_get(netdev_t *netdev, netopt_t opt,
                    void *val, size_t max_len)
{
    DEBUG("Entering lifi_get function\n");
    lifi_t *dev = (lifi_t *)netdev;

    (void)max_len;  /* only used in assert() */
    switch (opt) {
    case NETOPT_DEVICE_TYPE:
        assert(max_len == sizeof(uint16_t));
        *((uint16_t *)val) = NETDEV_TYPE_CC110X;
        return sizeof(uint16_t);
        break;
    case NETOPT_MAX_PDU_SIZE:
        assert(max_len == sizeof(uint16_t));
        *((uint16_t *)val) = CC110X_MAX_FRAME_SIZE - sizeof(cc1xxx_l2hdr_t);
        return sizeof(uint16_t);
    case NETOPT_ADDRESS:
        assert(max_len >= CC1XXX_ADDR_SIZE);
        *((uint8_t *)val) = dev->addr;
        return CC1XXX_ADDR_SIZE;
    default:
        break;
    }

    return -ENOTSUP;
//    cc110x_t *dev = (cc110x_t *)netdev;
//
//    (void)max_len;  /* only used in assert() */
//    switch (opt) {
//    case NETOPT_DEVICE_TYPE:
//        assert(max_len == sizeof(uint16_t));
//        *((uint16_t *)val) = NETDEV_TYPE_CC110X;
//        return sizeof(uint16_t);
//    case NETOPT_PROTO:
//        assert(max_len == sizeof(gnrc_nettype_t));
//        *((gnrc_nettype_t *)val) = CC110X_DEFAULT_PROTOCOL;
//        return sizeof(gnrc_nettype_t);
//    case NETOPT_MAX_PDU_SIZE:
//        assert(max_len == sizeof(uint16_t));
//        *((uint16_t *)val) = CC110X_MAX_FRAME_SIZE - sizeof(cc1xxx_l2hdr_t);
//        return sizeof(uint16_t);
//    case NETOPT_ADDR_LEN:
//    /* falls through */
//    case NETOPT_SRC_LEN:
//        assert(max_len == sizeof(uint16_t));
//        *((uint16_t *)val) = CC1XXX_ADDR_SIZE;
//        return sizeof(uint16_t);
//    case NETOPT_ADDRESS:
//        assert(max_len >= CC1XXX_ADDR_SIZE);
//        *((uint8_t *)val) = dev->addr;
//        return CC1XXX_ADDR_SIZE;
//    case NETOPT_CHANNEL:
//        assert(max_len == sizeof(uint16_t));
//        *((uint16_t *)val) = dev->channel;
//        return sizeof(uint16_t);
//    case NETOPT_TX_POWER:
//        assert(max_len == sizeof(uint16_t));
//        *((uint16_t *)val) = dbm_from_tx_power[dev->tx_power];
//        return sizeof(uint16_t);
//    case NETOPT_PROMISCUOUSMODE:
//        assert(max_len == sizeof(netopt_enable_t));
//        return cc110x_get_promiscuous_mode(dev, val);
//    case NETOPT_STATE:
//        assert(max_len == sizeof(netopt_state_t));
//        {
//            netopt_state_t *state = val;
//            switch (dev->state) {
//            case CC110X_STATE_RECEIVING:
//            case CC110X_STATE_FRAME_READY:
//            case CC110X_STATE_RXFIFO_OVERFLOW:
//                *state = NETOPT_STATE_RX;
//                break;
//            case CC110X_STATE_IDLE:
//                *state = NETOPT_STATE_STANDBY;
//                break;
//            case CC110X_STATE_OFF:
//                *state = NETOPT_STATE_SLEEP;
//                break;
//            case CC110X_STATE_TX_MODE:
//            case CC110X_STATE_TX_COMPLETING:
//            case CC110X_STATE_TXFIFO_UNDERFLOW:
//                *state = NETOPT_STATE_TX;
//                break;
//            case CC110X_STATE_RX_MODE:
//                *state = NETOPT_STATE_IDLE;
//                break;
//            default:
//                *state = NETOPT_STATE_RESET;
//                break;
//            }
//        }
//        break;
//    default:
//        break;
//    }
//    return -ENOTSUP;
}

/**
 * @brief   Set the given address as the device's layer 2 address
 *
 * @param   dev     Device descripter of the transceiver
 * @param   addr    Address to set
 */
static int lifi_set_addr(lifi_t *dev, uint8_t addr)
{
    (void)dev;
    (void)addr;
//    cc110x_acquire(dev);
//
//    dev->addr = addr;
//    cc110x_write(dev, CC110X_REG_ADDR, addr);
//    cc110x_release(dev);
    return 1;
}

///**
// * @brief   Enables/disables the CC110x's address filter
// * @param   dev     Transceiver to turn promiscuous mode on/off
// * @param   enable  Whether to enable or disable promiscuous mode
// *
// * @return  Returns the size of @ref netopt_enable_t to confirm with the API
// *          in @ref netdev_driver_t::set
// */
//static int cc110x_set_promiscuous_mode(cc110x_t *dev, netopt_enable_t enable)
//{
//    cc110x_acquire(dev);
//
//    uint8_t pktctrl1 = CC110X_PKTCTRL1_VALUE;
//    if (enable == NETOPT_ENABLE) {
//        pktctrl1 |= CC110X_PKTCTRL1_ADDR_ALL;
//    }
//    else {
//        pktctrl1 |= CC110X_PKTCTRL1_ADDR_MATCH;
//    }
//    cc110x_write(dev, CC110X_REG_PKTCTRL1, pktctrl1);
//    cc110x_release(dev);
//    return sizeof(netopt_enable_t);
//}

static int lifi_set(netdev_t *netdev, netopt_t opt,
                    const void *val, size_t len)
{
    (void)len;
    (void)netdev;
    (void)opt;
    (void)val;
    return 1;
//    cc110x_t *dev = (cc110x_t *)netdev;
//
//    switch (opt) {
//    case NETOPT_ADDRESS:
//        assert(len == CC1XXX_ADDR_SIZE);
//        return cc110x_set_addr(dev, *((uint8_t *)val));
//    case NETOPT_CHANNEL:
//    {
//        assert(len == sizeof(uint16_t));
//        int retval;
//        uint16_t channel = *((uint16_t *)val);
//        if (channel >= CC110X_MAX_CHANNELS) {
//            return -EINVAL;
//        }
//        if ((retval = cc110x_set_channel(dev, (uint8_t)channel))) {
//            return retval;
//        }
//    }
//        return sizeof(uint16_t);
//    case NETOPT_TX_POWER:
//    {
//        assert(len == sizeof(int16_t));
//        int16_t dbm = *((int16_t *)val);
//        cc110x_tx_power_t power = CC110X_TX_POWER_MINUS_30_DBM;
//        for ( ; power < CC110X_TX_POWER_PLUS_10_DBM; power++) {
//            if ((int16_t)tx_power_from_dbm[power] >= dbm) {
//                break;
//            }
//        }
//        if (cc110x_set_tx_power(dev, power)) {
//            return -EINVAL;
//        }
//    }
//        return sizeof(uint16_t);
//    case NETOPT_PROMISCUOUSMODE:
//        assert(len == sizeof(netopt_enable_t));
//        return cc110x_set_promiscuous_mode(dev, *((const netopt_enable_t *)val));
//    case NETOPT_STATE:
//        assert(len == sizeof(netopt_state_t));
//        switch (*((netopt_state_t *)val)) {
//        case NETOPT_STATE_RESET:
//        case NETOPT_STATE_IDLE:
//            cc110x_wakeup(dev);
//            return sizeof(netopt_state_t);
//        case NETOPT_STATE_OFF:
//        case NETOPT_STATE_SLEEP:
//            cc110x_sleep(dev);
//            return sizeof(netopt_state_t);
//        default:
//            return -ENOTSUP;
//        }
//        break;
//    default:
//        return -ENOTSUP;
//    }
}
