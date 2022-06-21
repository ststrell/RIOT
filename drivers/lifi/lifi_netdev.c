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
#include <log.h>
#include <lifi_params.h>
#include "checksum/crc16_ccitt.h"

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
#define COOL_DOWN_RECEIVE_SEND_US 400

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

// general todo
// todo implement a generic reset function
// todo implement states
// todo think about thread/state/ISR communication


void isr_callback_input_pin(void *_dev)
{
    gpio_toggle(RECEIVE_INTERRUPT);
    lifi_t *lifi_dev = (lifi_t *)_dev;

    /* skip the pwm turn_off interrupt as it does not mean anything */
    if (lifi_dev->transceiver_state.current_frame_part == e_first_receive && xtimer_diff(xtimer_now(),lifi_dev->transceiver_state.lastReceive) <
                                                      xtimer_ticks_from_usec(COOL_DOWN_RECEIVE_SEND_US))
    {
        gpio_toggle(MULTI_PURPOSE_DEBUG);
    }
    /* only start receiving if already not sending, receiving our own message does not make sense */
    else if (lifi_dev->state == LIFI_STATE_IDLE || lifi_dev->state == LIFI_STATE_RECEIVING) {
        lifi_dev->state = LIFI_STATE_RECEIVING;
        lifi_transceiver_state_t *transceiver_state = &lifi_dev->transceiver_state;

        if (check_receive_timeout(lifi_dev)){
            transceiver_state->current_frame_part = e_first_receive;
            transceiver_state->currentByte = 0;
            transceiver_state->currentBit = 7;
            transceiver_state->high_bit_counter = 7;
        }

        // Sync to sender frequency
        if (transceiver_state->current_frame_part == e_first_receive
            || transceiver_state->current_frame_part == e_preamble) {
            lifi_preamble_sync(lifi_dev);
        }
            // receive data
        else {
            // first call here means, that preamble is done. Preamble finished with half period
            // next interrupt can happen either at next full period or next half period, depends on the first bit to transfer

            // calculate ticks diff between now and last half period receive, used to check if edge has to be dropped
            xtimer_ticks32_t timediff = xtimer_diff(xtimer_now(), transceiver_state->lastHalfEdge);

            if (timediff >= transceiver_state->min_tolerated_half_clock &&
                timediff <= transceiver_state->max_tolerated_half_clock) {
                // Drop edges on full period
                gpio_toggle(FULL_EDGES_DROP_PIN);
            }
                // edge seems to be on half period
            else if (timediff >= transceiver_state->min_tolerated_half_clock * 2 &&
                     timediff <= 2 * transceiver_state->max_tolerated_half_clock) {
                transceiver_state->lastHalfEdge = xtimer_now();
                // edge is in expected half period timing

                // check current frame part and currentByte to decide where to store the bit
                if (transceiver_state->current_frame_part == e_len) {
                    read_store_bit(lifi_dev, &lifi_dev->input_buf.len, transceiver_state->currentBit);
                } else if (transceiver_state->current_frame_part == e_layer2_header) {
                    if (transceiver_state->currentByte == 1){
                        read_store_bit(lifi_dev, (uint8_t*) &lifi_dev->input_buf.layer2_hdr.dest_addr, transceiver_state->currentBit);
                    } else {
                        read_store_bit(lifi_dev, (uint8_t*) &lifi_dev->input_buf.layer2_hdr.src_addr, transceiver_state->currentBit);
                    }
                } else if (transceiver_state->current_frame_part == e_payload) {
                    read_store_bit(lifi_dev, &lifi_dev->input_buf.payload[transceiver_state->currentByte - 3],
                                   transceiver_state->currentBit);
                } else {
                    read_store_bit(lifi_dev,
                                   &(((uint8_t *) &lifi_dev->input_buf.crc_16)
                                   [transceiver_state->currentByte
                                   - lifi_dev->input_buf.len - 1
                                   - sizeof(lifi_l2hdr_t)])
                                   , transceiver_state->currentBit);
                }
                if (transceiver_state->currentBit <= -1) {
                    if (transceiver_state->current_frame_part == e_len) {
                        transceiver_state->current_frame_part = e_layer2_header;
                    }
                    if (transceiver_state->currentByte == sizeof(lifi_dev->input_buf.layer2_hdr)) {
                        transceiver_state->current_frame_part = e_payload;
                    } else if (transceiver_state->currentByte == lifi_dev->input_buf.len
                                + sizeof(lifi_dev->input_buf.layer2_hdr)) {
                        transceiver_state->current_frame_part = e_crc16;
                    }
                    transceiver_state->currentBit = 7;
                    transceiver_state->currentByte++;
                    if (transceiver_state->currentByte ==
                    sizeof(lifi_dev->input_buf.len)
                    + sizeof(lifi_dev->input_buf.layer2_hdr)
                    + lifi_dev->input_buf.len
                    + sizeof(lifi_dev->input_buf.crc_16)) {
                         /* Finish reception and save last receive to be able to skip pwm off interrupt */
                        lifi_dev->transceiver_state.lastReceive = xtimer_now();
    //                for (int byte = 0; byte < lifi_dev->input_buf.len; ++byte) {
    ////                    printf("char: %c \n", lifi_dev->input_buf.payload[byte]);
    //                    printf("byte: %u \n", lifi_dev->input_buf.payload[byte]);
    //                }
    //                    puts("resetting");
                        uint16_t crc_16 = crc16_ccitt_calc(lifi_dev->input_buf.payload, lifi_dev->input_buf.len);
                        lifi_dev->input_buf.crc_16 = ntohs(lifi_dev->input_buf.crc_16);

                        // reset transceiver
                        transceiver_state->current_frame_part = e_first_receive;
                        transceiver_state->currentByte = 0;
                        transceiver_state->currentBit = 7;
                        lifi_dev->state = LIFI_STATE_IDLE;

                        if (crc_16 != lifi_dev->input_buf.crc_16) {
    //                        puts("crc16 did not match!");
    //                        for (int pos = 0; pos < lifi_dev->input_buf.len; ++pos) {
    //                            printf("%u ", lifi_dev->input_buf.payload[pos]);
    //                        }
    //                        printf("\nCRC calculated: %u , CRC ingoing: %u , len: %u \n",crc_16, lifi_dev->input_buf.crc_16, lifi_dev->input_buf.len);
                            //todo errorhandling
                        } else {
                            netdev_trigger_event_isr(&lifi_dev->netdev);
                        }
                    }
                }
            } else {
                /* timing unexpected - reset and save last receive to be able to skip pwm off interrupt */
                gpio_toggle(TIMING_ISSUE_PIN);
                init_transceiver_state(lifi_dev);
                lifi_dev->transceiver_state.lastReceive = xtimer_now();
                gpio_toggle(MULTI_PURPOSE_DEBUG);
            }

        }
    }
}


static int lifi_init(netdev_t *netdev)
{
    DEBUG("[LiFi] netdev_driver_t::init() called lifi_init fun\n");
    lifi_t *lifi_dev = (lifi_t *)netdev;
    lifi_dev->params.pwm_frequency = DEFAULT_LIFI_PWM_FREQUENCY;
    lifi_dev->params.pwm_resolution = DEFAULT_LIFI_PWM_RESOLUTION;
    lifi_dev->params.pwm_high_gain = DEFAULT_LIFI_PWM_RESOLUTION/2;
    lifi_dev->params.pwm_low_gain = 0;
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
    gpio_init(MULTI_PURPOSE_DEBUG,GPIO_OUT);

    init_transceiver_state(lifi_dev);
    lifi_dev->baud = 50;

    /* Setup the layer 2 address, but do not accept CC1XXX_BCAST_ADDR (which
     * has the value 0x00 and is used for broadcast)
     */
    lifi_dev->addr = LIFI_BCAST_ADDR+1;
    lifi_eui_get(&lifi_dev->netdev, &lifi_dev->addr);
    assert(lifi_dev->addr != LIFI_BCAST_ADDR);

    // Todo: any mutexes for ISR like in CC1101?
    if (0 == pwm_init(device, mode, lifi_dev->params.pwm_frequency,lifi_dev->params.pwm_resolution)) {
        LOG_ERROR("[lifi] netdev_driver_t::init(): Failed to setup pwm device\n");
        retval = -EIO;
        error = true;
    }
    if (!error) {
        pwm_set(device, channel, 0);
    }

    if (!error) {
        if (gpio_init_int(inputPin, GPIO_IN, detectedFlanks, isr_callback_input_pin,
                          lifi_dev) != 0) {
            LOG_ERROR("[lifi] netdev_driver_t::init(): Failed to init input pin gpio interrupt\n");
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
    // todo: test this
    (void)info; // no receive information available
    lifi_t *dev = (lifi_t *)netdev;

    /* Call to cc110x_enter_rx_mode() will clear dev->buf.len, so back up it first */
    int size = dev->input_buf.len + sizeof(lifi_l2hdr_t);

    if (!buf) {
        /* Get the size of the frame; if len > 0 then also drop the frame */
        if (len > 0) {
            /* Drop frame requested */
            dev->input_buf.len = 0;
        }
        return size;
    }

    if (len < (size_t)size) {
        /* Drop frame and return -ENOBUFS */
        dev->input_buf.len = 0;
        return -ENOBUFS;
    }

    memcpy(buf, (void*) &dev->input_buf.layer2_hdr, (size_t)size);

    dev->input_buf.len = 0; /* frame collected, delete it */
    return size;
}

static int lifi_send(netdev_t *netdev, const iolist_t *iolist)
{
    lifi_t *lifi_dev = (lifi_t *)netdev;

    DEBUG("[LiFi] netdev_driver_t::send(): Called with state %i\n", (int)lifi_dev->state);

    assert(netdev && iolist && (iolist->iol_len == sizeof(lifi_l2hdr_t)));

    /* Check for receive timeout, needs to be done here, as timeout is only reset in GPIO interrupt,
     * which does not trigger if connection is lost. */
    if(check_receive_timeout(lifi_dev)){
        lifi_dev->state = LIFI_STATE_IDLE;
    }
    switch (lifi_dev->state) {
        case LIFI_STATE_RECEIVING:
            printf("[LiFi] netdev_driver_t::send(): Refusing to send while "
                  "receiving a frame\n");
            return -EBUSY;
        case LIFI_STATE_TRANSMITTING:
            printf("[LiFi] netdev_driver_t::send(): Refusing to send while "
                  "transmitting a frame\n");
            return -EBUSY;
        case LIFI_STATE_IDLE:
            /* able to send, so let's go ahead */
            break;
        default:
            printf("[LiFi] netdev_driver_t::send(): Driver state %i prevents "
                  "sending\n", (int)lifi_dev->state);
            return -1;
    }

    lifi_dev->state = LIFI_STATE_TRANSMITTING;
    lifi_dev->output_buf.preamble = PREAMBLE;
    /* Copy data to send into frame buffer */
    size_t payload_size = 0;
    memcpy(&lifi_dev->output_buf.layer2_hdr, iolist->iol_base, sizeof(lifi_l2hdr_t));

    for (iolist_t *iol = iolist->iol_next; iol; iol = iol->iol_next) {
        if (iol->iol_len) {
            DEBUG("Packet Type: %i\n",((gnrc_pktsnip_t*) iol)->type);
//            if(iol->iol_len == 8 || iol->iol_len == 4){
//                // Todo remove after debugging
//                lifi_dev->state = LIFI_STATE_IDLE;
//                return -1;
//            }
            if (payload_size + iol->iol_len > LIFI_MAX_FRAME_SIZE) {
                DEBUG("[liFi] netdev_driver_t::send(): Frame size of %uB "
                      "exceeds maximum supported size of %uB\n",
                      (unsigned)(payload_size + iol->iol_len),
                      (unsigned)LIFI_MAX_FRAME_SIZE);
                return -1;
            }
            DEBUG("[lifi] netdev_driver_t::send(): iolist entry has size of: %u\n",iol->iol_len);
            memcpy(lifi_dev->output_buf.payload + payload_size, iol->iol_base, iol->iol_len);
            payload_size += iol->iol_len;
        }
    }

    lifi_dev->output_buf.len = (uint8_t)payload_size;
    DEBUG("Sending %u bytes\n", payload_size);
    /* wait with sending until the pwm off interrupt settled */
    while (xtimer_diff(xtimer_now(),lifi_dev->transceiver_state.lastReceive) < xtimer_ticks_from_usec(400)){
        xtimer_usleep(50);
    }

    lifi_send_frame(lifi_dev);
    lifi_dev->state = LIFI_STATE_IDLE;
    return 0;
}


static int lifi_get(netdev_t *netdev, netopt_t opt,
                    void *val, size_t max_len)
{
    // todo implement more options
    DEBUG("Entering lifi_get function\n");
    lifi_t *dev = (lifi_t *)netdev;

    (void)max_len;  /* only used in assert() */
    switch (opt) {
    case NETOPT_DEVICE_TYPE:
        assert(max_len == sizeof(uint16_t));
        *((uint16_t *)val) = NETDEV_TYPE_LIFI;
        return sizeof(uint16_t);
        break;
    case NETOPT_MAX_PDU_SIZE:
        assert(max_len == sizeof(uint16_t));
        *((uint16_t *)val) = LIFI_MAX_FRAME_SIZE - sizeof(lifi_l2hdr_t);
        return sizeof(uint16_t);
    case NETOPT_ADDRESS:
        assert(max_len >= LIFI_ADDR_SIZE);
        *((uint8_t *)val) = dev->addr;
        return LIFI_ADDR_SIZE;
    case NETOPT_PROTO:
         assert(max_len == sizeof(gnrc_nettype_t));
         *((gnrc_nettype_t *)val) = LIFI_DEFAULT_PROTOCOL;
         return sizeof(gnrc_nettype_t);
    default:
        break;
    }

    return -ENOTSUP;
}


static int lifi_set(netdev_t *netdev, netopt_t opt,
                    const void *val, size_t len)
{
    // @todo implement setter according to getters
    (void)len;

    (void)opt;
    lifi_t *lifi_dev = (lifi_t *)netdev;

    switch (opt) {
    case NETOPT_ADDRESS:
        assert(len == LIFI_ADDR_SIZE);
        lifi_dev->addr = *(uint8_t *)val;
        return 0;
//    @todo think about implementing an "promiscuous" address filter
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
    default:
        return -ENOTSUP;
    }
}
