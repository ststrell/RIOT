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
 * @brief       Functions to manage sending/receiving frames with the CC110x
 *
 * @author      Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 * @}
 */

#include <byteorder.h>
#include "xtimer.h"
#include "lifi.h"
#include "lifi_rx_tx.h"
#include "checksum/crc16_ccitt.h"
#include "lifi_params.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#define BAUD_TO_US_PERIOD(baud) (1000000/(baud*8))
#define TIMEOUT_TICKS xtimer_ticks_from_usec(10000)

/* Use NETDEV_EVENT_ISR to indicate that no event needs to be passed to upper
 * layer at end of ISR, as ISR will never need this event
 */
#define NETDEV_NO_EVENT NETDEV_EVENT_ISR

void lifi_isr(netdev_t *netdev)
{
    lifi_t* lifi_dev = (lifi_t* )netdev;
    /* Pass event to upper layer, if needed */
    lifi_dev->netdev.event_callback(&lifi_dev->netdev, NETDEV_EVENT_RX_COMPLETE);
}

static void send_single_edge(lifi_t* lifi_dev, uint8_t gain){
    uint8_t channel = lifi_dev->params.output_pwm_device_channel;
    pwm_t device = lifi_dev->params.output_pwm_device;
    uint16_t baud = lifi_dev->baud;

    xtimer_usleep(BAUD_TO_US_PERIOD(baud)/2);
    pwm_set(device, channel, gain);
    xtimer_usleep(BAUD_TO_US_PERIOD(baud)/2);
}
static void send_double_edge(lifi_t* lifi_dev, uint8_t gain){
    uint8_t channel = lifi_dev->params.output_pwm_device_channel;
    pwm_t device = lifi_dev->params.output_pwm_device;
    uint16_t baud = lifi_dev->baud;
    uint16_t high_gain = lifi_dev->params.pwm_high_gain;
    uint16_t low_gain = lifi_dev->params.pwm_low_gain;

    pwm_set(device, channel, gain == high_gain ? low_gain : high_gain);
    xtimer_usleep(BAUD_TO_US_PERIOD(baud)/2);
    pwm_set(device, channel, gain);
    xtimer_usleep(BAUD_TO_US_PERIOD(baud)/2);
}

static void send_high_bit(lifi_t* lifi_dev){
    lifi_transceiver_state_t* transceiver_state = &lifi_dev->transceiver_state;
    uint16_t high_gain = lifi_dev->params.pwm_high_gain;

    gpio_set(DATA_SENDER_PIN);
    if (transceiver_state->previousStateHigh) {
        send_double_edge(lifi_dev, high_gain);
    }
    else {
        send_single_edge(lifi_dev, high_gain);
    }
    transceiver_state->previousStateHigh = true;
}

static void send_low_bit(lifi_t* lifi_dev){
    lifi_transceiver_state_t* transceiver_state = &lifi_dev->transceiver_state;
    uint16_t low_gain = lifi_dev->params.pwm_low_gain;

    gpio_clear(DATA_SENDER_PIN);
    if (transceiver_state->previousStateHigh) {
        send_single_edge(lifi_dev, low_gain);
    }
    else {
        send_double_edge(lifi_dev, low_gain);
    }
    transceiver_state->previousStateHigh = false;
}


static void lifi_send_bits(lifi_t* lifi_dev,uint16_t num_bytes, uint8_t* bytes){
    lifi_transceiver_state_t* transceiver_state = &lifi_dev->transceiver_state;

    for(uint16_t byte = 0; byte < num_bytes; byte++){
        for (int8_t bit = 7; bit >= 0; bit--) {
            gpio_toggle(CLOCK_PIN);
            // IEEE 802.3 rising edge for logic 1
            if ((bytes[byte] >> bit) & 0b1) {
                gpio_set(DATA_SENDER_PIN);
                if(transceiver_state->current_frame_part != e_preamble && transceiver_state->high_bit_counter >= 7){
                    // send a stuffing zero bit, to avoid preamble duplication
                    send_low_bit(lifi_dev);
                    transceiver_state->high_bit_counter = 0;
                }
                transceiver_state->high_bit_counter++;
                send_high_bit(lifi_dev);
            }
            else {
                // IEEE 802.3 falling edge for logic 0
                gpio_clear(DATA_SENDER_PIN);
                if(transceiver_state->current_frame_part != e_preamble && transceiver_state->high_bit_counter >= 7){
                    // sent 7 high bits, need so send a stuffing zero bit, even though the next bit would be zero anyway.
                    // Receiver does not know that when receiving
                    send_low_bit(lifi_dev);
                }
                send_low_bit(lifi_dev);
                transceiver_state->high_bit_counter = 0;
            }
            lifi_dev->output_buf.pos++;
        }
    }
}

void lifi_send_frame(lifi_t* lifi_dev){
    const uint8_t channel = lifi_dev->params.output_pwm_device_channel;
    const pwm_t device = lifi_dev->params.output_pwm_device;
    lifi_framebuf_t * framebuf = &lifi_dev->output_buf;
    lifi_transceiver_state_t* transceiver_state = &lifi_dev->transceiver_state;

    gpio_init(DATA_SENDER_PIN, GPIO_OUT);
    gpio_init(CLOCK_PIN, GPIO_OUT);

    framebuf->crc_16 = crc16_ccitt_calc(framebuf->payload,framebuf->len);
    DEBUG("[LiFi] lifi_rx_tx:lifi_send_frame: CRC outgoing: %u , len: %u \n", framebuf->crc_16, framebuf->len);

    init_transceiver_state(lifi_dev);

    transceiver_state->current_frame_part = e_preamble;
    lifi_send_bits(lifi_dev, sizeof(framebuf->preamble),&framebuf->preamble);
    transceiver_state->current_frame_part = e_len;
    lifi_send_bits(lifi_dev, sizeof(framebuf->len),&framebuf->len);
    transceiver_state->current_frame_part = e_layer2_header;
    lifi_send_bits(lifi_dev, sizeof(framebuf->layer2_hdr), (uint8_t *) &framebuf->layer2_hdr);
    transceiver_state->current_frame_part = e_payload;
    lifi_send_bits(lifi_dev, framebuf->len,framebuf->payload);
    transceiver_state->current_frame_part = e_crc16;
    framebuf->crc_16 = htons(framebuf->crc_16);
    lifi_send_bits(lifi_dev, sizeof(framebuf->crc_16), (uint8_t *) &framebuf->crc_16);

    bool lastStateHigh = lifi_dev->transceiver_state.previousStateHigh;
    init_transceiver_state(lifi_dev);

    if (lastStateHigh){
        lifi_dev->transceiver_state.lastReceive = xtimer_now();
        gpio_toggle(MULTI_PURPOSE_DEBUG);
    }
    framebuf->pos = 0;
    pwm_set(device, channel, 0);     // turn off pwm
    lifi_dev->transceiver_state.current_frame_part = e_first_receive;
    gpio_clear(DATA_SENDER_PIN);
}
void read_store_bit(lifi_t* lifi_dev, uint8_t* storage_byte, uint8_t bit_to_read)
{
    // todo universal high/low edge interpretation depending on transceiver type
    if (gpio_read(lifi_dev->params.input_pin) == 0) {
        // got high bit
         gpio_set(BIT_INTERPRETATION_PIN);
         lifi_dev->transceiver_state.high_bit_counter++;
         lifi_dev->transceiver_state.currentBit--;
         *storage_byte |= 1 << bit_to_read;
    }
    else {
        // got low bit

        if(lifi_dev->transceiver_state.high_bit_counter >= 7){
            // discard stuffing bit
        } else {
            gpio_clear(BIT_INTERPRETATION_PIN);
            lifi_dev->transceiver_state.currentBit--;
            *storage_byte &= ~(1 << bit_to_read);
        }

        lifi_dev->transceiver_state.high_bit_counter = 0;
    }
}

void lifi_preamble_sync(lifi_t* lifi_dev){
    const uint8_t expectedPreambleFlanks = 15; // we end at T/2
    static uint8_t preambleFlankCounter = 0;
    preambleFlankCounter++;
    lifi_transceiver_state_t* transceiver_state = &lifi_dev->transceiver_state;

    if (transceiver_state->current_frame_part == e_first_receive) {
        transceiver_state->lastReceive = xtimer_now();
        transceiver_state->meanHalfClockTicks = 0;
        transceiver_state->min_tolerated_half_clock = 0;
        transceiver_state->max_tolerated_half_clock = 0;
        preambleFlankCounter = 0;
        transceiver_state->current_frame_part = e_preamble;
    }
    else {
        xtimer_ticks32_t timediff = xtimer_diff(xtimer_now(), transceiver_state->lastReceive);
        transceiver_state->meanHalfClockTicks += (float)timediff / (float)expectedPreambleFlanks;
        transceiver_state->lastReceive = xtimer_now();

        if (preambleFlankCounter >= expectedPreambleFlanks -1) {
            transceiver_state->lastHalfEdge = xtimer_now();
            transceiver_state->current_frame_part = e_len;
            // todo move allowed range to settable variables
            transceiver_state->min_tolerated_half_clock = (uint32_t)(0.6 * transceiver_state->meanHalfClockTicks);
            transceiver_state->max_tolerated_half_clock = (uint32_t)(1.4 * transceiver_state->meanHalfClockTicks);
            transceiver_state->previousStateHigh = gpio_read(lifi_dev->params.input_pin) != 0;
            transceiver_state->high_bit_counter = 7;
        }
    }
}

void init_transceiver_state(lifi_t *lifi_dev) {
    lifi_dev->transceiver_state.min_tolerated_half_clock = 0;
    lifi_dev->transceiver_state.max_tolerated_half_clock = 0;
    lifi_dev->transceiver_state.current_frame_part = e_first_receive;
    lifi_dev->transceiver_state.previousStateHigh = false;
    lifi_dev->transceiver_state.meanHalfClockTicks = 0;
    lifi_dev->transceiver_state.lastReceive = 0 ;
    lifi_dev->transceiver_state.lastHalfEdge = 0;
    lifi_dev->transceiver_state.currentBit = 7;
    lifi_dev->transceiver_state.currentByte = 0;
    lifi_dev->transceiver_state.high_bit_counter = 0;
}

bool check_receive_timeout(lifi_t *lifi_dev) {
    bool timeout = false;
    lifi_transceiver_state_t *transceiver_state = &lifi_dev->transceiver_state;

    if (transceiver_state->current_frame_part == e_preamble &&
        xtimer_diff(xtimer_now(), transceiver_state->lastReceive) > TIMEOUT_TICKS) {
        // check if we have a timeout on lastReceive in preamble frame part -> reset
        timeout = true;
    }
    else if ((transceiver_state->current_frame_part == e_len || transceiver_state->current_frame_part == e_payload ||
         transceiver_state->current_frame_part == e_crc16)
        && xtimer_diff(xtimer_now(), transceiver_state->lastHalfEdge) > TIMEOUT_TICKS) {
        // check if we have a timeout in e_len/e_payload/e_crc16 mode -> reset
        timeout = true;
    }
    return timeout;
}
