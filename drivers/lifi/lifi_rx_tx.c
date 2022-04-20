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

#define ENABLE_DEBUG 0
#include "debug.h"

#define RESOLUTION 255
#define HIGH_GAIN (RESOLUTION / 2)
#define LOW_GAIN 0
#define SLEEP_TIME_US 1000

/* Use NETDEV_EVENT_ISR to indicate that no event needs to be passed to upper
 * layer at end of ISR, as ISR will never need this event
 */
#define NETDEV_NO_EVENT NETDEV_EVENT_ISR

void lifi_isr(netdev_t *netdev)
{
    (void) netdev;
//    cc110x_t *dev = (cc110x_t *)netdev;
//    /* We don't want to create events while device descriptor is acquired, to
//     * prevent a dead lock. (Currently e.g. on NETDEV_EVENT_RX_COMPLETE the
//     * upper layer will immediately call netdev_driver_t::recv(), which in
//     * turn wants to operate on the device descriptor. We could rely on this
//     * behaviour by skipping cc110x_acquire() there, but the driver would break
//     * when upper layer behaviour is changed. By moving the event notification
//     * at the end of the ISR (end after cc110x_release()), the driver becomes
//     * agnostic to which behaviour the upper layer implements.)
//     */
//    netdev_event_t post_isr_event = NETDEV_NO_EVENT;
//
//    cc110x_acquire(dev);
//
//    /* Disable IRQs in a coarse manner, instead of doing so any time the
//     * IOCFGx configuration registers are changed. (This should be less
//     * bug prone.)
//     */
//    gpio_irq_disable(dev->params.gdo0);
//    gpio_irq_disable(dev->params.gdo2);
//
//    switch (dev->state) {
//        case CC110X_STATE_RX_MODE:
//            if (gpio_read(dev->params.gdo0) || gpio_read(dev->params.gdo2)) {
//                dev->state = CC110X_STATE_RECEIVING;
//                dev->buf.pos = dev->buf.len = 0;
//            }
//            break;
//        case CC110X_STATE_RECEIVING:
//            post_isr_event = cc110x_rx_continue(dev);
//            break;
//        case CC110X_STATE_TX_MODE:
//            post_isr_event = cc110x_tx_continue(dev);
//            break;
//        case CC110X_STATE_TX_COMPLETING:
//            post_isr_event = cc110x_tx_done(dev);
//            break;
//        default:
//            DEBUG("[cc110x] ISR: CRITICAL ERROR: No interrupt expected "
//                  "for current state\n");
//            /* Go back to RX and pray that solved the problem */
//            cc110x_enter_rx_mode(dev);
//    }
//
//    /* Re-enable IRQs again, unless device state */
//    gpio_irq_enable(dev->params.gdo0);
//    gpio_irq_enable(dev->params.gdo2);
//    cc110x_release(dev);
//    /* Pass event to uper layer, if needed */
//    if (post_isr_event != NETDEV_NO_EVENT) {
//        dev->netdev.event_callback(&dev->netdev, post_isr_event);
//    }
}

static void send_single_edge(pwm_t device, uint8_t channel,uint16_t sleepTimeUs, uint8_t gain){
    xtimer_usleep(sleepTimeUs / 2);
    pwm_set(device, channel, gain);
    xtimer_usleep(sleepTimeUs / 2);
}
static void send_double_edge(pwm_t device, uint8_t channel,uint16_t sleepTimeUs, uint8_t gain){
    pwm_set(device, channel, gain == HIGH_GAIN ? LOW_GAIN : HIGH_GAIN);
    xtimer_usleep(sleepTimeUs / 2);
    pwm_set(device, channel, gain == HIGH_GAIN ? HIGH_GAIN :LOW_GAIN);
    xtimer_usleep(sleepTimeUs / 2);
}

static void send_high_bit(lifi_t* lifi_dev){
    uint8_t channel = lifi_dev->params.output_pwm_device_channel;
    pwm_t device = lifi_dev->params.output_pwm_device;
    lifi_transceiver_state_t* transceiver_state = &lifi_dev->transceiver_state;

    gpio_set(DATA_SENDER_PIN);
    if (transceiver_state->previousStateHigh) {
        send_double_edge(device, channel, SLEEP_TIME_US, HIGH_GAIN);
    }
    else {
        send_single_edge(device, channel, SLEEP_TIME_US, HIGH_GAIN);
    }
    transceiver_state->previousStateHigh = true;
}

static void send_low_bit(lifi_t* lifi_dev){
    uint8_t channel = lifi_dev->params.output_pwm_device_channel;
    pwm_t device = lifi_dev->params.output_pwm_device;
    lifi_transceiver_state_t* transceiver_state = &lifi_dev->transceiver_state;

    gpio_clear(DATA_SENDER_PIN);
    if (transceiver_state->previousStateHigh) {
        send_single_edge(device, channel, SLEEP_TIME_US, LOW_GAIN);
    }
    else {
        send_double_edge(device, channel, SLEEP_TIME_US, LOW_GAIN);
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

    gpio_init(DATA_SENDER_PIN, GPIO_OUT);
    gpio_init(CLOCK_PIN, GPIO_OUT);

    framebuf->crc_16 = crc16_ccitt_calc(framebuf->payload,framebuf->len);
    DEBUG("[LiFi] lifi_rx_tx:lifi_send_frame: CRC outgoing: %u , len: %u \n", framebuf->crc_16, framebuf->len);

    init_transceiver_state(lifi_dev);

    lifi_dev->transceiver_state.current_frame_part = e_preamble;
    lifi_send_bits(lifi_dev, sizeof(framebuf->preamble),&framebuf->preamble);
    lifi_dev->transceiver_state.current_frame_part = e_len;
    lifi_send_bits(lifi_dev, sizeof(framebuf->len),&framebuf->len);
    lifi_dev->transceiver_state.current_frame_part = e_payload;
    lifi_send_bits(lifi_dev, framebuf->len,framebuf->payload);
    lifi_dev->transceiver_state.current_frame_part = e_crc16;
    framebuf->crc_16 = htons(framebuf->crc_16);
    lifi_send_bits(lifi_dev, sizeof(framebuf->crc_16), (uint8_t *) &framebuf->crc_16);

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
        transceiver_state->min_tolerated_half_clock.ticks32 = 0;
        transceiver_state->max_tolerated_half_clock.ticks32 = 0;
        preambleFlankCounter = 0;
        transceiver_state->current_frame_part = e_preamble;
    }
    else {
        xtimer_ticks32_t timediff = xtimer_diff(xtimer_now(), transceiver_state->lastReceive);
        transceiver_state->meanHalfClockTicks += (float)timediff.ticks32 / (float)expectedPreambleFlanks;
        transceiver_state->lastReceive = xtimer_now();

        if (preambleFlankCounter >= expectedPreambleFlanks -1) {
            transceiver_state->lastHalfEdge = xtimer_now();
            transceiver_state->current_frame_part = e_len;
            transceiver_state->min_tolerated_half_clock.ticks32 = (uint32_t)(0.6 * transceiver_state->meanHalfClockTicks);
            transceiver_state->max_tolerated_half_clock.ticks32 = (uint32_t)(1.4 * transceiver_state->meanHalfClockTicks);
            transceiver_state->previousStateHigh = gpio_read(lifi_dev->params.input_pin) != 0;
            transceiver_state->high_bit_counter = 7;
        }
    }
}

void init_transceiver_state(lifi_t *lifi_dev) {
    lifi_dev->transceiver_state.min_tolerated_half_clock.ticks32 = 0;
    lifi_dev->transceiver_state.max_tolerated_half_clock.ticks32 = 0;
    lifi_dev->transceiver_state.current_frame_part = e_first_receive;
    lifi_dev->transceiver_state.previousStateHigh = false;
    lifi_dev->transceiver_state.meanHalfClockTicks = 0;
    lifi_dev->transceiver_state.lastReceive.ticks32 = 0 ;
    lifi_dev->transceiver_state.lastHalfEdge.ticks32 = 0;
    lifi_dev->transceiver_state.currentBit = 7;
    lifi_dev->transceiver_state.currentByte = 0;
    lifi_dev->transceiver_state.high_bit_counter = 0;
}
