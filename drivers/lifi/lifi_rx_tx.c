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

#include "xtimer.h"
#include "lifi.h"

#define ENABLE_DEBUG 0
#include "debug.h"

/* Use NETDEV_EVENT_ISR to indicate that no event needs to be passed to upper
 * layer at end of ISR, as ISR will never need this event
 */
#define NETDEV_NO_EVENT NETDEV_EVENT_ISR

void lifi_enter_rx_mode(lifi_t *dev)
{
    (void) dev;
//    DEBUG("[cc110x] Going to RX\n");
//    /* bring device to IDLE state and flush FIFOs (just in case) */
//    gpio_irq_disable(dev->params.gdo0);
//    gpio_irq_disable(dev->params.gdo2);
//    cc110x_cmd(dev, CC110X_STROBE_IDLE);
//    cc110x_cmd(dev, CC110X_STROBE_FLUSH_RX);
//    cc110x_cmd(dev, CC110X_STROBE_FLUSH_TX);
//    dev->buf.pos = dev->buf.len = 0;
//    /* Apply GDO2 config and go to RX */
//    cc110x_write(dev, CC110X_REG_IOCFG2, CC110X_GDO_ON_RX_DATA);
//    cc110x_write(dev, CC110X_REG_IOCFG0, CC110X_GDO_ON_TRANSMISSION);
//    cc110x_cmd(dev, CC110X_STROBE_RX);
//    dev->state = CC110X_STATE_RX_MODE;
//    gpio_irq_enable(dev->params.gdo2);
//    gpio_irq_enable(dev->params.gdo0);
}

/**
 * @brief   Function to run when frame is fully received
 *
 * @param   dev     Device descriptor of the transceiver
 *
 * Intended to be called from @ref cc110x_rx_continue
 */
static netdev_event_t lifi_rx_done(lifi_t *dev)
{
    (void) dev;
    return NETDEV_EVENT_RX_COMPLETE;
//    uint8_t lqi_crc;
//    int8_t rssi;
//
//    cc110x_read(dev, CC110X_REG_LQI, &lqi_crc);
//    cc110x_read(dev, CC110X_REG_RSSI, (uint8_t *)&rssi);
//
//    /* CRC_OK bit is most significant bit, see page 92 in the data sheet */
//    if (!(lqi_crc & 0x80)) {
//        DEBUG("[cc110x] ISR: CRC error, dropping frame\n");
//        /* Drop frame and go back to RX */
//        cc110x_enter_rx_mode(dev);
//        return NETDEV_EVENT_CRC_ERROR;
//    }
//
//    /* Copy all but the CRC_OK bit */
//    dev->rx_info.lqi = (uint8_t)lqi_crc & 0x7f;
//
//    /* Use the formula in section 17.3 on page 44 in the data sheet to obtain
//     * the correct RSSI value in dBm.
//     */
//    dev->rx_info.rssi =  (int16_t)(rssi / 2) - (int16_t)dev->rssi_offset;
//
//    /* Transceiver has automatically gone to IDLE. We keep it in IDLE until
//     * upper layer fetched the frame
//     */
//    dev->state = CC110X_STATE_FRAME_READY;
}

/**
 * @brief   Read a chunk of data from the RX-FIFO
 *
 * @param   dev     Device descriptor of the transceiver
 *
 * This function should be called from the ISR when data in the RX-FIFO is
 * available or the last byte of the frame was received
 */
static netdev_event_t lifi_rx_continue(lifi_t *dev)
{
    (void) dev;
    return NETDEV_NO_EVENT;
//    uint8_t in_fifo;
//    netdev_event_t retval = NETDEV_NO_EVENT;
//
//    while (gpio_read(dev->params.gdo2)) {
//        cc110x_read_reliable(dev, CC110X_REG_RXBYTES, &in_fifo);
//
//        if (in_fifo & 0x80) {
//            /* RXFIFO_OVERFLOW bit is set (see RXBYTES on page 94) */
//            DEBUG("[cc110x] ISR: RX-FIFO overflown, ISR too slow\n");
//            /* Drop frame and go to RX */
//            cc110x_enter_rx_mode(dev);
//            return NETDEV_EVENT_RX_TIMEOUT;
//        }
//
//        if (!in_fifo) {
//            /* GDO2 will be high when data is present *or* at end of packet */
//            break;
//        }
//
//        /* Handle first read from RX FIFO differently from subsequent reads, as
//         * in first reads the Length Field is read as well
//         */
//        if (!dev->buf.len) {
//            if (in_fifo < sizeof(cc1xxx_l2hdr_t) + 1) {
//                /* At least a frame header + Length Field (1B) is expected */
//                DEBUG("[cc110x] ISR: Incoming frame smaller than header "
//                      "--> drop\n");
//                cc110x_enter_rx_mode(dev);
//                /* Not exactly CRC, but incorrect CRC indicates a broken frame*/
//                return NETDEV_EVENT_CRC_ERROR;
//            }
//            cc110x_burst_read(dev, CC110X_MULTIREG_FIFO, &dev->buf,
//                              in_fifo - 1);
//            /* Update read position in payload, that is number of bytes read
//             * minus the Length Filed and minus the byte left in the FIFO to not
//             * trigger a silicon bug
//             */
//            dev->buf.pos = in_fifo - 2;
//            retval = NETDEV_EVENT_RX_STARTED;
//        }
//        else {
//            /* Prevent overflow of buffer */
//            if (dev->buf.pos + in_fifo > CC110X_MAX_FRAME_SIZE) {
//                DEBUG("[cc110x] ISR: Incoming frame exceeds maximum size\n");
//                cc110x_enter_rx_mode(dev);
//                /* Not exactly CRC, but incorrect CRC indicates a broken frame */
//                return NETDEV_EVENT_CRC_ERROR;
//            }
//
//            if (dev->buf.pos + in_fifo < dev->buf.len) {
//                /* Frame not fully received yet, keeping one byte in RX FIFO
//                 * to prevent triggering a silicon bug
//                 */
//                in_fifo--;
//            }
//
//            /* Continue reading data */
//            cc110x_burst_read(dev, CC110X_MULTIREG_FIFO,
//                              dev->buf.data + dev->buf.pos, in_fifo);
//            dev->buf.pos += in_fifo;
//
//        }
//    }
//
//    if (dev->buf.pos > dev->buf.len) {
//        DEBUG("[cc110x] ISR: Incoming frame larger than Length Field "
//              "--> drop\n");
//        cc110x_enter_rx_mode(dev);
//        /* Not exactly CRC, but incorrect CRC indicates a broken frame */
//        return NETDEV_EVENT_CRC_ERROR;
//    }
//
//    if (!gpio_read(dev->params.gdo0)) {
//        /* GDO0 is low when transmission is over ==> RX complete or corrupt
//           frame */
//        if (dev->buf.pos == dev->buf.len) {
//            return cc110x_rx_done(dev);
//        }
//        else {
//            DEBUG("[cc110x] ISR: Incoming frame smaller than Length Field "
//                  "--> drop\n");
//            cc110x_enter_rx_mode(dev);
//            /* Not exactly CRC, but incorrect CRC indicates a broken frame */
//            return NETDEV_EVENT_CRC_ERROR;
//        }
//    }
//
//    return retval;
}

/**
 * @brief   Function to run when frame is fully send
 *
 * @param   dev     Device descriptor of the transceiver
 */
static netdev_event_t lifi_tx_done(lifi_t *dev)
{
    (void)dev;
    return NETDEV_EVENT_TX_COMPLETE;
//    uint8_t status = cc110x_status(dev);
//    cc110x_state_t state = cc110x_state_from_status(status);
//    switch (state){
//        case CC110X_STATE_SETTLING:
//        case CC110X_STATE_CALIBRATE:
//        case CC110X_STATE_TX_MODE:
//            /* TX still in progress, or hasn't even started yet */
//            return NETDEV_NO_EVENT;
//        case CC110X_STATE_IDLE:
//            cc110x_enter_rx_mode(dev);
//            return NETDEV_EVENT_TX_COMPLETE;
//        case CC110X_STATE_TXFIFO_UNDERFLOW:
//            DEBUG("[cc110x] ISR: TX FIFO underflown.\n");
//            break;
//        default:
//            DEBUG("[cc110x] ISR: Unknown state during TX.\n");
//            break;
//    }
//
//    cc110x_enter_rx_mode(dev);
//    /* TX timeout is the only TX-related error event known to RIOT */
//    return NETDEV_EVENT_TX_TIMEOUT;
}

/**
 * @brief   Refill the TX-FIFO
 *
 * @param   dev     Device descriptor of the transceiver
 */
static netdev_event_t lifi_tx_continue(lifi_t *dev)
{
    (void)dev;
    return NETDEV_EVENT_TX_TIMEOUT;
//    uint8_t in_fifo;
//
//    cc110x_read_reliable(dev, CC110X_REG_TXBYTES, &in_fifo);
//
//    /* most significant bit indicates TXFIFO underflow, see page 94 in the
//     * data sheet
//     */
//    if (in_fifo & 0x80) {
//        DEBUG("[cc110x] ISR: ERROR: TX-FIFO underflown, ISR too slow\n");
//        /* Abort: Flush TX and go back to RX */
//        cc110x_cmd(dev, CC110X_STROBE_IDLE);
//        cc110x_cmd(dev, CC110X_STROBE_FLUSH_TX);
//        cc110x_enter_rx_mode(dev);
//        return NETDEV_EVENT_TX_TIMEOUT;
//    }
//
//    uint8_t to_write = CC110X_FIFO_SIZE - in_fifo;
//
//    if (to_write == 0) {
//        /* ISR came to early, nothing to do yet */
//        return NETDEV_NO_EVENT;
//    }
//
//    uint8_t left = dev->buf.len - dev->buf.pos;
//    to_write = (left < to_write) ? left : to_write;
//
//    cc110x_burst_write(dev, CC110X_MULTIREG_FIFO,
//                       dev->buf.data + dev->buf.pos, to_write);
//    dev->buf.pos += to_write;
//
//    if (dev->buf.pos == dev->buf.len) {
//        /* All data send to the transceiver, now waiting for transceiver to
//         * complete transmission
//         */
//        dev->state = CC110X_STATE_TX_COMPLETING;
//        /* Disable GDO2, as we do not need to further feed TX FIFO */
//        cc110x_write(dev, CC110X_REG_IOCFG2, CC110X_GDO_CONSTANT_LOW);
//    }
//
//    return NETDEV_NO_EVENT;
}

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

static void lifi_send_bits(lifi_t* lifi_dev,uint16_t num_bytes, uint8_t* bytes){
    const uint16_t resolution = 255;
    const uint16_t gain = resolution / 2;
    const uint8_t channel = lifi_dev->params.output_pwm_device_channel;
    const pwm_t device = lifi_dev->params.output_pwm_device;
    lifi_framebuf_t * framebuf = &lifi_dev->output_buf;

    gpio_t pin = GPIO_PIN(PORT_E, 11);
    gpio_t clock = GPIO_PIN(PORT_E, 13);

    const uint16_t sleepTimeUs = 1000;

    static bool oldStateHigh = false;

    for(uint16_t byte = 0; byte < num_bytes; byte++){
        for (int8_t bit = 7; bit >= 0; bit--) {
            gpio_toggle(clock);
            // IEEE 802.3 rising edge for logic 1
            if ((bytes[byte] >> bit) & 0b1) {
                gpio_set(pin);
                if (oldStateHigh) {
                    pwm_set(device, channel, 0);
                    xtimer_usleep(sleepTimeUs / 2);
                    pwm_set(device, channel, gain);
                    xtimer_usleep(sleepTimeUs / 2);
                }
                else {
                    xtimer_usleep(sleepTimeUs / 2);
                    pwm_set(device, channel, gain);
                    xtimer_usleep(sleepTimeUs / 2);
                }
                oldStateHigh = true;
            }
            else {
                // IEEE 802.3 falling edge for logic 0
                gpio_clear(pin);
                if (oldStateHigh) {
                    xtimer_usleep(sleepTimeUs / 2);
                    pwm_set(device, channel, 0);
                    xtimer_usleep(sleepTimeUs / 2);
                }
                else {
                    pwm_set(device, channel, gain);
                    xtimer_usleep(sleepTimeUs / 2);
                    pwm_set(device, channel, 0);
                    xtimer_usleep(sleepTimeUs / 2);
                }
                oldStateHigh = false;
            }
        }
        framebuf->pos++;
    }
}

void lifi_send_frame(lifi_t* lifi_dev){
    const uint16_t resolution = 255;
    const uint16_t gain = resolution / 2;
    const uint8_t channel = lifi_dev->params.output_pwm_device_channel;
    const pwm_t device = lifi_dev->params.output_pwm_device;
    lifi_framebuf_t * framebuf = &lifi_dev->output_buf;
    gpio_t pin = GPIO_PIN(PORT_E, 11);

    gpio_init(pin, GPIO_OUT);
    gpio_t clock = GPIO_PIN(PORT_E, 13);

    gpio_init(clock, GPIO_OUT);
    const uint16_t sleepTimeUs = 1000;

    framebuf->crc_16 = 0;

    lifi_send_bits(lifi_dev, sizeof(framebuf->preamble),&framebuf->preamble);
    lifi_send_bits(lifi_dev, sizeof(framebuf->len),&framebuf->len);
    lifi_send_bits(lifi_dev, framebuf->len,framebuf->payload);
    lifi_send_bits(lifi_dev, sizeof(framebuf->crc_16), (uint8_t *) &framebuf->crc_16);

    framebuf->pos = 0;
    pwm_set(device, channel, 0);     // turn off pwm
    gpio_clear(pin);
}