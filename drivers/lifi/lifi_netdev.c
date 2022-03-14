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

#define ENABLE_DEBUG 0
#include "debug.h"

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
    .isr  = lifi_isr,
    .get  = lifi_get,
    .set  = lifi_set,
};
#define sendDatasize 7
#define receiveDatasize 6
uint8_t receivedData[receiveDatasize];
void isr_callback_input_pin(void *_dev)
{
    lifi_t * lifi_dev = (lifi_t *) _dev;
    static int8_t currentBit = 7;
    static uint16_t currentByte = 0;
    static bool preamble = true;
    static bool firstReceive = true;
    static float meanHalfClockTicks = 0;
    static xtimer_ticks32_t minTolHalfClock;
    static xtimer_ticks32_t maxTolHalfClock;
    static xtimer_ticks32_t lastReceive={};
    static xtimer_ticks32_t lastHalfFlank={};
    uint8_t expectedPreambleFlanks = 15; // we end at T/2
    static uint8_t preambleFlankCounter = 0;
    static bool previousStateHigh = false;

    if (preamble){
        preambleFlankCounter++;
        if(firstReceive){
            lastReceive = xtimer_now();
            firstReceive = false;

//            puts("start preamble");
        } else {
            xtimer_ticks32_t timediff = xtimer_diff(xtimer_now(), lastReceive);
            meanHalfClockTicks += (float)timediff.ticks32 / (float) expectedPreambleFlanks;
            lastReceive = xtimer_now();
        }
        if(preambleFlankCounter >= expectedPreambleFlanks){
//            puts("end preamble");
            preamble = false;
            gpio_t pin = GPIO_PIN(PORT_F, 15);
            gpio_init(pin, GPIO_OUT);
            pin = GPIO_PIN(PORT_G, 14);
            gpio_init(pin, GPIO_OUT);
            minTolHalfClock.ticks32 = (uint32_t) (0.75 * meanHalfClockTicks);
            maxTolHalfClock.ticks32 = (uint32_t) (1.25 * meanHalfClockTicks);
//            printf("meanHalfClock %u\n",(uint16_t)meanHalfClockTicks);
            previousStateHigh = gpio_read(lifi_dev->params.input_pin) != 0;
            preambleFlankCounter = 0;
            lastHalfFlank = xtimer_now();
        }
    } else {
        // we are now at the end of a full period, next edge should come in T/2
        xtimer_ticks32_t timediff = xtimer_diff(xtimer_now(), lastHalfFlank);
        if(timediff.ticks32 >= minTolHalfClock.ticks32 && timediff.ticks32 <= maxTolHalfClock.ticks32) {
//            puts("Drop full flank");
            gpio_toggle(GPIO_PIN(PORT_F, 15));
        }
        else if(timediff.ticks32 >= minTolHalfClock.ticks32 *2 && timediff.ticks32 <= 2* maxTolHalfClock.ticks32) {
            lastHalfFlank = xtimer_now();
            // edge is in expected timing
            if (currentByte < receiveDatasize) {
                if (gpio_read(lifi_dev->params.input_pin) == 0) {
//                    puts("got falling edge");
                    gpio_set(GPIO_PIN(PORT_G, 14));
                    receivedData[currentByte] |= 1 << currentBit--;
                } else {
//                    puts("got rising edge");
                    gpio_clear(GPIO_PIN(PORT_G, 14));
                    receivedData[currentByte] &= ~(1 << currentBit--);
                }
            }
            if (currentBit <= -1){
                currentBit = 7;
                currentByte++;
            }
            if (currentByte >= receiveDatasize) {
                for (int byte = 0; byte < receiveDatasize; ++byte) {
//                    printf("%u \n", receivedData[byte]);
                    printf("char: %c \n",(char) receivedData[byte]);
//                    printf("num: %u \n", receivedData[byte]);
//                    for (int bit = 0; bit < 8; bit++){
//                        printf("%u \n", (receivedData[byte] >> bit) & 0b1);
//                    }
                }
                preamble = true;
                currentBit = 0;
            }

        } else {
            puts("Timing does not fit!\n");
        }

    }
//    puts("got interrupt");
//    cc110x_t *dev = _dev;
//    if ((dev->state & 0x07) == CC110X_STATE_TX_MODE) {
//        /* Unlock mutex to unblock netdev thread */
//        mutex_unlock(&dev->isr_signal);
//    }
//    else {
//        netdev_trigger_event_isr(&dev->netdev);
//    }
}


static int lifi_init(netdev_t *netdev)
{
    DEBUG("[LiFi] netdev_driver_t::init() called lifi_init fun\n");
    lifi_t * lifi_dev = (lifi_t*) netdev;
    const uint32_t frequency = 38000;
    const uint16_t resolution = 255;
    const pwm_t device = lifi_dev->params.output_pwm_device;
    const uint8_t channel = lifi_dev->params.output_pwm_device_channel;
    const pwm_mode_t mode = PWM_LEFT;

    const gpio_flank_t detectedFlanks = GPIO_BOTH;
    const gpio_t inputPin = lifi_dev->params.input_pin;

    bool error = false;
    uint8_t retval = 0;


    // Todo: any mutexes for ISR like in CC1101?
    if (0 == pwm_init(device, mode, frequency, resolution)) {
        DEBUG("[lifi] netdev_driver_t::init(): Failed to setup pwm device\n");
        retval = -EIO;
        error = true;
    }
    if(!error){
        pwm_set(device, channel, 0);
    }

    if(!error){
        if (gpio_init_int(inputPin, GPIO_IN, detectedFlanks, isr_callback_input_pin, lifi_dev) != 0) {
            DEBUG("[lifi] netdev_driver_t::init(): Failed to init input pin gpio interrupt\n");
            retval = -EIO;
            error = true;
        }
    }

    if(!error){
        DEBUG("[lifi] netdev_driver_t::init(): Success\n");
    }
    // todo update status of device
    // todo setup address?
    return retval;
}

static int lifi_recv(netdev_t *netdev, void *buf, size_t len, void *info)
{
    (void) netdev;
    (void) buf;
    (void) len;
    (void) info;
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
void manchesterSend(netdev_t *netdev){

    lifi_t * lifi_dev = (lifi_t*) netdev;
    const uint32_t frequency = 38000;
    const uint16_t resolution = 255;
    const uint16_t gain = resolution/2;
    const uint8_t channel = lifi_dev->params.output_pwm_device_channel;
    const pwm_t device = lifi_dev->params.output_pwm_device;
    const uint32_t secToUsec = 1000000;

    uint32_t signalLengthUs = 1000;
    uint32_t sleepTimeUs = 2500;
    uint32_t sendDurationInSec = 10;

    gpio_t pin = GPIO_PIN(PORT_E, 11);
    gpio_init(pin, GPIO_OUT);
    gpio_t clock = GPIO_PIN(PORT_E, 13);
    gpio_init(clock, GPIO_OUT);
    const uint16_t clockFrequency = 100;

    uint8_t data[sendDatasize] = {0b11111111,'F','e','r','k','e','l'};
    // ,0b11111111,0b11111111,0b10100111,0b10100111};
    bool oldStateHigh = false;
    pwm_set(device, channel, 0);
    gpio_clear(pin);

    for (uint8_t byte = 0; byte < sendDatasize; ++byte) {
        for (int8_t bit = 7; bit >= 0; bit--) {
            gpio_toggle(clock);
            // IEEE 802.3 rising edge for logic 1
            if ((data[byte] >> bit) & 0b1) {
                gpio_set(pin);
                if (oldStateHigh) {
                    pwm_set(device, channel, 0);
                    xtimer_msleep(clockFrequency / 2);
                    pwm_set(device, channel, gain);
                    xtimer_msleep(clockFrequency / 2);
                } else {
                    xtimer_msleep(clockFrequency / 2);
                    pwm_set(device, channel, gain);
                    xtimer_msleep(clockFrequency / 2);
                }
                oldStateHigh = true;
            } else {
                // IEEE 802.3 falling edge for logic 0
                gpio_clear(pin);
                if (oldStateHigh) {
                    xtimer_msleep(clockFrequency / 2);
                    pwm_set(device, channel, 0);
                    xtimer_msleep(clockFrequency / 2);
                } else {
                    pwm_set(device, channel, gain);
                    xtimer_msleep(clockFrequency / 2);
                    pwm_set(device, channel, 0);
                    xtimer_msleep(clockFrequency / 2);
                }
                oldStateHigh = false;
            }
        }
    }
        pwm_set(device, channel, 0); // turn off pwm
        gpio_clear(pin);

}
static int lifi_send(netdev_t *netdev, const iolist_t *iolist)
{

    manchesterSend(netdev);
//    lifi_t * lifi_dev = (lifi_t*) netdev;
//    const uint32_t frequency = 38000;
//    const uint16_t resolution = 255;
//    const uint16_t gain = resolution/2;
//    const uint8_t channel = lifi_dev->params.output_pwm_device_channel;
//    const pwm_t device = lifi_dev->params.output_pwm_device;
//    const uint32_t secToUsec = 1000000;
//
//    uint32_t signalLengthUs = 1000;
//    uint32_t sleepTimeUs = 2500;
//    uint32_t sendDurationInSec = 10;
//    xtimer_ticks32_t start = xtimer_now();
//    while (xtimer_usec_from_ticks(xtimer_diff(xtimer_now(), start)) < (sendDurationInSec * secToUsec)) {
////        puts("Sending");
//        pwm_set(device, channel, gain);
//        xtimer_usleep(signalLengthUs);
//        pwm_set(device, channel, 0);
//        xtimer_msleep(sleepTimeUs);
//    }

    return 1;
//    cc110x_t *dev = (cc110x_t *)netdev;
//
//    /* assert that cc110x_send was called with valid parameters */
//    assert(netdev && iolist && (iolist->iol_len == sizeof(cc1xxx_l2hdr_t)));
//
//    cc110x_acquire(dev);
//
//    switch (dev->state) {
//    case CC110X_STATE_FSTXON:
//        /* falls through */
//    case CC110X_STATE_RX_MODE:
//        break;
//    case CC110X_STATE_RECEIVING:
//        cc110x_release(dev);
//        DEBUG("[cc110x] netdev_driver_t::send(): Refusing to send while "
//              "receiving a frame\n");
//        return -EBUSY;
//    case CC110X_STATE_OFF:
//        cc110x_release(dev);
//        return -ENOTSUP;
//    default:
//        cc110x_release(dev);
//        DEBUG("[cc110x] netdev_driver_t::send(): Driver state %i prevents "
//              "sending\n", (int)dev->state);
//        return -1;
//    }
//
//    /* Copy data to send into frame buffer */
//    size_t size = sizeof(cc1xxx_l2hdr_t);
//    memcpy(dev->buf.data, iolist->iol_base, sizeof(cc1xxx_l2hdr_t));
//
//    for (const iolist_t *iol = iolist->iol_next; iol; iol = iol->iol_next) {
//        if (iol->iol_len) {
//            if (size + iol->iol_len > CC110X_MAX_FRAME_SIZE) {
//                cc110x_release(dev);
//                DEBUG("[cc110x] netdev_driver_t::send(): Frame size of %uB "
//                      "exceeds maximum supported size of %uB\n",
//                      (unsigned)(size + iol->iol_len),
//                      (unsigned)CC110X_MAX_FRAME_SIZE);
//                return -1;
//            }
//            memcpy(dev->buf.data + size, iol->iol_base, iol->iol_len);
//            size += iol->iol_len;
//        }
//    }
//
//    dev->buf.len = (uint8_t)size;
//
//    /* Disable IRQs, as GDO configuration will be changed now */
//    gpio_irq_disable(dev->params.gdo0);
//    gpio_irq_disable(dev->params.gdo2);
//
//    /* Fill the TX FIFO: First write the length, then the frame */
//    dev->buf.pos = (size > CC110X_FIFO_SIZE - 1) ? CC110X_FIFO_SIZE - 1 : size;
//    /* cc110x_framebuf_t has the same memory layout as the device expects */
//    cc110x_burst_write(dev, CC110X_MULTIREG_FIFO,
//                       &dev->buf, dev->buf.pos + 1);
//
//    /* Go to TX */
//    cc110x_cmd(dev, CC110X_STROBE_TX);
//
//    /* Configure GDO2 and update state */
//    if (dev->buf.pos < dev->buf.len) {
//        /* We need to keep feeding TX FIFO */
//        cc110x_write(dev, CC110X_REG_IOCFG2, CC110X_GDO_ON_TX_DATA);
//        dev->state = CC110X_STATE_TX_MODE;
//    }
//    else {
//        /* All data in TX FIFO, just waiting for transceiver to finish */
//        cc110x_write(dev, CC110X_REG_IOCFG2, CC110X_GDO_CONSTANT_LOW);
//        dev->state = CC110X_STATE_TX_COMPLETING;
//    }
//
//    cc110x_release(dev);
//
//    /* Restore IRQs */
//    gpio_irq_enable(dev->params.gdo0);
//    gpio_irq_enable(dev->params.gdo2);
//
//    while ((dev->state & 0x07) == CC110X_STATE_TX_MODE) {
//        uint64_t timeout = (dev->state != CC110X_STATE_TX_COMPLETING) ?
//                           2048 : 1024;
//        /* Block until mutex is unlocked from ISR, or a timeout occurs. The
//         * timeout prevents deadlocks when IRQs are lost. If the TX FIFO
//         * still needs to be filled, the timeout is 1024 µs - or after 32 Byte
//         * (= 50%) of the FIFO have been transmitted at 250 kbps. If
//         * no additional data needs to be fed into the FIFO, a timeout of
//         * 2048 µs is used instead to allow the frame to be completely drained
//         * before the timeout triggers. The ISR handler is prepared to be
//         * called prematurely, so we don't need to worry about extra calls
//         * to cc110x_isr() introduced by accident.
//         */
//        xtimer_mutex_lock_timeout(&dev->isr_signal, timeout);
//        cc110x_isr(&dev->netdev);
//    }
//
//    return (int)size;
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
