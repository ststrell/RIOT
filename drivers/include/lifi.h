

#ifndef LIFI_H
#define LIFI_H


#include <stdint.h>
#include <periph/gpio.h>
#include <net/netdev.h>
#include <periph/pwm.h>
#include <xtimer.h>

//#include "cc1xxx_common.h"
#include "mutex.h"
#include "cc1xxx_common.h"
//#include "net/gnrc/nettype.h"
//#include "net/netdev.h"
//#include "periph/adc.h"
//#include "periph/gpio.h"
//#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Length of a layer 2 frame
 *
 * This does not include the preamble, sync word, CRC field, and length field.
 */
#define CC110X_MAX_FRAME_SIZE           0xFF

/**
 * @brief   Maximum (layer 2) payload size supported by the driver
 */
#define CC110X_MAX_PAYLOAD_SIZE         (CC110X_MAX_FRAME_SIZE - CC1XXX_HEADER_SIZE)

/**
 * @brief   Maximum number of channels supported by the driver
 */
#define CC110X_MAX_CHANNELS             8

/**
 * @brief   Default protocol for data that is coming in
 */
#ifdef MODULE_GNRC_SIXLOWPAN
#define CC110X_DEFAULT_PROTOCOL         (GNRC_NETTYPE_SIXLOWPAN)
#else
#define CC110X_DEFAULT_PROTOCOL         (GNRC_NETTYPE_UNDEF)
#endif

/**
 * @defgroup drivers_cc110x_config CC1100/CC1100e/CC1101 Sub-GHz transceiver driver
 *                                 compile time configuration
 * @ingroup config_drivers_netdev
 * @{
 */
/**
 * @brief The default channel to set up after initializing the device
 */
#ifndef CONFIG_CC110X_DEFAULT_CHANNEL
#define CONFIG_CC110X_DEFAULT_CHANNEL      (0U)
#endif
/** @} */

/**
 * @brief   The state of the CC1100/CC1101 transceiver
 *
 * The three least significant bytes match the representation of the matching
 * transceiver state given in the status byte of the hardware. See Table 32 on
 * page 31 in the data sheet for the possible states in the status byte.
 */
typedef enum {
    LIFI_STATE_RECEIVING = 0,          /**< Receiving a frame */
    LIFI_STATE_TRANSMITTING = 1,       /**< Transmitting a frame */
    LIFI_STATE_IDLE = 2,               /**< IDLE state */
} lifi_state_t;

/**
 * @brief   Structure holding all parameter for driver initialization
 */
typedef struct {
//    spi_t spi;                          /**< SPI bus connected to the device */
//    spi_clk_t spi_clk;                  /**< SPI clock to use (max 6.5 MHz) */
//    spi_cs_t cs;                        /**< GPIO pin connected to chip select */
//    gpio_t gdo0;                        /**< GPIO pin connected to GDO0 */
    pwm_t output_pwm_device;                        /**< GPIO pin connected to GDO2 */
    uint8_t output_pwm_device_channel;                        /**< GPIO pin connected to GDO2 */
    gpio_t input_pin;                        /**< GPIO pin connected to GDO2 */
} lifi_params_t;

/**
 * @brief   Buffer to temporary store incoming/outgoing packet
 *
 * The CC1100/CC1101 transceiver's FIFO sadly is only 64 bytes in size. To
 * support frames bigger than that, chunks of the frame have to be
 * transferred between the MCU and the CC1100/CC1101 transceiver while the
 * frame is in transit.
 */
typedef struct __attribute__((packed)) {
    uint8_t preamble;
    uint8_t len;            /**< Length of the payload in bytes */
    cc1xxx_l2hdr_t layer2_hdr;           /**< Length of the payload in bytes */
    /**
     * @brief   The payload data of the frame
     */
    uint8_t payload[CC110X_MAX_FRAME_SIZE];
    uint16_t crc_16;
    /**
     * @brief   Index of the next @ref cc110x_framebuf_t::data element to
     *          transfer
     *
     * In RX mode: Index of the next @ref cc110x_framebuf_t::data element to
     * store data read from the RX-FIFO into.
     *
     * In TX mode: Index of the next @ref cc110x_framebuf_t::data element to
     * write to the TX-FIFO.
     */
    uint8_t pos;
} lifi_framebuf_t;

typedef enum {
    e_first_receive,
    e_preamble,
    e_len,
    e_layer2_header,
    e_payload,
    e_crc16
} lifi_transceiver_frame_part;

typedef struct{
    xtimer_ticks32_t min_tolerated_half_clock;
    xtimer_ticks32_t max_tolerated_half_clock;
    lifi_transceiver_frame_part current_frame_part;
    bool previousStateHigh;
    float meanHalfClockTicks;
    xtimer_ticks32_t lastReceive;
    xtimer_ticks32_t lastHalfEdge;
    int8_t currentBit;
    uint16_t currentByte;
    uint8_t high_bit_counter;
} lifi_transceiver_state_t;

/**
 * @brief   Device descriptor for CC1100/CC1101 transceivers
 */
typedef struct {
    netdev_t netdev;                    /**< RIOT's interface to this driver */
    uint8_t addr;                       /**< Layer 2 address of this device */
    uint16_t baud;
    /* Keep above in sync with cc1xx_t members, as they must overlap! */
    lifi_state_t state;               /**< State of the transceiver */
    /* Struct packing:  addr, state, tx_power and channel add up to 32 bit */
    lifi_params_t params;             /**< Configuration of the driver */
    lifi_framebuf_t output_buf;              /**< Temporary frame buffer */
    lifi_framebuf_t input_buf;              /**< Temporary frame buffer */
    lifi_transceiver_state_t transceiver_state;
    /**
     * @brief   Use mutex to block during TX and unblock from ISR when ISR
     *          needs to be handled from thread-context
     *
     * Blocking during TX within the driver prevents the upper layers from
     * calling @ref netdev_driver_t::send while already transmitting a frame.
     */
    mutex_t isr_signal;
} lifi_t;

/**
 * @brief   Setup the CC1100/CC1101 driver, but perform no initialization
 *
 * @ref netdev_driver_t::init can be used after this call to initialize the
 * transceiver.
 *
 * @param   dev     Device descriptor to use
 * @param   params  Parameter of the device to setup
 * @param   index   Index of @p params in a global parameter struct array.
 *                  If initialized manually, pass a unique identifier instead.
 *
 * @retval  0       Device successfully set up
 * @retval  -EINVAL @p dev or @p params is `NULL`, or @p params is invalid
 */
int lifi_setup(lifi_t *dev, const lifi_params_t *params, uint8_t index);

int lifi_set_baud(lifi_t* dev, uint16_t baud);

#ifdef __cplusplus
}
#endif

#endif