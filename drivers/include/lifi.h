

#ifndef LIFI_H
#define LIFI_H


#include <stdint.h>
#include <periph/gpio.h>
#include <net/netdev.h>
#include <periph/pwm.h>
#include <xtimer.h>

//#include "cc1xxx_common.h"
#include "mutex.h"
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
//    CC110X_STATE_IDLE               = 0x00,   /**< IDLE state */
//    /**
//     * @brief   Frame received, waiting for upper layer to retrieve it
//     *
//     * Transceiver is in IDLE state.
//     */
//    CC110X_STATE_FRAME_READY        = 0x08,
//    /**
//     * @brief   Devices is powered down
//     *
//     * Transceiver is in SLEEP state. There is no matching representation in the
//     * status byte, as reading the status byte will power up the transceiver in
//     * bring it in the IDLE state. Thus, we set the three least significant bits
//     * to the IDLE state
//     */
//    CC110X_STATE_OFF                = 0x10,
//    CC110X_STATE_RX_MODE            = 0x01,   /**< Listening for frames */
//    /**
//     * @brief   Receiving a frame just now
//     *
//     * Transceiver is in RX state.
//     */
//    CC110X_STATE_RECEIVING          = 0x09,
//    CC110X_STATE_TX_MODE            = 0x02,   /**< Transmit mode */
//    /**
//     * @brief   Waiting for transceiver to complete outgoing transmission
//     *
//     * Transceiver is in TX state
//     */
//    CC110X_STATE_TX_COMPLETING      = 0x0A,
//    CC110X_STATE_FSTXON             = 0x03,   /**< Fast TX ready */
//    CC110X_STATE_CALIBRATE          = 0x04,   /**< Device is calibrating */
//    CC110X_STATE_SETTLING           = 0x05,   /**< PLL is settling */
//    CC110X_STATE_RXFIFO_OVERFLOW    = 0x06,   /**< RX FIFO overflown */
//    CC110X_STATE_TXFIFO_UNDERFLOW   = 0x07,   /**< TX FIFO underflown */
    LIFI_STATE_IDLE = 0x00,   /**< IDLE state */
} lifi_state_t;
//
///**
// * @brief Enumeration over the possible TX power settings the driver offers
// */
//typedef enum {
//    CC110X_TX_POWER_MINUS_30_DBM,                   /**< -30 dBm */
//    CC110X_TX_POWER_MINUS_20_DBM,                   /**< -20 dBm */
//    CC110X_TX_POWER_MINUS_15_DBM,                   /**< -15 dBm */
//    CC110X_TX_POWER_MINUS_10_DBM,                   /**< -10 dBm */
//    CC110X_TX_POWER_0_DBM,                          /**< 0 dBm */
//    CC110X_TX_POWER_PLUS_5_DBM,                     /**< 5 dBm */
//    CC110X_TX_POWER_PLUS_7_DBM,                     /**< 7 dBm */
//    CC110X_TX_POWER_PLUS_10_DBM,                    /**< 10 dBm */
//    CC110X_TX_POWER_NUMOF,                          /**< Number of TX power options */
//} cc110x_tx_power_t;
//
///**
// * @brief   Structure that holds the PATABLE, which allows to configure the
// *          8 available output power levels using a magic number for each level.
// *
// * See Section "24 Output Power Programming" on page 59ff in the data sheet.
// * The values suggested in Table 39 on page 60 in the data sheet are already
// * available by this driver, but will only be linked in (8 bytes of ROM size)
// * when they are referenced.
// *
// * @see cc110x_patable_433mhz
// * @see cc110x_patable_868mhz
// * @see cc110x_patable_915mhz
// */
//typedef struct {
//    uint8_t data[8]; /**< Magic number to store in the configuration register */
//} cc110x_patable_t;
//
///**
// * @brief   Configuration of the transceiver to use
// *
// * @warning Two transceivers with different configurations will be unable
// *          to communicate.
// *
// * The data uploaded into configuration registers are stored in
// * @ref cc110x_conf. Most of them cannot be changed, as the driver relies on
// * their values. However, the base frequency, the symbol rate (which equals
// * the bit rate for the chosen modulation and error correction) and the
// * channel bandwidth can be configured using this data structure.
// *
// * Please note that while the CC1100/CC1101 chip is compatible with a huge
// * frequency range (300 MHz - 928 MHz), the complete circuit is optimized to
// * a narrow frequency band. So make sure the configured base frequency is within
// * that frequency band that is compatible with that circuit. (Most break out
// * board will operate at the 433 MHz band. In the EU the 868 MHz band would be
// * more interesting, but 433 MHz is license free as well. In the USA the 915 MHz
// * band is license free.
// *
// * Please verify that the driver is configured in a way that allows legal
// * operation according to rules and laws that apply for you.
// */
//typedef struct {
//    uint8_t base_freq[3]; /**< Base frequency to use */
//    /**
//     * @brief   FSCTRL1 configuration register value that affects the
//     *          intermediate frequency of the transceiver to use
//     * @note    The 4 most significant bits have to be 0.
//     *
//     * Assuming a 26 MHz crystal the IF is calculated as follows (in kHz):
//     *
//     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     * double intermediate_frequency = 26000 / 1024 * fsctrl1;
//     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     */
//    uint8_t fsctrl1;
//    /**
//     * @brief   MDMCFG4 configuration register value that affects channel filter
//     *          bandwidth and the data rate
//     *
//     * See page 76 in the data sheet.
//     *
//     * Assuming a 26 MHz crystal the channel filter bandwidth is calculated
//     * as follows (in kHz):
//     *
//     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     * uint8_t exponent = mdmcfg4 >> 6;
//     * uint8_t mantissa = (mdmcfg4 >> 4) & 0x03;
//     * double bandwidth = 26000.0 / (8 * (4 + mantissa) * (1L << exponent));
//     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     */
//    uint8_t mdmcfg4;
//    /**
//     * @brief   MDMCFG3 configuration register value that affects the data rate
//     *
//     * @see     cc110x_config_t::mdmcfg4
//     *
//     * See page 76 in the data sheet.
//     *
//     * Assuming a 26 MHz crystal the symbol rate of the transceiver is calculated
//     * as follows (in kBaud):
//     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     * uint8_t exponent = mdmcfg4 & 0x0f;
//     * int32_t mantissa = mdmcfg3;
//     * double baudrate = (256 + mantissa) * 26000.0 / (1L << (28 - exponent));
//     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     */
//    uint8_t mdmcfg3;
//    /**
//     * @brief   DEVIANT configuration register that affects the amount by which
//     *          the radio frequency is shifted in FSK/GFSK modulation
//     *
//     * @see cc110x_config_t::mdmcfg4
//     *
//     * See page 79 in the data sheet.
//     *
//     * In an ideal world the channel bandwidth would be twice the channel
//     * deviation. In the real world the used channel bandwidth is higher.
//     * Assuming a 26 MHz crystal and GFSK modulation (the driver will configure
//     * the transceiver to use GFSK) the deviation
//     *
//     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     * uint8_t exponent = (deviatn >> 4) & 0x07;
//     * int32_t mantissa = deviatn & 0x07;
//     * double deviation = (8 + mantissa) * 26000.0 / (1L << (17 - exponent));
//     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     *
//     * For reliable operation at high symbol rates, the deviation has to be
//     * increased as well.
//     */
//    uint8_t deviatn;
//} cc110x_config_t;
//
///**
// * @brief   Structure to hold mapping between virtual and physical channel numbers
// *
// * This driver will provide "virtual" channel numbers 0 to 7, which will be
// * translated to "physical" channel numbers before being send to the
// * transceiver. This is used to overcome the following limitations:
// *
// * - The transceiver does not support channel maps with varying distance between
// *   channels. However, e.g. the LoRa channels 10 - 16 in the 868 MHz band have
// *   a distance of 300 kHz, but channel 16 and 17 have a distance of 1 MHz.
// * - The transceiver does not supports channel distances higher than 405.46 kHz.
// *
// * This mapping overcomes both limitations be using 50kHz physical channel
// * spacing and use the map to translate to the correct physical channel. This
// * also allows to keep the same MDMCFG1 and MDMCFG0 configuration register
// * values for all channel layouts. Finally, different channel sets can be
// * used by different groups of IoT device in the same environment to limit
// * collisions between those groups - assuming that enough non-overlapping
// * channels are available.
// *
// * The "virtual" channel (the channel number presented to RIOT) will be used
// * as index in @ref cc110x_chanmap_t::map, the value in there will give the
// * corresponding "physical" channel number, or 255 if this virtual channel
// * number is not available.
// */
//typedef struct {
//    uint8_t map[CC110X_MAX_CHANNELS]; /**< "Physical" channel numbers */
//} cc110x_chanmap_t;
//
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
//
///**
// * @brief   Structure holding the calibration data of the frequency synthesizer
// */
//typedef struct {
//    /**
//     * @brief   VCO capacitance calibration, which depends on the frequency and,
//     *          thus, has to be stored for each channel
//     */
//    char fscal1[CC110X_MAX_CHANNELS];
//    char fscal2;    /**< VCO current calibration, independent of channel */
//    char fscal3;    /**< charge pump current calibration, independent of channel */
//} cc110x_fs_calibration_t;
//
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
    /* Keep above in sync with cc1xx_t members, as they must overlap! */
    lifi_state_t state;               /**< State of the transceiver */
    uint8_t channel;                    /**< Currently tuned (virtual) channel */
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

#ifdef __cplusplus
}
#endif

#endif