/*
 * http://www.cascoda.com/products/ca-821x/
 * Copyright (c) 2016, Cascoda, Ltd.
 * All rights reserved.
 *
 * This code is dual-licensed under both GPLv2 and 3-clause BSD. What follows is
 * the license notice for both respectively.
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 *
 *******************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/cdev.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/ieee802154.h>
#include <linux/kfifo.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/workqueue.h>

#include <net/ieee802154_netdev.h>
#include <net/mac802154.h>

#define DRIVER_NAME "ca8210"

/* external clock frequencies */
#define ONE_MHZ      1000000
#define TWO_MHZ      (2*ONE_MHZ)
#define FOUR_MHZ     (4*ONE_MHZ)
#define EIGHT_MHZ    (8*ONE_MHZ)
#define SIXTEEN_MHZ  (16*ONE_MHZ)

/* spi constants */
#define CA8210_SPI_BUF_SIZE 256
#define CA8210_SYNC_TIMEOUT 1000     /* Timeout for synchronous commands [ms] */

/* api constants */
#define CA8210_DATA_CNF_TIMEOUT 300   /* Timeout for data confirms [ms] */

/* test interface constants */
#define CA8210_TEST_INT_FILE_NAME "ca8210_test"
#define CA8210_TEST_INT_FIFO_SIZE 256

/* MAC status enumerations */
#define MAC_SUCCESS                     (0x00)
#define MAC_ERROR                       (0x01)
#define MAC_CANCELLED                   (0x02)
#define MAC_READY_FOR_POLL              (0x03)
#define MAC_COUNTER_ERROR               (0xDB)
#define MAC_IMPROPER_KEY_TYPE           (0xDC)
#define MAC_IMPROPER_SECURITY_LEVEL     (0xDD)
#define MAC_UNSUPPORTED_LEGACY          (0xDE)
#define MAC_UNSUPPORTED_SECURITY        (0xDF)
#define MAC_BEACON_LOST                 (0xE0)
#define MAC_CHANNEL_ACCESS_FAILURE      (0xE1)
#define MAC_DENIED                      (0xE2)
#define MAC_DISABLE_TRX_FAILURE         (0xE3)
#define MAC_SECURITY_ERROR              (0xE4)
#define MAC_FRAME_TOO_LONG              (0xE5)
#define MAC_INVALID_GTS                 (0xE6)
#define MAC_INVALID_HANDLE              (0xE7)
#define MAC_INVALID_PARAMETER           (0xE8)
#define MAC_NO_ACK                      (0xE9)
#define MAC_NO_BEACON                   (0xEA)
#define MAC_NO_DATA                     (0xEB)
#define MAC_NO_SHORT_ADDRESS            (0xEC)
#define MAC_OUT_OF_CAP                  (0xED)
#define MAC_PAN_ID_CONFLICT             (0xEE)
#define MAC_REALIGNMENT                 (0xEF)
#define MAC_TRANSACTION_EXPIRED         (0xF0)
#define MAC_TRANSACTION_OVERFLOW        (0xF1)
#define MAC_TX_ACTIVE                   (0xF2)
#define MAC_UNAVAILABLE_KEY             (0xF3)
#define MAC_UNSUPPORTED_ATTRIBUTE       (0xF4)
#define MAC_INVALID_ADDRESS             (0xF5)
#define MAC_ON_TIME_TOO_LONG            (0xF6)
#define MAC_PAST_TIME                   (0xF7)
#define MAC_TRACKING_OFF                (0xF8)
#define MAC_INVALID_INDEX               (0xF9)
#define MAC_LIMIT_REACHED               (0xFA)
#define MAC_READ_ONLY                   (0xFB)
#define MAC_SCAN_IN_PROGRESS            (0xFC)
#define MAC_SUPERFRAME_OVERLAP          (0xFD)
#define MAC_SYSTEM_ERROR                (0xFF)

/* HWME attribute IDs */
#define HWME_EDTHRESHOLD       (0x04)
#define HWME_EDVALUE           (0x06)
#define HWME_SYSCLKOUT         (0x0F)

/* TDME attribute IDs */
#define TDME_CHANNEL          (0x00)
#define TDME_ATM_CONFIG       (0x06)

#define MAX_HWME_ATTRIBUTE_SIZE  16
#define MAX_TDME_ATTRIBUTE_SIZE  2

/* PHY/MAC PIB Attribute Enumerations */
#define PHY_CURRENT_CHANNEL               (0x00)
#define PHY_TRANSMIT_POWER                (0x02)
#define PHY_CCA_MODE                      (0x03)
#define MAC_ASSOCIATION_PERMIT            (0x41)
#define MAC_AUTO_REQUEST                  (0x42)
#define MAC_BATT_LIFE_EXT                 (0x43)
#define MAC_BATT_LIFE_EXT_PERIODS         (0x44)
#define MAC_BEACON_PAYLOAD                (0x45)
#define MAC_BEACON_PAYLOAD_LENGTH         (0x46)
#define MAC_BEACON_ORDER                  (0x47)
#define MAC_GTS_PERMIT                    (0x4d)
#define MAC_MAX_CSMA_BACKOFFS             (0x4e)
#define MAC_MIN_BE                        (0x4f)
#define MAC_PAN_ID                        (0x50)
#define MAC_PROMISCUOUS_MODE              (0x51)
#define MAC_RX_ON_WHEN_IDLE               (0x52)
#define MAC_SHORT_ADDRESS                 (0x53)
#define MAC_SUPERFRAME_ORDER              (0x54)
#define MAC_ASSOCIATED_PAN_COORD          (0x56)
#define MAC_MAX_BE                        (0x57)
#define MAC_MAX_FRAME_RETRIES             (0x59)
#define MAC_RESPONSE_WAIT_TIME            (0x5A)
#define MAC_SECURITY_ENABLED              (0x5D)

#define MAC_AUTO_REQUEST_SECURITY_LEVEL   (0x78)
#define MAC_AUTO_REQUEST_KEY_ID_MODE      (0x79)

#define NS_IEEE_ADDRESS                   (0xFF) /* Non-standard IEEE address */

/* MAC Address Mode Definitions */
#define MAC_MODE_NO_ADDR                ((unsigned int)0x00)
#define MAC_MODE_SHORT_ADDR             ((unsigned int)0x02)
#define MAC_MODE_LONG_ADDR              ((unsigned int)0x03)

/* MAC constants */
#define MAX_PHY_PACKET_SIZE            (127)
#define MAX_BEACON_OVERHEAD            (75)
#define MAX_BEACON_PAYLOAD_LENGTH      (MAX_PHY_PACKET_SIZE-MAX_BEACON_OVERHEAD)

#define MAX_ATTRIBUTE_SIZE              (250)
#define MAX_DATA_SIZE                   (114)

#define CA8210_VALID_CHANNELS                 (0x07FFF800)

/* MAC workarounds for V1.1 and MPW silicon (V0.x) */
#define CA8210_MAC_WORKAROUNDS (0)
#define CA8210_MAC_MPW         (0)

/* memory manipulation macros */
#define LS_BYTE(x)     ((uint8_t)((x)&0xFF))
#define MS_BYTE(x)     ((uint8_t)(((x)>>8)&0xFF))

/* message ID codes in SPI commands */
/* downstream */
#define MCPS_DATA_REQUEST                     (0x00)
#define MLME_ASSOCIATE_REQUEST                (0x02)
#define MLME_ASSOCIATE_RESPONSE               (0x03)
#define MLME_DISASSOCIATE_REQUEST             (0x04)
#define MLME_GET_REQUEST                      (0x05)
#define MLME_ORPHAN_RESPONSE                  (0x06)
#define MLME_RESET_REQUEST                    (0x07)
#define MLME_RX_ENABLE_REQUEST                (0x08)
#define MLME_SCAN_REQUEST                     (0x09)
#define MLME_SET_REQUEST                      (0x0A)
#define MLME_START_REQUEST                    (0x0B)
#define MLME_POLL_REQUEST                     (0x0D)
#define HWME_SET_REQUEST                      (0x0E)
#define HWME_GET_REQUEST                      (0x0F)
#define TDME_SETSFR_REQUEST                   (0x11)
#define TDME_GETSFR_REQUEST                   (0x12)
#define TDME_SET_REQUEST                      (0x14)
/* upstream */
#define MCPS_DATA_INDICATION                  (0x00)
#define MCPS_DATA_CONFIRM                     (0x01)
#define MLME_RESET_CONFIRM                    (0x0A)
#define MLME_SET_CONFIRM                      (0x0E)
#define MLME_START_CONFIRM                    (0x0F)
#define HWME_SET_CONFIRM                      (0x12)
#define HWME_GET_CONFIRM                      (0x13)
#define HWME_WAKEUP_INDICATION		      (0x15)
#define TDME_SETSFR_CONFIRM                   (0x17)


/* SPI command IDs */
/* bit indicating a confirm or indication from slave to master */
#define SPI_S2M                            (0x20)
/* bit indicating a synchronous message */
#define SPI_SYN                            (0x40)

/* SPI command definitions */
#define SPI_IDLE                           (0xFF)
#define SPI_NACK                           (0xF0)

#define SPI_MCPS_DATA_REQUEST              (MCPS_DATA_REQUEST)
#define SPI_MCPS_DATA_INDICATION           (MCPS_DATA_INDICATION+SPI_S2M)
#define SPI_MCPS_DATA_CONFIRM              (MCPS_DATA_CONFIRM+SPI_S2M)

#define SPI_MLME_ASSOCIATE_REQUEST         (MLME_ASSOCIATE_REQUEST)
#define SPI_MLME_RESET_REQUEST             (MLME_RESET_REQUEST+SPI_SYN)
#define SPI_MLME_SET_REQUEST               (MLME_SET_REQUEST+SPI_SYN)
#define SPI_MLME_START_REQUEST             (MLME_START_REQUEST+SPI_SYN)
#define SPI_MLME_RESET_CONFIRM             (MLME_RESET_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_MLME_SET_CONFIRM               (MLME_SET_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_MLME_START_CONFIRM             (MLME_START_CONFIRM+SPI_S2M+SPI_SYN)

#define SPI_HWME_SET_REQUEST               (HWME_SET_REQUEST+SPI_SYN)
#define SPI_HWME_GET_REQUEST               (HWME_GET_REQUEST+SPI_SYN)
#define SPI_HWME_SET_CONFIRM               (HWME_SET_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_HWME_GET_CONFIRM               (HWME_GET_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_HWME_WAKEUP_INDICATION         (HWME_WAKEUP_INDICATION+SPI_S2M)

#define SPI_TDME_SETSFR_REQUEST            (TDME_SETSFR_REQUEST+SPI_SYN)
#define SPI_TDME_SET_REQUEST               (TDME_SET_REQUEST+SPI_SYN)
#define SPI_TDME_SETSFR_CONFIRM            (TDME_SETSFR_CONFIRM+SPI_S2M+SPI_SYN)

/* TDME SFR addresses */
/* Page 0 */
#define CA8210_SFR_PACFG                   (0xB1)
#define CA8210_SFR_MACCON                  (0xD8)
#define CA8210_SFR_PACFGIB                 (0xFE)
/* Page 1 */
#define CA8210_SFR_LOTXCAL                 (0xBF)
#define CA8210_SFR_PTHRH                   (0xD1)
#define CA8210_SFR_PRECFG                  (0xD3)
#define CA8210_SFR_LNAGX40                 (0xE1)
#define CA8210_SFR_LNAGX41                 (0xE2)
#define CA8210_SFR_LNAGX42                 (0xE3)
#define CA8210_SFR_LNAGX43                 (0xE4)
#define CA8210_SFR_LNAGX44                 (0xE5)
#define CA8210_SFR_LNAGX45                 (0xE6)
#define CA8210_SFR_LNAGX46                 (0xE7)
#define CA8210_SFR_LNAGX47                 (0xE9)

#define CA8210_IOCTL_HARD_RESET            (0x00)

/* Structs/Enums */

/**
 * struct cas_control - spi transfer structure
 * @tx_msg:               spi_message for each downstream exchange
 * @rx_msg:               spi_message for each upstream exchange
 * @tx_transfer:          spi_transfer for each downstream exchange
 * @rx_transfer:          spi_transfer for each upstream exchange
 * @tx_buf:               source array for transmission
 * @tx_in_buf:            array storing bytes received during transmission
 * @rx_buf:               destination array for reception
 * @rx_out_buf:           array storing bytes to present downstream during
 *                        reception
 * @rx_final_buf:         destination array for finished receive packet
 * @spi_mutex:            mutex protecting spi interface
 *
 * This structure stores all the necessary data passed around during spi
 * exchange for a single device.
 */
struct cas_control {
	struct spi_message tx_msg, rx_msg;
	struct spi_transfer tx_transfer, rx_transfer;

	uint8_t *tx_buf;
	uint8_t *tx_in_buf;
	uint8_t *rx_buf;
	uint8_t *rx_out_buf;
	uint8_t *rx_final_buf;

	struct mutex spi_mutex;
};

/**
 * struct ca8210_test - ca8210 test interface structure
 * @ca8210_dfs_spi_int: pointer to the entry in the debug fs for this device
 * @up_fifo:            fifo for upstream messages
 *
 * This structure stores all the data pertaining to the debug interface
 */
struct ca8210_test {
	struct dentry *ca8210_dfs_spi_int;
	struct kfifo up_fifo;
	wait_queue_head_t readq;
};

/**
 * struct ca8210_priv - ca8210 private data structure
 * @spi:                    pointer to the ca8210 spi device object
 * @hw:                     pointer to the ca8210 ieee802154_hw object
 * @hw_registered:          true if hw has been registered with ieee802154
 * @lock:                   spinlock protecting the private data area
 * @async_tx_workqueue:     workqueue for asynchronous transmission
 * @mlme_workqueue:           workqueue for triggering MLME Reset
 * @irq_workqueue:          workqueue for irq processing
 * @async_tx_work:          work object for a single asynchronous transmission
 * @async_tx_timeout_work:  delayed work object for a single asynchronous
 *                           transmission timeout
 * @tx_skb:                 current socket buffer to transmit
 * @nextmsduhandle:         msdu handle to pass to the 15.4 MAC layer for the
 *                           next transmission
 * @clk:                    external clock provided by the ca8210
 * @cas_ctl:                spi control data section for this instance
 * @last_dsn:               sequence number of last data packet received, for
 *                           resend detection
 * @test:                   test interface data section for this instance
 * @async_tx_pending:       true if an asynchronous transmission was started and
 *                           is not complete
 * @sync_tx_pending:        true if a synchronous (from driver perspective)
 *                           transmission was started and is not complete
 * @sync_command_pending:   true if waiting for a synchronous (Cascoda API)
 *                           response
 * @sync_command_mutex:     mutex controlling access to sync command objects
 * @sync_command_response:  pointer to buffer to fill with sync response
 * @ca8210_is_awake:        nonzero if ca8210 is initialised, ready for comms
 * @sync_down:              counts number of downstream synchronous commands
 * @sync_up:                counts number of upstream synchronous commands
 *
 */
struct ca8210_priv {
	struct spi_device *spi;
	struct ieee802154_hw *hw;
	bool hw_registered;
	spinlock_t lock;
	struct workqueue_struct *async_tx_workqueue, *mlme_workqueue;
	struct workqueue_struct *irq_workqueue;
	struct work_struct async_tx_work;
	struct delayed_work async_tx_timeout_work;
	struct sk_buff *tx_skb;
	uint8_t nextmsduhandle;
	struct clk *clk;
	struct cas_control cas_ctl;
	int last_dsn;
	struct ca8210_test test;
	bool async_tx_pending, sync_tx_pending;
	bool sync_command_pending;
	struct mutex sync_command_mutex;
	uint8_t *sync_command_response;
	atomic_t ca8210_is_awake;
	int sync_down, sync_up;
	int spi_errno;
};

/**
 * struct work_priv_container - link between a work object and the relevant
 *                              device's private data
 * @work: work object being executed
 * @priv: device's private data section
 *
 */
struct work_priv_container {
	struct work_struct work;
	struct ca8210_priv *priv;
};

/**
 * struct ca8210_platform_data - ca8210 platform data structure
 * @extclockenable: true if the external clock is to be enabled
 * @extclockfreq:   frequency of the external clock
 * @extclockgpio:   ca8210 output gpio of the external clock
 * @gpio_reset:     gpio number of ca8210 reset line
 * @gpio_irq:       gpio number of ca8210 interrupt line
 * @irq_id:         identifier for the ca8210 irq
 *
 */
struct ca8210_platform_data {
	bool extclockenable;
	unsigned int extclockfreq;
	unsigned int extclockgpio;
	int gpio_reset;
	int gpio_irq;
	int irq_id;
};

/**
 * struct fulladdr - full MAC addressing information structure
 * @mode:    address mode (none, short, extended)
 * @pan_id:  16-bit LE pan id
 * @address: LE address, variable length as specified by mode
 *
 */
struct fulladdr {
	uint8_t         mode;
	uint8_t         pan_id[2];
	uint8_t         address[8];
};

/**
 * union macaddr: generic MAC address container
 * @short_addr:   16-bit short address
 * @ieee_address: 64-bit extended address as LE byte array
 *
 */
union macaddr {
	uint16_t        short_address;
	uint8_t         ieee_address[8];
};

/**
 * struct secspec: security specification for SAP commands
 * @security_level: 0-7, controls level of authentication & encryption
 * @key_id_mode:    0-3, specifies how to obtain key
 * @key_source:     extended key retrieval data
 * @key_index:      single-byte key identifier
 *
 */
struct secspec {
	uint8_t         security_level;
	uint8_t         key_id_mode;
	uint8_t         key_source[8];
	uint8_t         key_index;
};

/* downlink functions parameter set definitions */
struct mcps_data_request_pset {
	uint8_t         src_addr_mode;
	struct fulladdr dst;
	uint8_t         msdu_length;
	uint8_t         msdu_handle;
	uint8_t         tx_options;
	uint8_t         msdu[MAX_DATA_SIZE];
};

struct mlme_set_request_pset {
	uint8_t         pib_attribute;
	uint8_t         pib_attribute_index;
	uint8_t         pib_attribute_length;
	uint8_t         pib_attribute_value[MAX_ATTRIBUTE_SIZE];
};

struct hwme_set_request_pset {
	uint8_t         hw_attribute;
	uint8_t         hw_attribute_length;
	uint8_t         hw_attribute_value[MAX_HWME_ATTRIBUTE_SIZE];
};

struct hwme_get_request_pset {
	uint8_t         hw_attribute;
};

struct tdme_setsfr_request_pset {
	uint8_t         sfr_page;
	uint8_t         sfr_address;
	uint8_t         sfr_value;
};

/* uplink functions parameter set definitions */
struct hwme_set_confirm_pset {
	uint8_t         status;
	uint8_t         hw_attribute;
};

struct hwme_get_confirm_pset {
	uint8_t         status;
	uint8_t         hw_attribute;
	uint8_t         hw_attribute_length;
	uint8_t         hw_attribute_value[MAX_HWME_ATTRIBUTE_SIZE];
};

struct tdme_setsfr_confirm_pset {
	uint8_t         status;
	uint8_t         sfr_page;
	uint8_t         sfr_address;
};

struct mac_message {
	uint8_t      command_id;
	uint8_t      length;
	union {
		struct mcps_data_request_pset       data_req;
		struct mlme_set_request_pset        set_req;
		struct hwme_set_request_pset        hwme_set_req;
		struct hwme_get_request_pset        hwme_get_req;
		struct tdme_setsfr_request_pset     tdme_set_sfr_req;
		struct hwme_set_confirm_pset        hwme_set_cnf;
		struct hwme_get_confirm_pset        hwme_get_cnf;
		struct tdme_setsfr_confirm_pset     tdme_set_sfr_cnf;
		uint8_t                             u8param;
		uint8_t                             status;
		uint8_t                             payload[254];
	} pdata;
};

static int (*cascoda_api_upstream)(
	const uint8_t *buf,
	size_t len,
	void *device_ref
);

/**
 * link_to_linux_err() - Translates an 802.15.4 return code into the closest
 *                       linux error
 * @link_status:  802.15.4 status code
 *
 * Return: 0 or Linux error code
 */
static int link_to_linux_err(int link_status)
{
	if (link_status < 0) {
		/* status is already a Linux code */
		return link_status;
	}
	switch (link_status) {
	case MAC_SUCCESS:
	case MAC_REALIGNMENT:
		return 0;
	case MAC_IMPROPER_KEY_TYPE:
		return -EKEYREJECTED;
	case MAC_IMPROPER_SECURITY_LEVEL:
	case MAC_UNSUPPORTED_LEGACY:
	case MAC_DENIED:
		return -EACCES;
	case MAC_BEACON_LOST:
	case MAC_NO_ACK:
	case MAC_NO_BEACON:
		return -ENETUNREACH;
	case MAC_CHANNEL_ACCESS_FAILURE:
	case MAC_TX_ACTIVE:
	case MAC_SCAN_IN_PROGRESS:
		return -EBUSY;
	case MAC_DISABLE_TRX_FAILURE:
	case MAC_OUT_OF_CAP:
		return -EAGAIN;
	case MAC_FRAME_TOO_LONG:
		return -EMSGSIZE;
	case MAC_INVALID_GTS:
	case MAC_PAST_TIME:
		return -EBADSLT;
	case MAC_INVALID_HANDLE:
		return -EBADMSG;
	case MAC_INVALID_PARAMETER:
	case MAC_UNSUPPORTED_ATTRIBUTE:
	case MAC_ON_TIME_TOO_LONG:
	case MAC_INVALID_INDEX:
		return -EINVAL;
	case MAC_NO_DATA:
		return -ENODATA;
	case MAC_NO_SHORT_ADDRESS:
		return -EFAULT;
	case MAC_PAN_ID_CONFLICT:
		return -EADDRINUSE;
	case MAC_TRANSACTION_EXPIRED:
		return -ETIME;
	case MAC_TRANSACTION_OVERFLOW:
		return -ENOBUFS;
	case MAC_UNAVAILABLE_KEY:
		return -ENOKEY;
	case MAC_INVALID_ADDRESS:
		return -ENXIO;
	case MAC_TRACKING_OFF:
	case MAC_SUPERFRAME_OVERLAP:
		return -EREMOTEIO;
	case MAC_LIMIT_REACHED:
		return -EDQUOT;
	case MAC_READ_ONLY:
		return -EROFS;
	default:
		return -EPROTO;
	}
}

/**
 * ca8210_test_int_driver_write() - Writes a message to the test interface to be
 *                                  read by the userspace
 * @buf:  Buffer containing upstream message
 * @len:  length of message to write
 * @spi:  SPI device of message originator
 *
 * Return: 0 or linux error code
 */
static int ca8210_test_int_driver_write(
	const uint8_t  *buf,
	size_t          len,
	void           *spi
)
{
	struct ca8210_priv *priv = spi_get_drvdata(spi);
	struct ca8210_test *test = &priv->test;
	char *fifo_buffer;
	int i;

	dev_dbg(
		&priv->spi->dev,
		"test_interface: Buffering upstream message:\n"
	);
	for (i = 0; i < len; i++) {
		dev_dbg(&priv->spi->dev, "%#03x\n", buf[i]);
	}

	fifo_buffer = kmalloc(len, GFP_KERNEL);
	memcpy(fifo_buffer, buf, len);
	kfifo_in(&test->up_fifo, &fifo_buffer, 4);

	return 0;
}

/* SPI Operation */

static int ca8210_spi_write_dummy(struct spi_device *spi);
static int ca8210_net_rx(
	struct ieee802154_hw  *hw,
	uint8_t               *command,
	size_t                 len
);
static uint8_t mlme_reset_request_sync(
	uint8_t  set_default_pib,
	void    *device_ref
);

/**
 * ca8210_reset_send() - Hard resets the ca8210 for a given time
 * @spi:  Pointer to target ca8210 spi device
 * @ms:   Milliseconds to hold the reset line low for
 */
static void ca8210_reset_send(struct spi_device *spi, unsigned int ms)
{
	struct ca8210_platform_data *pdata = spi->dev.platform_data;
	struct ca8210_priv *priv = spi_get_drvdata(spi);
	unsigned long startjiffies;

	#define RESET_OFF 0
	#define RESET_ON 1

	gpio_set_value(pdata->gpio_reset, RESET_OFF);
	atomic_set(&priv->ca8210_is_awake, 0);
	msleep(ms);
	gpio_set_value(pdata->gpio_reset, RESET_ON);

	/* Wait until wakeup indication seen */
	startjiffies = jiffies;
	while (atomic_read(&priv->ca8210_is_awake) == 0) {
		if (jiffies - startjiffies >
		    msecs_to_jiffies(CA8210_SYNC_TIMEOUT)) {
			dev_crit(
				&spi->dev,
				"Fatal: No wakeup from ca8210 after reset!\n"
			);
			break;
		}
		msleep(1);
	}

	dev_dbg(&spi->dev, "Reset the device\n");
	#undef RESET_OFF
	#undef RESET_ON
}

/**
 * ca8210_mlme_reset_worker() - Resets the MLME, Called when the MAC OVERFLOW
 *                              condition happens.
 * @work:  Pointer to work being executed
 */
static void ca8210_mlme_reset_worker(struct work_struct *work)
{
	struct work_priv_container *wpc = container_of(
		work,
		struct work_priv_container,
		work
	);

	struct ca8210_priv *priv = wpc->priv;
	mlme_reset_request_sync(0, priv->spi);
	kfree(wpc);
}

/**
 * ca8210_rx_done() - Calls various message dispatches responding to a received
 *                    command
 * @arg:  Pointer to the drivers private data structure
 *
 * Presents a received SAP command from the ca8210 to the Cascoda EVBME, test
 * interface and network driver.
 */
static void ca8210_rx_done(struct ca8210_priv *priv)
{
	uint8_t buf[CA8210_SPI_BUF_SIZE];
	uint8_t len;
	unsigned long flags;
	struct work_priv_container *mlme_reset_wpc;


	spin_lock_irqsave(&priv->lock, flags);

	len = priv->cas_ctl.rx_final_buf[1] + 2;
	if (len > CA8210_SPI_BUF_SIZE) {
		dev_crit(
			&priv->spi->dev,
			"Received packet len (%d) erroneously long\n",
			len
		);
		memset(
			priv->cas_ctl.rx_final_buf,
			SPI_IDLE,
			CA8210_SPI_BUF_SIZE
		);
		spin_unlock_irqrestore(&priv->lock, flags);
		return;
	}

	memcpy(buf, priv->cas_ctl.rx_final_buf, len);
	memset(priv->cas_ctl.rx_final_buf, SPI_IDLE, CA8210_SPI_BUF_SIZE);

	spin_unlock_irqrestore(&priv->lock, flags);

	if (buf[0] & SPI_SYN) {
		if (mutex_lock_interruptible(&priv->sync_command_mutex)) {
			return;
		}
		if (priv->sync_command_pending) {
			if (priv->sync_command_response == NULL) {
				priv->sync_command_pending = false;
				mutex_unlock(&priv->sync_command_mutex);
				dev_crit(
					&priv->spi->dev,
					"Sync command provided no response " \
					"buffer\n"
				);
				return;
			}
			memcpy(priv->sync_command_response, buf, len);
			priv->sync_command_pending = false;
			mutex_unlock(&priv->sync_command_mutex);
		} else {
			mutex_unlock(&priv->sync_command_mutex);
			if (cascoda_api_upstream != NULL)
				cascoda_api_upstream(buf, len, priv->spi);
			priv->sync_up++;
		}
	} else {
		if (cascoda_api_upstream != NULL)
			cascoda_api_upstream(buf, len, priv->spi);
	}

	ca8210_net_rx(priv->hw, buf, len);
	if (buf[0] == SPI_MCPS_DATA_CONFIRM) {
		if (buf[3] == MAC_TRANSACTION_OVERFLOW) {
			dev_info(
				&priv->spi->dev,
				"Waiting for transaction overflow to " \
				"stabilise...\n");
			msleep(2000);
			dev_info(
				&priv->spi->dev,
				"Resetting MAC...\n");

			mlme_reset_wpc = kmalloc(
				sizeof(struct work_priv_container),
				GFP_KERNEL
			);
			INIT_WORK(
				&mlme_reset_wpc->work,
				ca8210_mlme_reset_worker
			);
			mlme_reset_wpc->priv = priv;
			queue_work(priv->mlme_workqueue, &mlme_reset_wpc->work);
		}
	} else if (buf[0] == SPI_HWME_WAKEUP_INDICATION) {
		dev_notice(
			&priv->spi->dev,
			"Wakeup indication received, reason:\n"
		);
		switch (buf[2]) {
		case 0:
			dev_notice(
				&priv->spi->dev,
				"Transceiver woken up from Power Up / " \
				"System Reset\n"
			);
			break;
		case 1:
			dev_notice(
				&priv->spi->dev,
				"Watchdog Timer Time-Out\n"
			);
			break;
		case 2:
			dev_notice(
				&priv->spi->dev,
				"Transceiver woken up from Power-Off by " \
				"Sleep Timer Time-Out\n");
			break;
		case 3:
			dev_notice(
				&priv->spi->dev,
				"Transceiver woken up from Power-Off by " \
				"GPIO Activity\n"
			);
			break;
		case 4:
			dev_notice(
				&priv->spi->dev,
				"Transceiver woken up from Standby by " \
				"Sleep Timer Time-Out\n"
			);
			break;
		case 5:
			dev_notice(
				&priv->spi->dev,
				"Transceiver woken up from Standby by " \
				"GPIO Activity\n"
			);
			break;
		case 6:
			dev_notice(
				&priv->spi->dev,
				"Sleep-Timer Time-Out in Active Mode\n"
			);
			break;
		default:
			dev_warn(&priv->spi->dev, "Wakeup reason unknown\n");
			break;
		}
		spin_lock_irqsave(&priv->lock, flags);
		atomic_inc(&priv->ca8210_is_awake);
		spin_unlock_irqrestore(&priv->lock, flags);
	}

	return;
}

/**
 * ca8210_spi_read() - Read a SAP command from ca8210
 * @spi:  Pointer to spi device to read from
 *
 * Reads a pending SAP command. Dispatches a workqueue to ca8210_rx_done with
 * the received packet.
 */
static int ca8210_spi_read(struct spi_device *spi)
{
	int status, i;
	struct ca8210_priv *priv = spi_get_drvdata(spi);
	unsigned long flags;

	dev_dbg(&spi->dev, "ca8210_spi_read called\n");

	do {
		spin_lock_irqsave(&priv->lock, flags);
		if (priv->cas_ctl.rx_final_buf[0] == SPI_IDLE) {
			/* spi receive buffer cleared of last rx */
			spin_unlock_irqrestore(&priv->lock, flags);
			break;
		}
		/* spi receive buffer still in use */
		spin_unlock_irqrestore(&priv->lock, flags);
		msleep(1);
	} while (1);


	memset(priv->cas_ctl.rx_buf, SPI_IDLE, CA8210_SPI_BUF_SIZE);
	memset(priv->cas_ctl.rx_out_buf, SPI_IDLE, CA8210_SPI_BUF_SIZE);

	/* Read the first 2 bytes: CMD and LENGTH */
	priv->cas_ctl.rx_transfer.tx_buf = priv->cas_ctl.rx_out_buf;
	priv->cas_ctl.rx_transfer.rx_buf = priv->cas_ctl.rx_buf;
	priv->cas_ctl.rx_transfer.len = 2;
	/* Keep chip select asserted after reading the bytes */
	priv->cas_ctl.rx_transfer.cs_change = 1;
	priv->cas_ctl.rx_transfer.delay_usecs = 0;
	spi_message_init(&priv->cas_ctl.rx_msg);
	spi_message_add_tail(&priv->cas_ctl.rx_transfer, &priv->cas_ctl.rx_msg);

	status = spi_sync(spi, &priv->cas_ctl.rx_msg);
	if (status) {
		dev_crit(
			&spi->dev,
			"status %d from spi_sync in read\n",
			status
		);
		goto error;
	}

	dev_dbg(
		&spi->dev,
		"spi received cmdid: %d, len: %d\n",
		priv->cas_ctl.rx_buf[0],
		priv->cas_ctl.rx_buf[1]
	);

	if (priv->cas_ctl.rx_buf[0] == SPI_IDLE) {
		status = -EBADE;
		goto error;
	}
	else if (priv->cas_ctl.rx_buf[0] == SPI_NACK) {
		status = -EBUSY;
		goto error;
	}

	priv->cas_ctl.rx_final_buf[0] = priv->cas_ctl.rx_buf[0];
	priv->cas_ctl.rx_final_buf[1] = priv->cas_ctl.rx_buf[1];

	spi_message_init(&priv->cas_ctl.rx_msg);

	priv->cas_ctl.rx_transfer.tx_buf = priv->cas_ctl.rx_out_buf;
	priv->cas_ctl.rx_transfer.rx_buf = priv->cas_ctl.rx_buf;
	priv->cas_ctl.rx_transfer.len = priv->cas_ctl.rx_final_buf[1];
	priv->cas_ctl.rx_transfer.cs_change = 0;
	priv->cas_ctl.rx_transfer.delay_usecs = 0;

	spi_message_add_tail(&priv->cas_ctl.rx_transfer, &priv->cas_ctl.rx_msg);

	status = spi_sync(spi, &priv->cas_ctl.rx_msg);

	if (status) {
		dev_crit(
			&spi->dev,
			"status %d from spi_sync in read\n",
			status
		);
		goto error;
	}

	spin_lock_irqsave(&priv->lock, flags);

	for (i = 0; i < priv->cas_ctl.rx_final_buf[1]; i++) {
		priv->cas_ctl.rx_final_buf[2+i] = priv->cas_ctl.rx_buf[i];
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(
		&spi->dev,
		"device_comm: command ID = %#03x length = %#03x Data:",
		priv->cas_ctl.rx_final_buf[0],
		priv->cas_ctl.rx_final_buf[1]
	);

	for (i = 2; i < priv->cas_ctl.rx_final_buf[1] + 2; i++) {
		dev_dbg(&spi->dev, "%#03x\n", priv->cas_ctl.rx_final_buf[i]);
	}


	ca8210_rx_done(priv);
	return 0;

error:
	ca8210_spi_write_dummy(spi);
	return status;
}

/**
 * ca8210_spi_write() - Write a message to the ca8210 over spi
 * @spi: Pointer to spi device to write to
 * @buf: Octet array to send
 * @len: length of the buffer being sent
 *
 * Return: 0 or linux error code
 */
static int ca8210_spi_write(
	struct spi_device  *spi,
	const uint8_t      *buf,
	size_t              len
)
{
	int status = 0;
	int i;
	bool dummy, duplex_rx = false;
	struct ca8210_priv *priv = spi_get_drvdata(spi);
	unsigned long flags;
	int payload_len = 0;

	if (spi == NULL) {
		dev_crit(
			&spi->dev,
			"NULL spi device passed to ca8210_spi_write\n"
		);
		return -ENODEV;
	}

	if (buf[0] == SPI_IDLE && len == 1) {
		dummy = true;
	} else {
		dummy = false;

		/* Set in/out buffers to idle, copy over data to send */
		memset(priv->cas_ctl.tx_buf, SPI_IDLE, CA8210_SPI_BUF_SIZE);
		memset(priv->cas_ctl.tx_in_buf, SPI_IDLE, CA8210_SPI_BUF_SIZE);
		memcpy(priv->cas_ctl.tx_buf, buf, len);

		dev_dbg(&spi->dev, "ca8210_spi_write called\n");

		dev_dbg(
			&spi->dev,
			"device_comm: command ID = %#03x length = %#03x Data:\n",
			priv->cas_ctl.tx_buf[0],
			priv->cas_ctl.tx_buf[1]
		);

		for (i = 2; i < len; i++) {
			dev_dbg(&spi->dev, "%#03x\n", priv->cas_ctl.tx_buf[i]);
		}
	}

	spi_message_init(&priv->cas_ctl.tx_msg);

	priv->cas_ctl.tx_transfer.tx_buf = priv->cas_ctl.tx_buf;
	priv->cas_ctl.tx_transfer.rx_buf = priv->cas_ctl.tx_in_buf;
	priv->cas_ctl.tx_transfer.delay_usecs = 0;

	if (!dummy) {
		/* Regular transmission, keep CS asserted in case of
		 * incomplete concurrent read
		 */
		priv->cas_ctl.tx_transfer.cs_change = 1;
		priv->cas_ctl.tx_transfer.len = 2;
	} else {
		/* dummy transmission, de-assert CS */
		priv->cas_ctl.tx_transfer.cs_change = 0;
		priv->cas_ctl.tx_transfer.len = len;
	}

	spi_message_add_tail(
		&priv->cas_ctl.tx_transfer,
		&priv->cas_ctl.tx_msg
	);

	status = spi_sync(spi, &priv->cas_ctl.tx_msg);
	if (status < 0) {
		dev_crit(
			&spi->dev,
			"status %d from spi_sync in write\n",
			status
		);
	} else if (!dummy
	        && priv->cas_ctl.tx_in_buf[0] == SPI_NACK
	        && priv->cas_ctl.tx_in_buf[1] == SPI_NACK) {
		/* ca8210 is busy */
		dev_info(&spi->dev, "ca8210 was busy during attempted write\n");
		ca8210_spi_write_dummy(spi);
		return -EBUSY;
	} else if (!dummy) {
		if (priv->cas_ctl.tx_in_buf[0] != SPI_IDLE &&
		    priv->cas_ctl.tx_in_buf[0] != SPI_NACK) {
			duplex_rx = true;
			do {
				spin_lock_irqsave(&priv->lock, flags);
				if (priv->cas_ctl.rx_final_buf[0] == SPI_IDLE) {
					/* spi receive buffer cleared of last
					 * rx */
					spin_unlock_irqrestore(
						&priv->lock,
						flags
					);
					break;
				}
				/* spi receive buffer still in use */
				spin_unlock_irqrestore(&priv->lock, flags);
				msleep(1);
			} while (1);
			priv->cas_ctl.rx_final_buf[0] =
				priv->cas_ctl.tx_in_buf[0];
			priv->cas_ctl.rx_final_buf[1] =
				priv->cas_ctl.tx_in_buf[1];
		}
		spi_message_init(&priv->cas_ctl.tx_msg);

		priv->cas_ctl.tx_transfer.tx_buf = priv->cas_ctl.tx_buf + 2;
		priv->cas_ctl.tx_transfer.rx_buf = priv->cas_ctl.tx_in_buf + 2;
		priv->cas_ctl.tx_transfer.delay_usecs = 0;
		priv->cas_ctl.tx_transfer.cs_change = 0;

		if (duplex_rx
		 && priv->cas_ctl.tx_in_buf[1] > priv->cas_ctl.tx_buf[1]) {
			payload_len = priv->cas_ctl.tx_in_buf[1];
		} else {
			payload_len = priv->cas_ctl.tx_buf[1];
		}
		priv->cas_ctl.tx_transfer.len = payload_len;
		spi_message_add_tail(
			&priv->cas_ctl.tx_transfer,
			&priv->cas_ctl.tx_msg
		);

		status = spi_sync(spi, &priv->cas_ctl.tx_msg);
		if (status < 0) {
			dev_crit(
				&spi->dev,
				"status %d from spi_sync in write\n",
				status
			);
		}
	}

	if (dummy)
		return status;

	dev_dbg(&spi->dev, "spi received during transfer:\n");
	for (i = 0; i < payload_len + 2; i++) {
		dev_dbg(&spi->dev, "%#03x\n", priv->cas_ctl.tx_in_buf[i]);
	}

	if (duplex_rx) {
		dev_dbg(&spi->dev, "READ CMD DURING TX\n");
		memcpy(
			priv->cas_ctl.rx_final_buf + 2,
			priv->cas_ctl.tx_in_buf + 2,
			priv->cas_ctl.rx_final_buf[1]
		);
		ca8210_rx_done(priv);
	}
	return status;
}

/**
 * ca8210_spi_write_dummy() - Write a "dummy" packet to the ca8210
 * @spi:  Pointer to spi device to write to
 *
 * This functions exists solely to toggle the spi chip select to the ca8210. The
 * ca8210 sends and receives variable length spi packets to its host processor.
 * This means the chip select must be held asserted inbetween processing
 * individual bytes of messages. Using the current spi framework the only way to
 * toggle the chip select is through an spi transfer so this functions writes
 * an "IDLE" 0xFF byte to the ca8210 for the sole purpose of de-asserting the
 * chip select when an exchange is complete.
 *
 * Return: 0 or linux error code
 */
static int ca8210_spi_write_dummy(struct spi_device *spi)
{
	int ret;
	uint8_t idle = SPI_IDLE;

	dev_dbg(&spi->dev, "spi: writing dummy packet\n");
	ret =  ca8210_spi_write(spi, &idle, 1);
	dev_dbg(&spi->dev, "spi: wrote dummy packet\n");
	return ret;
}

/**
 * ca8210_spi_exchange() - Exchange API/SAP commands with the radio
 * @buf:         Octet array of command being sent downstream
 * @len:         length of buf
 * @response:    buffer for storing synchronous response
 * @device_ref:  spi_device pointer for ca8210
 *
 * Effectively calls ca8210_spi_write to write buf[] to the spi, then for
 * synchronous commands waits for the corresponding response to be read from
 * the spi before returning. The response is written to the response parameter.
 *
 * Return: 0 or linux error code
 */
static int ca8210_spi_exchange(
	const uint8_t *buf,
	size_t len,
	uint8_t *response,
	void *device_ref
)
{
	int status = 0;
	unsigned long startjiffies, currentjiffies;
	struct spi_device *spi = device_ref;
	struct ca8210_priv *priv = spi->dev.driver_data;
	int write_retries = 0;

	if (mutex_lock_interruptible(&priv->cas_ctl.spi_mutex)) {
		priv->spi_errno = status;
		return -ERESTARTSYS;
	}

	if ((buf[0] & SPI_SYN) && response) { /* if sync lock mutex */
		if (mutex_lock_interruptible(&priv->sync_command_mutex)) {
			status = -ERESTARTSYS;
			goto cleanup;
		}
	}

	do {
		status = ca8210_spi_write(priv->spi, buf, len);
		if (status >= 0) {
			continue;
		}
		if (status == -EBUSY) {
			msleep(1);
			write_retries++;
			if (write_retries > 20) {
				dev_err(
					&spi->dev,
					"too many retries!\n"
				);
				if (((buf[0] & SPI_SYN) && response)) {
					mutex_unlock(&priv->sync_command_mutex);
				}
				status = -EAGAIN;
				goto cleanup;
			}
			dev_info(
				&spi->dev,
				"spi write retry %d...\n",
				write_retries
			);
		} else {
			dev_warn(
				&spi->dev,
				"spi write failed, returned %d\n",
				status
			);
			if (((buf[0] & SPI_SYN) && response)) {
				mutex_unlock(&priv->sync_command_mutex);
			}
			goto cleanup;
		}
	} while (status < 0);
	mutex_unlock(&priv->cas_ctl.spi_mutex);

	if (!((buf[0] & SPI_SYN) && response)) {
		priv->spi_errno = status;
		return status;
	}

	/* if sync wait for confirm */
	priv->sync_command_response = response;
	priv->sync_command_pending = true;
	mutex_unlock(&priv->sync_command_mutex);
	startjiffies = jiffies;
	while (1) {
		if (mutex_lock_interruptible(
			&priv->sync_command_mutex)) {
			status = -ERESTARTSYS;
			break;
		}
		if (!priv->sync_command_pending) {
			priv->sync_command_response = NULL;
			mutex_unlock(&priv->sync_command_mutex);
			break;
		}
		mutex_unlock(&priv->sync_command_mutex);
		currentjiffies = jiffies;
		if ((currentjiffies - startjiffies) >
		    msecs_to_jiffies(CA8210_SYNC_TIMEOUT)) {
			dev_err(
				&spi->dev,
				"Synchronous confirm timeout\n"
			);
			status = -ETIME;
			break;
		}
	}
	priv->spi_errno = status;
	return status;

cleanup:
	mutex_unlock(&priv->cas_ctl.spi_mutex);
	priv->spi_errno = status;
	return status;
}

/**
 * ca8210_irq_worker() - Starts the spi read process after having the work
 *                       handed off by the interrupt handler
 * @work:  Pointer to work being executed
 */
static void ca8210_irq_worker(struct work_struct *work)
{
	struct work_priv_container *wpc = container_of(
		work,
		struct work_priv_container,
		work
	);
	struct ca8210_priv *priv = wpc->priv;
	int status;

	if (mutex_lock_interruptible(&priv->cas_ctl.spi_mutex)) {
		kfree(wpc);
		return;
	}
	do {
		status = ca8210_spi_read(priv->spi);
		if (status < 0) {
			if (status == -EBUSY) {
				goto cleanup;
			} else {
				dev_warn(
					&priv->spi->dev,
					"spi read failed, returned %d\n",
					status
				);
				goto cleanup;
			}
		}
	} while (status < 0);

cleanup:
	mutex_unlock(&priv->cas_ctl.spi_mutex);
	kfree(wpc);
}

/**
 * ca8210_interrupt_handler() - Called when an irq is received from the ca8210
 * @irq:     Id of the irq being handled
 * @dev_id:  Pointer passed by the system, pointing to the ca8210's private data
 *
 * This function is called when the irq line from the ca8210 is asserted,
 * signifying that the ca8210 has a message to send upstream to us. Queues the
 * work of reading this message to be executed in non-atomic context.
 *
 * Return: irq return code
 */
static irqreturn_t ca8210_interrupt_handler(int irq, void *dev_id)
{
	struct ca8210_priv *priv = dev_id;
	struct work_priv_container *irq_wpc;

	dev_dbg(&priv->spi->dev, "irq: Interrupt occured\n");
	irq_wpc = kmalloc(
		sizeof(struct work_priv_container),
		GFP_ATOMIC
	);
	INIT_WORK(&irq_wpc->work, ca8210_irq_worker);
	irq_wpc->priv = priv;
	queue_work(priv->irq_workqueue, &irq_wpc->work);
	return IRQ_HANDLED;
}

static int (*cascoda_api_downstream)(
	const uint8_t *buf,
	size_t len,
	uint8_t *response,
	void *device_ref
) = ca8210_spi_exchange;

/* Cascoda API / 15.4 SAP Primitives */

/**
 * tdme_setsfr_request_sync() - TDME_SETSFR_request/confirm according to API
 * @sfr_page:    SFR Page
 * @sfr_address: SFR Address
 * @sfr_value:   SFR Value
 * @device_ref:  Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of TDME-SETSFR.confirm
 */
static uint8_t tdme_setsfr_request_sync(
	uint8_t      sfr_page,
	uint8_t      sfr_address,
	uint8_t      sfr_value,
	void         *device_ref
)
{
	int ret;
	struct mac_message command, response;
	struct spi_device *spi = device_ref;

	command.command_id = SPI_TDME_SETSFR_REQUEST;
	command.length = 3;
	command.pdata.tdme_set_sfr_req.sfr_page    = sfr_page;
	command.pdata.tdme_set_sfr_req.sfr_address = sfr_address;
	command.pdata.tdme_set_sfr_req.sfr_value   = sfr_value;
	response.command_id = SPI_IDLE;
	ret = cascoda_api_downstream(
		&command.command_id,
		command.length + 2,
		&response.command_id,
		device_ref
	);
	if (ret) {
		dev_crit(&spi->dev, "cascoda_api_downstream returned %d", ret);
		return MAC_SYSTEM_ERROR;
	}

	if (response.command_id != SPI_TDME_SETSFR_CONFIRM) {
		dev_crit(
			&spi->dev,
			"sync response to SPI_TDME_SETSFR_REQUEST was not " \
			"SPI_TDME_SETSFR_CONFIRM, it was %d\n",
			response.command_id
		);
		return MAC_SYSTEM_ERROR;
	}

	return response.pdata.tdme_set_sfr_cnf.status;
}

/**
 * tdme_chipinit() - TDME Chip Register Default Initialisation Macro
 * @device_ref: Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of API calls
 */
static uint8_t tdme_chipinit(void *device_ref)
{
	uint8_t status = MAC_SUCCESS;
	uint8_t sfr_address;
	struct spi_device *spi = device_ref;

	if ((status = tdme_setsfr_request_sync(
		1, (sfr_address = CA8210_SFR_LNAGX40), 0x29, device_ref))
	) /* LNA Gain Settings */
		goto finish;
	if ((status = tdme_setsfr_request_sync(
		1, (sfr_address = CA8210_SFR_LNAGX41), 0x54, device_ref))
	)
		goto finish;
	if ((status = tdme_setsfr_request_sync(
		1, (sfr_address = CA8210_SFR_LNAGX42), 0x6C, device_ref))
	)
		goto finish;
	if ((status = tdme_setsfr_request_sync(
		1, (sfr_address = CA8210_SFR_LNAGX43), 0x7A, device_ref))
	)
		goto finish;
	if ((status = tdme_setsfr_request_sync(
		1, (sfr_address = CA8210_SFR_LNAGX44), 0x84, device_ref))
	)
		goto finish;
	if ((status = tdme_setsfr_request_sync(
		1, (sfr_address = CA8210_SFR_LNAGX45), 0x8B, device_ref))
	)
		goto finish;
	if ((status = tdme_setsfr_request_sync(
		1, (sfr_address = CA8210_SFR_LNAGX46), 0x92, device_ref))
	)
		goto finish;
	if ((status = tdme_setsfr_request_sync(
		1, (sfr_address = CA8210_SFR_LNAGX47), 0x96, device_ref))
	)
		goto finish;
	if ((status = tdme_setsfr_request_sync(
		1, (sfr_address = CA8210_SFR_PRECFG), 0x5B, device_ref))
	) /* Preamble Timing Config */
		goto finish;
	if ((status = tdme_setsfr_request_sync(
		1, (sfr_address = CA8210_SFR_PTHRH), 0x5A, device_ref))
	) /* Preamble Threshold High */
		goto finish;
	if ((status = tdme_setsfr_request_sync(
		0, (sfr_address = CA8210_SFR_PACFGIB), 0x3F, device_ref))
	) /* Tx Output Power 8 dBm */
		goto finish;

finish:
	if (status != MAC_SUCCESS) {
		dev_err(
			&spi->dev,
			"failed to set sfr at %#03x, status = %#03x\n",
			sfr_address,
			status
		);
	}
	return status;
}

/**
 * tdme_channelinit() - TDME Channel Register Default Initialisation Macro (Tx)
 * @channel:    802.15.4 channel to initialise chip for
 * @device_ref: Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of API calls
 */
static uint8_t tdme_channelinit(uint8_t channel, void *device_ref)
{
	uint8_t txcalval;

	if (channel >= 25)
		txcalval = 0xA7;
	else if (channel >= 23)
		txcalval = 0xA8;
	else if (channel >= 22)
		txcalval = 0xA9;
	else if (channel >= 20)
		txcalval = 0xAA;
	else if (channel >= 17)
		txcalval = 0xAB;
	else if (channel >= 16)
		txcalval = 0xAC;
	else if (channel >= 14)
		txcalval = 0xAD;
	else if (channel >= 12)
		txcalval = 0xAE;
	else
		txcalval = 0xAF;

	return tdme_setsfr_request_sync(
		1,
		CA8210_SFR_LOTXCAL,
		txcalval,
		device_ref
	);  /* LO Tx Cal */
}

/**
 * tdme_checkpibattribute() - Checks Attribute Values that are not checked in
 *                            MAC
 * @pib_attribute:        Attribute Number
 * @pib_attribute_length: Attribute length
 * @pib_attribute_value:  Pointer to Attribute Value
 * @device_ref:           Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of checks
 */
static uint8_t tdme_checkpibattribute(
	uint8_t      pib_attribute,
	uint8_t      pib_attribute_length,
	const void   *pib_attribute_value
)
{
	uint8_t status = MAC_SUCCESS;
	uint8_t value;

	value  = *((uint8_t *)pib_attribute_value);

	switch (pib_attribute) {
	/* PHY */
	case PHY_TRANSMIT_POWER:
		if (value > 0x3F)
			status = MAC_INVALID_PARAMETER;
		break;
	case PHY_CCA_MODE:
		if (value > 0x03)
			status = MAC_INVALID_PARAMETER;
		break;
	/* MAC */
	case MAC_BATT_LIFE_EXT_PERIODS:
		if ((value < 6) || (value > 41))
			status = MAC_INVALID_PARAMETER;
		break;
	case MAC_BEACON_PAYLOAD:
		if (pib_attribute_length > MAX_BEACON_PAYLOAD_LENGTH)
			status = MAC_INVALID_PARAMETER;
		break;
	case MAC_BEACON_PAYLOAD_LENGTH:
		if (value > MAX_BEACON_PAYLOAD_LENGTH)
			status = MAC_INVALID_PARAMETER;
		break;
	case MAC_BEACON_ORDER:
		if (value > 15)
			status = MAC_INVALID_PARAMETER;
		break;
	case MAC_MAX_BE:
		if ((value < 3) || (value > 8))
			status = MAC_INVALID_PARAMETER;
		break;
	case MAC_MAX_CSMA_BACKOFFS:
		if (value > 5)
			status = MAC_INVALID_PARAMETER;
		break;
	case MAC_MAX_FRAME_RETRIES:
		if (value > 7)
			status = MAC_INVALID_PARAMETER;
		break;
	case MAC_MIN_BE:
		if (value > 8)
			status = MAC_INVALID_PARAMETER;
		break;
	case MAC_RESPONSE_WAIT_TIME:
		if ((value < 2) || (value > 64))
			status = MAC_INVALID_PARAMETER;
		break;
	case MAC_SUPERFRAME_ORDER:
		if (value > 15)
			status = MAC_INVALID_PARAMETER;
		break;
	/* boolean */
	case MAC_ASSOCIATED_PAN_COORD:
	case MAC_ASSOCIATION_PERMIT:
	case MAC_AUTO_REQUEST:
	case MAC_BATT_LIFE_EXT:
	case MAC_GTS_PERMIT:
	case MAC_PROMISCUOUS_MODE:
	case MAC_RX_ON_WHEN_IDLE:
	case MAC_SECURITY_ENABLED:
		if (value > 1)
			status = MAC_INVALID_PARAMETER;
		break;
	/* MAC SEC */
	case MAC_AUTO_REQUEST_SECURITY_LEVEL:
		if (value > 7)
			status = MAC_INVALID_PARAMETER;
		break;
	case MAC_AUTO_REQUEST_KEY_ID_MODE:
		if (value > 3)
			status = MAC_INVALID_PARAMETER;
		break;
	default:
		break;
	}

	return status;
}

/**
 * tdme_settxpower() - Sets the tx power for MLME_SET phyTransmitPower
 * @txp:        Transmit Power
 * @device_ref: Nondescript pointer to target device
 *
 * Normalised to 802.15.4 Definition (6-bit, signed):
 * Bit 7-6: not used
 * Bit 5-0: tx power (-32 - +31 dB)
 *
 * Return: 802.15.4 status code of api calls
 */
static uint8_t tdme_settxpower(uint8_t txp, void *device_ref)
{
	uint8_t status;
	int8_t txp_val;
	uint8_t txp_ext;
	uint8_t paib;

	/* extend from 6 to 8 bit */
	txp_ext = 0x3F & txp;
	if (txp_ext & 0x20)
		txp_ext += 0xC0;
	txp_val = (int8_t)txp_ext;

	if (CA8210_MAC_MPW) {
		if (txp_val > 0) {
			/* 8 dBm: ptrim = 5, itrim = +3 => +4 dBm */
			paib = 0xD3;
		} else {
			/* 0 dBm: ptrim = 7, itrim = +3 => -6 dBm */
			paib = 0x73;
		}
		/* write PACFG */
		status = tdme_setsfr_request_sync(
			0,
			CA8210_SFR_PACFG,
			paib,
			device_ref
		);
	} else {
		/* Look-Up Table for Setting Current and Frequency Trim values
		 * for desired Output Power */
		if (txp_val > 8) {
			paib = 0x3F;
		} else if (txp_val == 8) {
			paib = 0x32;
		} else if (txp_val == 7) {
			paib = 0x22;
		} else if (txp_val == 6) {
			paib = 0x18;
		} else if (txp_val == 5) {
			paib = 0x10;
		} else if (txp_val == 4) {
			paib = 0x0C;
		} else if (txp_val == 3) {
			paib = 0x08;
		} else if (txp_val == 2) {
			paib = 0x05;
		} else if (txp_val == 1) {
			paib = 0x03;
		} else if (txp_val == 0) {
			paib = 0x01;
		} else         /*  <  0 */ {
			paib = 0x00;
		}
		/* write PACFGIB */
		status = tdme_setsfr_request_sync(
			0,
			CA8210_SFR_PACFGIB,
			paib,
			device_ref
		);
	}

	return status;
}

/**
 * mcps_data_request() - mcps_data_request (Send Data) according to API Spec
 * @src_addr_mode:    Source Addressing Mode
 * @dst_address_mode: Destination Addressing Mode
 * @dst_pan_id:       Destination PAN ID
 * @dst_addr:         Pointer to Destination Address
 * @msdu_length:      length of Data
 * @msdu:             Pointer to Data
 * @msdu_handle:      Handle of Data
 * @tx_options:       Tx Options Bit Field
 * @security:         Pointer to Security Structure or NULL
 * @device_ref:       Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of action
 */
static uint8_t mcps_data_request(
	uint8_t          src_addr_mode,
	uint8_t          dst_address_mode,
	uint16_t         dst_pan_id,
	union macaddr   *dst_addr,
	uint8_t          msdu_length,
	uint8_t         *msdu,
	uint8_t          msdu_handle,
	uint8_t          tx_options,
	struct secspec  *security,
	void            *device_ref
)
{
	struct secspec *pSec;
	struct mac_message command;

	#define DATAREQ (command.pdata.data_req)
	command.command_id = SPI_MCPS_DATA_REQUEST;
	DATAREQ.src_addr_mode = src_addr_mode;
	DATAREQ.dst.mode = dst_address_mode;
	if (dst_address_mode != MAC_MODE_NO_ADDR) {
		DATAREQ.dst.pan_id[0] = LS_BYTE(dst_pan_id);
		DATAREQ.dst.pan_id[1] = MS_BYTE(dst_pan_id);
		if (dst_address_mode == MAC_MODE_SHORT_ADDR) {
			DATAREQ.dst.address[0] = LS_BYTE(
				dst_addr->short_address
			);
			DATAREQ.dst.address[1] = MS_BYTE(
				dst_addr->short_address
			);
		} else {   /* MAC_MODE_LONG_ADDR*/
			memcpy(DATAREQ.dst.address, dst_addr->ieee_address, 8);
		}
	}
	DATAREQ.msdu_length = msdu_length;
	DATAREQ.msdu_handle = msdu_handle;
	DATAREQ.tx_options = tx_options;
	memcpy(DATAREQ.msdu, msdu, msdu_length);
	pSec = (struct secspec *)(DATAREQ.msdu + msdu_length);
	command.length = sizeof(struct mcps_data_request_pset) -
		MAX_DATA_SIZE + msdu_length;
	if ((security == NULL) || (security->security_level == 0)) {
		pSec->security_level = 0;
		command.length += 1;
	} else {
		*pSec = *security;
		command.length += sizeof(struct secspec);
	}

	if (cascoda_api_downstream(
		&command.command_id, command.length + 2, NULL, device_ref)
	)
		return MAC_SYSTEM_ERROR;

	return MAC_SUCCESS;
	#undef DATAREQ
}

/**
 * mlme_reset_request_sync() - MLME_RESET_request/confirm according to API Spec
 * @set_default_pib: Set defaults in PIB
 * @device_ref:      Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of MLME-RESET.confirm
 */
static uint8_t mlme_reset_request_sync(
	uint8_t  set_default_pib,
	void    *device_ref
)
{
	uint8_t status;
	struct mac_message command, response;
	struct spi_device *spi = device_ref;

	#define SIMPLEREQ (command.pdata)
	#define SIMPLECNF (response.pdata)
	command.command_id = SPI_MLME_RESET_REQUEST;
	command.length = 1;
	SIMPLEREQ.u8param = set_default_pib;

	if (cascoda_api_downstream(
		&command.command_id,
		command.length + 2,
		&response.command_id,
		device_ref)) {
		dev_err(&spi->dev, "cascoda_api_downstream failed\n");
		return MAC_SYSTEM_ERROR;
	}

	if (response.command_id != SPI_MLME_RESET_CONFIRM)
		return MAC_SYSTEM_ERROR;

	status = SIMPLECNF.status;

	/* reset COORD Bit for Channel Filtering as Coordinator */
	if (CA8210_MAC_WORKAROUNDS && set_default_pib && (!status)) {
		status = tdme_setsfr_request_sync(
			0,
			CA8210_SFR_MACCON,
			0,
			device_ref
		);
	}

	return status;
	#undef SIMPLEREQ
	#undef SIMPLECNF
}

/**
 * mlme_set_request_sync() - MLME_SET_request/confirm according to API Spec
 * @pib_attribute:        Attribute Number
 * @pib_attribute_index:  Index within Attribute if an Array
 * @pib_attribute_length: Attribute length
 * @pib_attribute_value:  Pointer to Attribute Value
 * @device_ref:           Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of MLME-SET.confirm
 */
static uint8_t mlme_set_request_sync(
	uint8_t       pib_attribute,
	uint8_t       pib_attribute_index,
	uint8_t       pib_attribute_length,
	const void   *pib_attribute_value,
	void         *device_ref
)
{
	uint8_t status;
	struct mac_message command, response;

	#define SETREQ    (command.pdata.set_req)
	#define SIMPLECNF (response.pdata)
	/* pre-check the validity of pib_attribute values that are not checked
	 * in MAC
	 */
	if (tdme_checkpibattribute(
		pib_attribute, pib_attribute_length, pib_attribute_value)) {
		return MAC_INVALID_PARAMETER;
	}

	if (pib_attribute == PHY_CURRENT_CHANNEL) {
		status = tdme_channelinit(
			*((uint8_t *)pib_attribute_value),
			device_ref
		);
		if (status) {
			return status;
		}
	}

	if (pib_attribute == PHY_TRANSMIT_POWER) {
		return tdme_settxpower(
			*((uint8_t *)pib_attribute_value),
			device_ref
		);
	}

	command.command_id = SPI_MLME_SET_REQUEST;
	command.length = sizeof(struct mlme_set_request_pset) -
		MAX_ATTRIBUTE_SIZE + pib_attribute_length;
	SETREQ.pib_attribute = pib_attribute;
	SETREQ.pib_attribute_index = pib_attribute_index;
	SETREQ.pib_attribute_length = pib_attribute_length;
	memcpy(
		SETREQ.pib_attribute_value,
		pib_attribute_value,
		pib_attribute_length
	);

	if (cascoda_api_downstream(
		&command.command_id,
		command.length + 2,
		&response.command_id,
		device_ref)) {
		return MAC_SYSTEM_ERROR;
	}

	if (response.command_id != SPI_MLME_SET_CONFIRM)
		return MAC_SYSTEM_ERROR;

	return SIMPLECNF.status;
	#undef SETREQ
	#undef SIMPLECNF
}

/**
 * hwme_set_request_sync() - HWME_SET_request/confirm according to API Spec
 * @hw_attribute:        Attribute Number
 * @hw_attribute_length: Attribute length
 * @hw_attribute_value:  Pointer to Attribute Value
 * @device_ref:          Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of HWME-SET.confirm
 */
static uint8_t hwme_set_request_sync(
	uint8_t      hw_attribute,
	uint8_t      hw_attribute_length,
	uint8_t     *hw_attribute_value,
	void        *device_ref
)
{
	struct mac_message command, response;

	command.command_id = SPI_HWME_SET_REQUEST;
	command.length = 2 + hw_attribute_length;
	command.pdata.hwme_set_req.hw_attribute = hw_attribute;
	command.pdata.hwme_set_req.hw_attribute_length = hw_attribute_length;
	memcpy(
		command.pdata.hwme_set_req.hw_attribute_value,
		hw_attribute_value,
		hw_attribute_length
	);

	if (cascoda_api_downstream(
		&command.command_id,
		command.length + 2,
		&response.command_id,
		device_ref)) {
		return MAC_SYSTEM_ERROR;
	}

	if (response.command_id != SPI_HWME_SET_CONFIRM)
		return MAC_SYSTEM_ERROR;

	return response.pdata.hwme_set_cnf.status;
}

/**
 * hwme_get_request_sync() - HWME_GET_request/confirm according to API Spec
 * @hw_attribute:        Attribute Number
 * @hw_attribute_length: Attribute length
 * @hw_attribute_value:  Pointer to Attribute Value
 * @device_ref:          Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of HWME-GET.confirm
 */
static uint8_t hwme_get_request_sync(
	uint8_t      hw_attribute,
	uint8_t     *hw_attribute_length,
	uint8_t     *hw_attribute_value,
	void        *device_ref
)
{
	struct mac_message command, response;

	command.command_id = SPI_HWME_GET_REQUEST;
	command.length = 1;
	command.pdata.hwme_get_req.hw_attribute = hw_attribute;

	if (cascoda_api_downstream(
		&command.command_id,
		command.length + 2,
		&response.command_id,
		device_ref)) {
		return MAC_SYSTEM_ERROR;
	}

	if (response.command_id != SPI_HWME_GET_CONFIRM)
		return MAC_SYSTEM_ERROR;

	if (response.pdata.hwme_get_cnf.status == MAC_SUCCESS) {
		*hw_attribute_length = \
			response.pdata.hwme_get_cnf.hw_attribute_length;
		memcpy(
			hw_attribute_value,
			response.pdata.hwme_get_cnf.hw_attribute_value,
			*hw_attribute_length
		);
	}

	return response.pdata.hwme_get_cnf.status;
}

/* Network driver operation */

/**
 * ca8210_async_xmit_complete() - Called to announce that an asynchronous
 *                                transmission has finished
 * @hw:          ieee802154_hw of ca8210 that has finished exchange
 * @msduhandle:  Identifier of transmission that has completed
 * @status:      Returned 802.15.4 status code of the transmission
 *
 * Return: 0 or linux error code
 */
static int ca8210_async_xmit_complete(
	struct ieee802154_hw  *hw,
	uint8_t                msduhandle,
	uint8_t                status)
{
	struct ca8210_priv *priv = hw->priv;
	unsigned long flags;

	if (priv->nextmsduhandle != msduhandle) {
		dev_err(
			&priv->spi->dev,
			"Unexpected msdu_handle on data confirm, " \
			"Expected %d, got %d\n",
			priv->nextmsduhandle,
			msduhandle
		);
		return -EIO;
	}

	/* stop timeout work */
	if (!cancel_delayed_work_sync(&priv->async_tx_timeout_work)) {
		dev_err(
			&priv->spi->dev,
			"async tx timeout wasn't pending when transfer " \
			"complete\n"
		);
	}

	spin_lock_irqsave(&priv->lock, flags);

	priv->async_tx_pending = false;
	priv->nextmsduhandle++;

	spin_unlock_irqrestore(&priv->lock, flags);

	if (status) {
		dev_err(
			&priv->spi->dev,
			"Link transmission unsuccessful, status = %d\n",
			status
		);
		if (status != MAC_TRANSACTION_OVERFLOW) {
			ieee802154_wake_queue(priv->hw);
			return 0;
		}
	}
	ieee802154_xmit_complete(priv->hw, priv->tx_skb, true);

	return 0;
}

/**
 * ca8210_skb_rx() - Contructs a properly framed socket buffer from a received
 *                   MCPS_DATA_indication
 * @hw:        ieee802154_hw that MCPS_DATA_indication was received by
 * @len:       length of MCPS_DATA_indication
 * @data_ind:  Octet array of MCPS_DATA_indication
 *
 * Called by the spi driver whenever a SAP command is received, this function
 * will ascertain whether the command is of interest to the network driver and
 * take necessary action.
 *
 * Return: 0 or linux error code
 */
static int ca8210_skb_rx(
	struct ieee802154_hw  *hw,
	size_t                 len,
	uint8_t               *data_ind
)
{
	struct ieee802154_hdr hdr;
	int msdulen;
	int hlen;
	struct sk_buff *skb;
	struct ca8210_priv *priv = hw->priv;

	/* Allocate mtu size buffer for every rx packet */
	skb = dev_alloc_skb(IEEE802154_MTU + sizeof(hdr));
	if (skb == NULL) {
		dev_crit(&priv->spi->dev, "dev_alloc_skb failed\n");
		return -ENOMEM;
	}
	skb_reserve(skb, sizeof(hdr));

	msdulen = data_ind[22]; /* msdu_length */
	if (msdulen > 127) {
		dev_err(
			&priv->spi->dev,
			"received erroneously large msdu length!\n"
		);
		kfree_skb(skb);
		return -EMSGSIZE;
	}
	dev_dbg(&priv->spi->dev, "skb buffer length = %d\n", msdulen);

	/* Populate hdr */
	hdr.sec.level = data_ind[29 + msdulen];
	dev_dbg(&priv->spi->dev, "security level: %#03x\n", hdr.sec.level);
	if (hdr.sec.level > 0) {
		hdr.sec.key_id_mode = data_ind[30 + msdulen];
		memcpy(&hdr.sec.extended_src, &data_ind[31 + msdulen], 8);
		hdr.sec.key_id = data_ind[39 + msdulen];
	}
	hdr.source.mode = data_ind[0];
	dev_dbg(&priv->spi->dev, "srcAddrMode: %#03x\n", hdr.source.mode);
	hdr.source.pan_id = *(uint16_t *)&data_ind[1];
	dev_dbg(&priv->spi->dev, "srcPanId: %#06x\n", hdr.source.pan_id);
	memcpy(&hdr.source.extended_addr, &data_ind[3], 8);
	hdr.dest.mode = data_ind[11];
	dev_dbg(&priv->spi->dev, "dstAddrMode: %#03x\n", hdr.dest.mode);
	hdr.dest.pan_id = *(uint16_t *)&data_ind[12];
	dev_dbg(&priv->spi->dev, "dstPanId: %#06x\n", hdr.dest.pan_id);
	memcpy(&hdr.dest.extended_addr, &data_ind[14], 8);

	/* Fill in FC implicitly */
	hdr.fc.type = 1; /* Data frame */
	if (hdr.sec.level)
		hdr.fc.security_enabled = 1;
	else
		hdr.fc.security_enabled = 0;
	if (data_ind[1] != data_ind[12] || data_ind[2] != data_ind[13])
		hdr.fc.intra_pan = 1;
	else
		hdr.fc.intra_pan = 0;
	hdr.fc.dest_addr_mode = hdr.dest.mode;
	hdr.fc.source_addr_mode = hdr.source.mode;

	/* Add hdr to front of buffer */
	hlen = ieee802154_hdr_push(skb, &hdr);

	if (hlen < 0) {
		dev_crit(&priv->spi->dev, "failed to push mac hdr onto skb!\n");
		kfree_skb(skb);
		return hlen;
	}

	skb_reset_mac_header(skb);
	skb->mac_len = hlen;

	/* Add <msdulen> bytes of space to the back of the buffer */
	/* Copy msdu to skb */
	memcpy(skb_put(skb, msdulen), &data_ind[29], msdulen);

	ieee802154_rx_irqsafe(hw, skb, data_ind[23]/*LQI*/);
	/* TODO: Set protocol & checksum? */
	/* TODO: update statistics */
	return 0;
}

/**
 * ca8210_net_rx() - Acts upon received SAP commands relevant to the network
 *                   driver
 * @hw:       ieee802154_hw that command was received by
 * @command:  Octet array of received command
 * @len:      length of the received command
 *
 * Called by the spi driver whenever a SAP command is received, this function
 * will ascertain whether the command is of interest to the network driver and
 * take necessary action.
 *
 * Return: 0 or linux error code
 */
static int ca8210_net_rx(struct ieee802154_hw *hw, uint8_t *command, size_t len)
{
	struct ca8210_priv *priv = hw->priv;
	unsigned long flags;
	uint8_t status;

	dev_dbg(&priv->spi->dev, "ca8210_net_rx(), CmdID = %d\n", command[0]);


	if (command[0] == SPI_MCPS_DATA_INDICATION) {
		/* Received data */
		spin_lock_irqsave(&priv->lock, flags);
		if (command[26] == priv->last_dsn) {
			dev_dbg(
				&priv->spi->dev,
				"DSN %d resend received, ignoring...\n",
				command[26]
			);
			spin_unlock_irqrestore(&priv->lock, flags);
			return 0;
		}
		priv->last_dsn = command[26];
		spin_unlock_irqrestore(&priv->lock, flags);
		return ca8210_skb_rx(hw, len-2, command+2);
	} else if (command[0] == SPI_MCPS_DATA_CONFIRM) {
		status = command[3];
		if (priv->async_tx_pending) {
			return ca8210_async_xmit_complete(
				hw,
				command[2],
				status
			);
		} else if (priv->sync_tx_pending) {
			priv->sync_tx_pending = false;
		}
	}

	return 0;
}

/**
 * ca8210_skb_tx() - Transmits a given socket buffer using the ca8210
 * @skb:         Socket buffer to transmit
 * @msduhandle:  Data identifier to pass to the 802.15.4 MAC
 * @priv:        Pointer to private data section of target ca8210
 *
 * Return: 0 or linux error code
 */
static int ca8210_skb_tx(
	struct sk_buff      *skb,
	uint8_t              msduhandle,
	struct ca8210_priv  *priv
)
{
	int status;
	struct ieee802154_hdr header = { 0 };
	struct secspec secspec;
	unsigned int mac_len;

	dev_dbg(&priv->spi->dev, "ca8210_skb_tx() called\n");

	/* Get addressing info from skb - ieee802154 layer creates a full
	 * packet
	 */
	mac_len = ieee802154_hdr_peek_addrs(skb, &header);

	secspec.security_level = header.sec.level;
	secspec.key_id_mode = header.sec.key_id_mode;
	if (secspec.key_id_mode == 2)
		memcpy(secspec.key_source, &header.sec.short_src, 4);
	else if (secspec.key_id_mode == 3)
		memcpy(secspec.key_source, &header.sec.extended_src, 8);
	secspec.key_index = header.sec.key_id;

	/* Pass to Cascoda API */
	status =  mcps_data_request(header.source.mode,
	                            header.dest.mode,
	                            header.dest.pan_id,
	                            (union macaddr *)&header.dest.extended_addr,
	                            skb->len - mac_len,
	                            &skb->data[mac_len],
	                            msduhandle,
	                            header.fc.ack_request,
	                            &secspec,
	                            priv->spi);
	return link_to_linux_err(status);
}

/**
 * ca8210_async_tx_worker() - Dispatched to handle asynchronous data
 *                            transmission
 * @work:  Work being executed
 */
static void ca8210_async_tx_worker(struct work_struct *work)
{
	struct ca8210_priv *priv = container_of(
		work,
		struct ca8210_priv,
		async_tx_work
	);
	unsigned long flags;
	int ret;

	if (priv->tx_skb == NULL) {
		return;
	}

	ret = ca8210_skb_tx(priv->tx_skb, priv->nextmsduhandle, priv);
	if (ret < 0) {
		dev_warn(
			&priv->spi->dev,
			"Failed to transmit skb, returned %d\n",
			ret
		);
		/* retry transmission higher up */
		if (priv->spi_errno == -EAGAIN) {
			dev_crit(
				&priv->spi->dev,
				"CA8210 CONSTANTLY NACKING!\n"
			);
			return;
		}
		ieee802154_wake_queue(priv->hw);
		return;
	}

	queue_delayed_work(priv->async_tx_workqueue,
	                   &priv->async_tx_timeout_work,
	                   msecs_to_jiffies(CA8210_DATA_CNF_TIMEOUT));

	spin_lock_irqsave(&priv->lock, flags);

	priv->async_tx_pending = true;

	spin_unlock_irqrestore(&priv->lock, flags);
}

/**
 * ca8210_async_tx_timeout_worker() - Executed when an asynchronous transmission
 *                                    times out
 * @work:  Work being executed
 */
static void ca8210_async_tx_timeout_worker(struct work_struct *work)
{
	struct delayed_work *del_work = container_of(
		work,
		struct delayed_work,
		work
	);
	struct ca8210_priv *priv = container_of(
		del_work,
		struct ca8210_priv,
		async_tx_timeout_work
	);
	unsigned long flags;

	dev_err(&priv->spi->dev, "data confirm timed out\n");

	spin_lock_irqsave(&priv->lock, flags);

	priv->async_tx_pending = false;

	spin_unlock_irqrestore(&priv->lock, flags);

	ieee802154_wake_queue(priv->hw);
}

/**
 * ca8210_start() - Starts the network driver
 * @hw:  ieee802154_hw of ca8210 being started
 *
 * Return: 0 or linux error code
 */
static int ca8210_start(struct ieee802154_hw *hw)
{
	int status;
	uint8_t rx_on_when_idle;
	struct ca8210_priv *priv = hw->priv;

	priv->async_tx_workqueue = alloc_ordered_workqueue(
		"ca8210 tx worker",
		0
	);
	if (priv->async_tx_workqueue == NULL) {
		dev_crit(&priv->spi->dev, "alloc_ordered_workqueue failed\n");
		return -ENOMEM;
	}
	INIT_WORK(&priv->async_tx_work, ca8210_async_tx_worker);
	INIT_DELAYED_WORK(
		&priv->async_tx_timeout_work,
		ca8210_async_tx_timeout_worker
	);

	priv->last_dsn = -1;
	/* Turn receiver on when idle for now just to test rx */
	rx_on_when_idle = 1;
	status = mlme_set_request_sync(
		MAC_RX_ON_WHEN_IDLE,
		0,
		1,
		&rx_on_when_idle,
		priv->spi
	);
	if (status) {
		dev_crit(
			&priv->spi->dev,
			"Setting rx_on_when_idle failed, status = %d\n",
			status
		);
		return link_to_linux_err(status);
	}

	return 0;
}

/**
 * ca8210_stop() - Stops the network driver
 * @hw:  ieee802154_hw of ca8210 being stopped
 *
 * Return: 0 or linux error code
 */
static void ca8210_stop(struct ieee802154_hw *hw)
{
	struct ca8210_priv *priv = hw->priv;

	flush_workqueue(priv->async_tx_workqueue);
	destroy_workqueue(priv->async_tx_workqueue);
}

/**
 * ca8210_xmit_sync() - Synchronously transmits a given socket buffer using the
 *                      ca8210
 * @hw:   ieee802154_hw of ca8210 to transmit from
 * @skb:  Socket buffer to transmit
 *
 * Transmits the buffer and does not return until a confirmation of the exchange
 * is received from the ca8210.
 *
 * Return: 0 or linux error code
 */
static int ca8210_xmit_sync(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct ca8210_priv *priv = hw->priv;
	int status;

	dev_dbg(&priv->spi->dev, "calling ca8210_xmit_sync()\n");

	status = ca8210_skb_tx(skb, priv->nextmsduhandle++, priv);
	if (status)
		return status;

	priv->sync_tx_pending = true;

	while (priv->sync_tx_pending) {
		msleep(1);
	}

	return 0;
}

/**
 * ca8210_xmit_async() - Asynchronously transmits a given socket buffer using
 *                       the ca8210
 * @hw:   ieee802154_hw of ca8210 to transmit from
 * @skb:  Socket buffer to transmit
 *
 * Hands the transmission of the buffer off to a workqueue and returns
 * immediately.
 *
 * Return: 0 or linux error code
 */
static int ca8210_xmit_async(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct ca8210_priv *priv = hw->priv;

	dev_dbg(&priv->spi->dev, "calling ca8210_xmit_async()\n");

	priv->tx_skb = skb;
	queue_work(priv->async_tx_workqueue, &priv->async_tx_work);

	return 0;
}

/**
 * ca8210_get_ed() - Returns the measured energy on the current channel at this
 *                   instant in time
 * @hw:     ieee802154_hw of target ca8210
 * @level:  Measured Energy Detect level
 *
 * Return: 0 or linux error code
 */
static int ca8210_get_ed(struct ieee802154_hw *hw, uint8_t *level)
{
	uint8_t lenvar;
	struct ca8210_priv *priv = hw->priv;

	return link_to_linux_err(
		hwme_get_request_sync(HWME_EDVALUE, &lenvar, level, priv->spi)
	);
}

/**
 * ca8210_set_channel() - Sets the current operating 802.15.4 channel of the
 *                        ca8210
 * @hw:       ieee802154_hw of target ca8210
 * @page:     Channel page to set
 * @channel:  Channel number to set
 *
 * Return: 0 or linux error code
 */
static int ca8210_set_channel(
	struct ieee802154_hw  *hw,
	uint8_t                page,
	uint8_t                channel
)
{
	uint8_t status;
	struct ca8210_priv *priv = hw->priv;

	status = mlme_set_request_sync(
		PHY_CURRENT_CHANNEL,
		0,
		1,
		&channel,
		priv->spi
	);
	if (status) {
		dev_err(
			&priv->spi->dev,
			"error setting channel, MLME-SET.confirm status = %d\n",
			status
		);
	}
	return link_to_linux_err(status);
}

/**
 * ca8210_set_hw_addr_filt() - Sets the address filtering parameters of the
 *                             ca8210
 * @hw:       ieee802154_hw of target ca8210
 * @filt:     Filtering parameters
 * @changed:  Bitmap representing which parameters to change
 *
 * Effectively just sets the actual addressing information identifying this node
 * as all filtering is performed by the ca8210 as detailed in the IEEE 802.15.4
 * 2006 specification.
 *
 * Return: 0 or linux error code
 */
static int ca8210_set_hw_addr_filt(
	struct ieee802154_hw            *hw,
	struct ieee802154_hw_addr_filt  *filt,
	unsigned long                    changed
)
{
	uint8_t status = 0;
	struct ca8210_priv *priv = hw->priv;

	if (changed&IEEE802154_AFILT_PANID_CHANGED) {
		status = mlme_set_request_sync(
			MAC_PAN_ID,
			0,
			2,
			&filt->pan_id, priv->spi
		);
		if (status) {
			dev_err(
				&priv->spi->dev,
				"error setting pan id, " \
				"MLME-SET.confirm status = %d",
				status
			);
			return link_to_linux_err(status);
		}
	}
	if (changed&IEEE802154_AFILT_SADDR_CHANGED) {
		status = mlme_set_request_sync(
			MAC_SHORT_ADDRESS,
			0,
			2,
			&filt->short_addr, priv->spi
		);
		if (status) {
			dev_err(
				&priv->spi->dev,
				"error setting short address, " \
				"MLME-SET.confirm status = %d",
				status
			);
			return link_to_linux_err(status);
		}
	}
	if (changed&IEEE802154_AFILT_IEEEADDR_CHANGED) {
		status = mlme_set_request_sync(
			NS_IEEE_ADDRESS,
			0,
			8,
			&filt->ieee_addr,
			priv->spi
		);
		if (status) {
			dev_err(
				&priv->spi->dev,
				"error setting ieee address, " \
				"MLME-SET.confirm status = %d",
				status
			);
			return link_to_linux_err(status);
		}
	}
	/* TODO: Should use MLME_START to set coord bit? */
	return 0;
}

/**
 * ca8210_set_tx_power() - Sets the transmit power of the ca8210
 * @hw:   ieee802154_hw of target ca8210
 * @dbm:  Transmit power in dBm
 *
 * Return: 0 or linux error code
 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0)
static int ca8210_set_tx_power(struct ieee802154_hw *hw, s32 dbm)
#else
static int ca8210_set_tx_power(struct ieee802154_hw *hw, s8 dbm)
#endif
{
	struct ca8210_priv *priv = hw->priv;

	return link_to_linux_err(
		mlme_set_request_sync(PHY_TRANSMIT_POWER, 0, 1, &dbm, priv->spi)
	);
}

/**
 * ca8210_set_cca_mode() - Sets the clear channel assessment mode of the ca8210
 * @hw:   ieee802154_hw of target ca8210
 * @cca:  CCA mode to set
 *
 * Return: 0 or linux error code
 */
static int ca8210_set_cca_mode(
	struct ieee802154_hw       *hw,
	const struct wpan_phy_cca  *cca
)
{
	uint8_t status;
	uint8_t cca_mode;
	struct ca8210_priv *priv = hw->priv;

	cca_mode = cca->mode & 3;
	if (cca_mode == 3 && cca->opt == NL802154_CCA_OPT_ENERGY_CARRIER_OR) {
		/* cca_mode 0 == CS OR ED, 3 == CS AND ED */
		cca_mode = 0;
	}
	status = mlme_set_request_sync(
		PHY_CCA_MODE,
		0,
		1,
		&cca_mode,
		priv->spi
	);
	if (status) {
		dev_err(
			&priv->spi->dev,
			"error setting cca mode, " \
			"MLME-SET.confirm status = %d",
			status
		);
	}
	return link_to_linux_err(status);
}

/**
 * ca8210_set_cca_ed_level() - Sets the CCA ED level of the ca8210
 * @hw:     ieee802154_hw of target ca8210
 * @level:  ED level to set
 *
 * Sets the minumum threshold of measured energy above which the ca8210 will
 * back off and retry a transmission.
 *
 * Return: 0 or linux error code
 */
static int ca8210_set_cca_ed_level(struct ieee802154_hw *hw, int32_t level)
{
	uint8_t status;
	uint8_t ed_threshold = level * 2 + 256;
	struct ca8210_priv *priv = hw->priv;

	status = hwme_set_request_sync(
		HWME_EDTHRESHOLD,
		1,
		&ed_threshold,
		priv->spi
	);
	if (status) {
		dev_err(
			&priv->spi->dev,
			"error setting ed threshold, " \
			"HWME-SET.confirm status = %d",
			status
		);
	}
	return link_to_linux_err(status);
}

/**
 * ca8210_set_csma_params() - Sets the CSMA parameters of the ca8210
 * @hw:       ieee802154_hw of target ca8210
 * @min_be:   Minimum backoff exponent when backing off a transmission
 * @max_be:   Maximum backoff exponent when backing off a transmission
 * @retries:  Number of times to retry after backing off
 *
 * Return: 0 or linux error code
 */
static int ca8210_set_csma_params(
	struct ieee802154_hw  *hw,
	uint8_t                min_be,
	uint8_t                max_be,
	uint8_t                retries
)
{
	uint8_t status;
	struct ca8210_priv *priv = hw->priv;

	status = mlme_set_request_sync(MAC_MIN_BE, 0, 1, &min_be, priv->spi);
	if (status) {
		dev_err(
			&priv->spi->dev,
			"error setting min be, MLME-SET.confirm status = %d",
			status
		);
		return link_to_linux_err(status);
	}
	status = mlme_set_request_sync(MAC_MAX_BE, 0, 1, &max_be, priv->spi);
	if (status) {
		dev_err(
			&priv->spi->dev,
			"error setting max be, MLME-SET.confirm status = %d",
			status
		);
		return link_to_linux_err(status);
	}
	status = mlme_set_request_sync(
		MAC_MAX_CSMA_BACKOFFS,
		0,
		1,
		&retries,
		priv->spi
	);
	if (status) {
		dev_err(
			&priv->spi->dev,
			"error setting max csma backoffs, " \
			"MLME-SET.confirm status = %d",
			status
		);
	}
	return link_to_linux_err(status);
}

/**
 * ca8210_set_frame_retries() - Sets the maximum frame retries of the ca8210
 * @hw:       ieee802154_hw of target ca8210
 * @retries:  Number of retries
 *
 * Sets the number of times to retry a transmission if no acknowledgement was
 * was received from the other end when one was requested.
 *
 * Return: 0 or linux error code
 */
static int ca8210_set_frame_retries(struct ieee802154_hw *hw, s8 retries)
{
	uint8_t status;
	struct ca8210_priv *priv = hw->priv;

	status = mlme_set_request_sync(
		MAC_MAX_FRAME_RETRIES,
		0,
		1,
		&retries,
		priv->spi
	);
	if (status) {
		dev_err(
			&priv->spi->dev,
			"error setting panid, MLME-SET.confirm status = %d",
			status
		);
	}
	return link_to_linux_err(status);
}

static const struct ieee802154_ops ca8210_phy_ops = {
	.start = ca8210_start,
	.stop = ca8210_stop,
	.xmit_sync = ca8210_xmit_sync,
	.xmit_async = ca8210_xmit_async,
	.ed = ca8210_get_ed,
	.set_channel = ca8210_set_channel,
	.set_hw_addr_filt = ca8210_set_hw_addr_filt,
	.set_txpower = ca8210_set_tx_power,
	.set_cca_mode = ca8210_set_cca_mode,
	.set_cca_ed_level = ca8210_set_cca_ed_level,
	.set_csma_params = ca8210_set_csma_params,
	.set_frame_retries = ca8210_set_frame_retries
};

/* Test/EVBME Interface */

/**
 * ca8210_test_int_open() - Opens the test interface to the userspace
 * @inodp:  inode representation of file interface
 * @filp:   file interface
 *
 * Return: 0 or linux error code
 */
static int ca8210_test_int_open(struct inode *inodp, struct file *filp)
{
	struct ca8210_priv *priv = inodp->i_private;

	filp->private_data = priv;
	return 0;
}

/**
 * ca8210_test_check_upstream() - Checks a command received from the upstream
 *                                testing interface for required action
 * @buf:        Buffer containing command to check
 * @device_ref: Nondescript pointer to target device
 *
 * Return: 0 or linux error code
 */
static int ca8210_test_check_upstream(uint8_t *buf, void *device_ref)
{
	int ret;
	uint8_t response[CA8210_SPI_BUF_SIZE];

	if (buf[0] == SPI_MLME_SET_REQUEST) {
		ret = tdme_checkpibattribute(buf[2], buf[4], buf + 5);
		if (ret) {
			response[0]  = SPI_MLME_SET_CONFIRM;
			response[1] = 3;
			response[2] = MAC_INVALID_PARAMETER;
			response[3] = buf[2];
			response[4] = buf[3];
			if (cascoda_api_upstream != NULL)
				cascoda_api_upstream(response, 5, device_ref);
			return ret;
		}
	}
	if (buf[0] == SPI_MLME_ASSOCIATE_REQUEST) {
		return tdme_channelinit(buf[2], device_ref);
	} else if (buf[0] == SPI_MLME_START_REQUEST) {
		return tdme_channelinit(buf[4], device_ref);
	} else if ((buf[0] == SPI_MLME_SET_REQUEST) &&
	           (buf[2] == PHY_CURRENT_CHANNEL)) {
		return tdme_channelinit(buf[5], device_ref);
	} else if ((buf[0] == SPI_TDME_SET_REQUEST) &&
	           (buf[2] == TDME_CHANNEL)) {
		return tdme_channelinit(buf[4], device_ref);
	} else if ((CA8210_MAC_WORKAROUNDS) &&
	           (buf[0] == SPI_MLME_RESET_REQUEST) && (buf[2] == 1)) {
		/* reset COORD Bit for Channel Filtering as Coordinator */
		return tdme_setsfr_request_sync(
			0,
			CA8210_SFR_MACCON,
			0,
			device_ref
		);
	}
	return 0;
} /* End of EVBMECheckSerialCommand() */

/**
 * ca8210_test_int_user_write() - Called by a process in userspace to send a
 *                                message to the ca8210 drivers
 * @filp:    file interface
 * @in_buf:  Buffer containing message to write
 * @len:     length of message
 * @off:     file offset
 *
 * Return: 0 or linux error code
 */
static ssize_t ca8210_test_int_user_write(
	struct file  *filp,
	const char   *in_buf,
	size_t        len,
	loff_t       *off
)
{
	int ret;
	struct ca8210_priv *priv = filp->private_data;
	uint8_t command[CA8210_SPI_BUF_SIZE];

	if (len > CA8210_SPI_BUF_SIZE) {
		dev_warn(
			&priv->spi->dev,
			"userspace requested erroneously long write (%d)\n",
			len
		);
		return -EMSGSIZE;
	}

	ret = copy_from_user(command, in_buf, len);
	if (ret) {
		dev_err(
			&priv->spi->dev,
			"%d bytes could not be copied from userspace\n",
			ret
		);
		return -EIO;
	}

	ret = ca8210_test_check_upstream(command, priv->spi);
	if (ret == 0) {
		ret = ca8210_spi_exchange(
			command,
			command[1] + 2,
			NULL,
			priv->spi
		);
		if (ret < 0) {
			/* effectively 0 bytes were written successfully */
			dev_err(
				&priv->spi->dev,
				"spi exchange failed\n"
			);
			return ret;
		}
		if (command[0] & SPI_SYN) {
			priv->sync_down++;
		}
	}

	return len;
}

/**
 * ca8210_test_int_user_read() - Called by a process in userspace to read a
 *                               message from the ca8210 drivers
 * @filp:  file interface
 * @buf:   Buffer to write message to
 * @len:   length of message to read (ignored)
 * @offp:  file offset
 *
 * Return: 0 or linux error code
 */
static ssize_t ca8210_test_int_user_read(
	struct file  *filp,
	char __user  *buf,
	size_t        len,
	loff_t       *offp
)
{
	int i, cmdlen;
	struct ca8210_priv *priv = filp->private_data;
	unsigned char *fifo_buffer;
	unsigned long bytes_not_copied;

	if (filp->f_flags & O_NONBLOCK) {
		/* Non-blocking mode */
		if (kfifo_is_empty(&priv->test.up_fifo))
			return 0;
	} else {
		/* Blocking mode */
		wait_event_interruptible(
			priv->test.readq,
			!kfifo_is_empty(&priv->test.up_fifo)
		);
	}


	if (kfifo_out(&priv->test.up_fifo, &fifo_buffer, 4) != 4) {
		dev_err(
			&priv->spi->dev,
			"test_interface: Wrong number of elements popped " \
			"from upstream fifo\n"
		);
		return 0;
	}
	cmdlen = fifo_buffer[1];
	bytes_not_copied = cmdlen + 2;

	bytes_not_copied = copy_to_user(buf, fifo_buffer, bytes_not_copied);
	if (bytes_not_copied > 0) {
		dev_err(
			&priv->spi->dev,
			"%lu bytes could not be copied to user space!\n",
			bytes_not_copied
		);
	}

	kfree(fifo_buffer);

	dev_dbg(&priv->spi->dev, "test_interface: Cmd len = %d\n", cmdlen);

	dev_dbg(&priv->spi->dev, "test_interface: Read\n");
	for (i = 0; i < cmdlen+2; i++) {
		dev_dbg(&priv->spi->dev, "%#03x\n", buf[i]);
	}

	return cmdlen+2;
}

static long ca8210_test_int_ioctl(
	struct file *filp,
	unsigned int ioctl_num,
	unsigned long ioctl_param
)
{
	struct ca8210_priv *priv = filp->private_data;
	switch (ioctl_num) {
	case CA8210_IOCTL_HARD_RESET:
		ca8210_reset_send(priv->spi, ioctl_param);
		break;
	default:
		break;
	}
	return 0;
}

static unsigned int ca8210_test_int_poll(
	struct file *filp,
	struct poll_table_struct *ptable
)
{
	unsigned int return_flags = 0;
	struct ca8210_priv *priv = filp->private_data;
	poll_wait(filp, &priv->test.readq, ptable);
	if (!kfifo_is_empty(&priv->test.up_fifo)) {
		return_flags |= (POLLIN | POLLRDNORM);
	}
	if (wait_event_interruptible(
		priv->test.readq,
		!kfifo_is_empty(&priv->test.up_fifo))) {
		return POLLERR;
	}
	return return_flags;
}

static const struct file_operations test_int_fops = {
	.read =           ca8210_test_int_user_read,
	.write =          ca8210_test_int_user_write,
	.open =           ca8210_test_int_open,
	.release =        NULL,
	.unlocked_ioctl = ca8210_test_int_ioctl,
	.poll =           ca8210_test_int_poll
};

/* Init/Deinit */

/**
 * ca8210_get_platform_data() - Populate a ca8210_platform_data object
 * @spi_device:  Pointer to ca8210 spi device object to get data for
 * @pdata:       Pointer to ca8210_platform_data object to populate
 *
 * Return: 0 or linux error code
 */
static int ca8210_get_platform_data(
	struct spi_device *spi_device,
	struct ca8210_platform_data *pdata
)
{
	int ret = 0;

	if (spi_device->dev.of_node == NULL)
		return -EINVAL;

	pdata->extclockenable = of_property_read_bool(spi_device->dev.of_node,
	                                              "extclock-enable");
	if (pdata->extclockenable) {
		ret = of_property_read_u32(spi_device->dev.of_node,
		                           "extclock-freq",
		                           &pdata->extclockfreq);
		if (ret < 0)
			return ret;

		ret = of_property_read_u32(spi_device->dev.of_node,
		                           "extclock-gpio",
		                           &pdata->extclockgpio);
	}

	return ret;
}

/**
 * ca8210_config_extern_clk() - Configure the external clock provided by the
 *                              ca8210
 * @pdata:  Pointer to ca8210_platform_data containing clock parameters
 * @spi:    Pointer to target ca8210 spi device
 * @on:	    True to turn the clock on, false to turn off
 *
 * The external clock is configured with a frequency and output pin taken from
 * the platform data.
 *
 * Return: 0 or linux error code
 */
static int ca8210_config_extern_clk(
	struct ca8210_platform_data *pdata,
	struct spi_device *spi,
	bool on
)
{
	uint8_t clkparam[2];

	if (on) {
		dev_info(&spi->dev, "Switching external clock on\n");
		switch (pdata->extclockfreq) {
		case SIXTEEN_MHZ:
			clkparam[0] = 1;
			break;
		case EIGHT_MHZ:
			clkparam[0] = 2;
			break;
		case FOUR_MHZ:
			clkparam[0] = 3;
			break;
		case TWO_MHZ:
			clkparam[0] = 4;
			break;
		case ONE_MHZ:
			clkparam[0] = 5;
			break;
		default:
			dev_crit(&spi->dev, "Invalid extclock-freq\n");
			return -EINVAL;
		}
		clkparam[1] = pdata->extclockgpio;
	} else {
		dev_info(&spi->dev, "Switching external clock off\n");
		clkparam[0] = 0; /* off */
		clkparam[1] = 0;
	}
	return link_to_linux_err(
		hwme_set_request_sync(HWME_SYSCLKOUT, 2, clkparam, spi)
	);
}

/**
 * ca8210_register_ext_clock() - Register ca8210's external clock with kernel
 * @spi:  Pointer to target ca8210 spi device
 *
 * Return: 0 or linux error code
 */
static int ca8210_register_ext_clock(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct ca8210_priv *priv = spi_get_drvdata(spi);
	struct ca8210_platform_data *pdata = spi->dev.platform_data;
	int ret = 0;

	if (!np)
		return -EFAULT;

	priv->clk = clk_register_fixed_rate(&spi->dev,
	                                    np->name,
	                                    NULL,
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 6, 0)
	                                    0,
#else
	                                    CLK_IS_ROOT,
#endif
	                                    pdata->extclockfreq);

	if (IS_ERR(priv->clk)) {
		dev_crit(&spi->dev, "Failed to register external clk\n");
		return PTR_ERR(priv->clk);
	}
	ret = of_clk_add_provider(np, of_clk_src_simple_get, priv->clk);
	if (ret) {
		clk_unregister(priv->clk);
		dev_crit(
			&spi->dev,
			"Failed to register external clock as clock provider\n"
		);
	} else {
		dev_info(&spi->dev, "External clock set as clock provider\n");
	}

	return ret;
}

/**
 * ca8210_unregister_ext_clock() - Unregister ca8210's external clock with
 *                                 kernel
 * @spi:  Pointer to target ca8210 spi device
 */
static void ca8210_unregister_ext_clock(struct spi_device *spi)
{
	struct ca8210_priv *priv = spi_get_drvdata(spi);

	if (priv->clk == NULL)
		return

	of_clk_del_provider(spi->dev.of_node);
	clk_unregister(priv->clk);
	dev_info(&spi->dev, "External clock unregistered\n");
}

/**
 * ca8210_reset_init() - Initialise the reset input to the ca8210
 * @spi:  Pointer to target ca8210 spi device
 *
 * Return: 0 or linux error code
 */
static int ca8210_reset_init(struct spi_device *spi)
{
	int ret;
	struct ca8210_platform_data *pdata = spi->dev.platform_data;

	pdata->gpio_reset = of_get_named_gpio(spi->dev.of_node,
	                                      "reset-gpio",
	                                      0);

	ret = gpio_direction_output(pdata->gpio_reset, 1);
	if (ret < 0) {
		dev_crit(
			&spi->dev,
			"Reset GPIO %d did not set to output mode\n",
			pdata->gpio_reset);
	}

	return ret;
}

/**
 * ca8210_interrupt_init() - Initialise the irq output from the ca8210
 * @spi:  Pointer to target ca8210 spi device
 *
 * Return: 0 or linux error code
 */
static int ca8210_interrupt_init(struct spi_device *spi)
{
	int ret;
	struct ca8210_platform_data *pdata = spi->dev.platform_data;

	pdata->gpio_irq = of_get_named_gpio(spi->dev.of_node,
	                                    "irq-gpio",
	                                    0);

	pdata->irq_id = gpio_to_irq(pdata->gpio_irq);
	if (pdata->irq_id < 0) {
		dev_crit(&spi->dev, "Could not get irq for gpio pin %d\n",
			pdata->gpio_irq);
		gpio_free(pdata->gpio_irq);
		return pdata->irq_id;
	}

	ret = request_irq(pdata->irq_id,
	                  ca8210_interrupt_handler,
	                  IRQF_TRIGGER_FALLING,
	                  "ca8210-irq",
	                  spi_get_drvdata(spi));
	if (ret) {
		dev_crit(&spi->dev, "request_irq %d failed\n", pdata->irq_id);
		gpio_unexport(pdata->gpio_irq);
		gpio_free(pdata->gpio_irq);
	}

	return ret;
}

/**
 * ca8210_dev_com_init() - Initialise the spi communication component
 * @priv:  Pointer to private data structure
 *
 * Return: 0 or linux error code
 */
static int ca8210_dev_com_init(struct ca8210_priv *priv)
{
	int status;

	mutex_init(&priv->cas_ctl.spi_mutex);

	priv->cas_ctl.tx_buf = kmalloc(
		CA8210_SPI_BUF_SIZE,
		GFP_DMA | GFP_KERNEL
	);
	if (priv->cas_ctl.tx_buf == NULL) {
		status = -EFAULT;
		goto error;
	}
	memset(priv->cas_ctl.tx_buf, SPI_IDLE, CA8210_SPI_BUF_SIZE);

	priv->cas_ctl.tx_in_buf = kmalloc(
		CA8210_SPI_BUF_SIZE,
		GFP_DMA | GFP_KERNEL
	);
	if (priv->cas_ctl.tx_in_buf == NULL) {
		status = -EFAULT;
		goto error;
	}
	memset(priv->cas_ctl.tx_in_buf, SPI_IDLE, CA8210_SPI_BUF_SIZE);

	priv->cas_ctl.rx_buf = kmalloc(
		CA8210_SPI_BUF_SIZE,
		GFP_DMA | GFP_KERNEL
	);
	if (priv->cas_ctl.rx_buf == NULL) {
		status = -EFAULT;
		goto error;
	}
	memset(priv->cas_ctl.rx_buf, SPI_IDLE, CA8210_SPI_BUF_SIZE);

	priv->cas_ctl.rx_out_buf = kmalloc(
		CA8210_SPI_BUF_SIZE,
		GFP_DMA | GFP_KERNEL
	);
	if (priv->cas_ctl.rx_out_buf == NULL) {
		status = -EFAULT;
		goto error;
	}
	memset(priv->cas_ctl.rx_out_buf, SPI_IDLE, CA8210_SPI_BUF_SIZE);

	priv->cas_ctl.rx_final_buf = kmalloc(
		CA8210_SPI_BUF_SIZE,
		GFP_DMA | GFP_KERNEL
	);
	if (priv->cas_ctl.rx_final_buf == NULL) {
		status = -EFAULT;
		goto error;
	}
	memset(priv->cas_ctl.rx_final_buf, SPI_IDLE, CA8210_SPI_BUF_SIZE);

	priv->cas_ctl.tx_transfer.tx_nbits = 1; /* 1 MOSI line */
	priv->cas_ctl.tx_transfer.rx_nbits = 1; /* 1 MISO line */
	priv->cas_ctl.tx_transfer.speed_hz = 0; /* Use device setting */
	priv->cas_ctl.tx_transfer.bits_per_word = 0; /* Use device setting */
	priv->cas_ctl.rx_transfer.tx_nbits = 1; /* 1 MOSI line */
	priv->cas_ctl.rx_transfer.rx_nbits = 1; /* 1 MISO line */
	priv->cas_ctl.rx_transfer.speed_hz = 0; /* Use device setting */
	priv->cas_ctl.rx_transfer.bits_per_word = 0; /* Use device setting */

	priv->mlme_workqueue = alloc_ordered_workqueue(
		"MLME work queue",
		WQ_UNBOUND
	);
	if (priv->mlme_workqueue == NULL) {
		dev_crit(&priv->spi->dev, "alloc of mlme_workqueue failed!\n");
	}

	priv->irq_workqueue = alloc_ordered_workqueue(
		"ca8210 irq worker",
		WQ_UNBOUND
	);
	if (priv->irq_workqueue == NULL) {
		dev_crit(&priv->spi->dev, "alloc of irq_workqueue failed!\n");
	}

	return 0;

error:
	kfree(priv->cas_ctl.rx_buf);
	priv->cas_ctl.rx_buf = NULL;
	kfree(priv->cas_ctl.tx_in_buf);
	priv->cas_ctl.tx_in_buf = NULL;
	kfree(priv->cas_ctl.tx_buf);
	priv->cas_ctl.tx_buf = NULL;
	kfree(priv->cas_ctl.rx_out_buf);
	priv->cas_ctl.rx_out_buf = NULL;
	kfree(priv->cas_ctl.rx_final_buf);
	priv->cas_ctl.rx_final_buf = NULL;

	return status;
}

/**
 * ca8210_dev_com_clear() - Deinitialise the spi communication component
 * @priv:  Pointer to private data structure
 */
static void ca8210_dev_com_clear(struct ca8210_priv *priv)
{
	flush_workqueue(priv->mlme_workqueue);
	destroy_workqueue(priv->mlme_workqueue);
	flush_workqueue(priv->irq_workqueue);
	destroy_workqueue(priv->irq_workqueue);

	kfree(priv->cas_ctl.rx_buf);
	priv->cas_ctl.rx_buf = NULL;
	kfree(priv->cas_ctl.tx_in_buf);
	priv->cas_ctl.tx_in_buf = NULL;
	kfree(priv->cas_ctl.tx_buf);
	priv->cas_ctl.tx_buf = NULL;
	kfree(priv->cas_ctl.rx_out_buf);
	priv->cas_ctl.rx_out_buf = NULL;
	kfree(priv->cas_ctl.rx_final_buf);
	priv->cas_ctl.rx_final_buf = NULL;
}

/**
 * ca8210_hw_setup() - Populate the ieee802154_hw phy attributes with the
 *                     ca8210's defaults
 * @ca8210_hw:  Pointer to ieee802154_hw to populate
 */
static void ca8210_hw_setup(struct ieee802154_hw *ca8210_hw)
{
	/* Support channels 11-26 */
	ca8210_hw->phy->current_channel = 18;
	ca8210_hw->phy->current_page = 0;
	ca8210_hw->phy->transmit_power = 800;
	ca8210_hw->phy->cca.mode = NL802154_CCA_ENERGY_CARRIER;
	ca8210_hw->phy->cca.opt = NL802154_CCA_OPT_ENERGY_CARRIER_AND;
	ca8210_hw->phy->cca_ed_level = 0x3C;
	ca8210_hw->phy->symbol_duration = 16;
	ca8210_hw->phy->lifs_period = 40;
	ca8210_hw->phy->sifs_period = 12;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0)
	ca8210_hw->phy->supported.channels[0] = CA8210_VALID_CHANNELS;
	ca8210_hw->flags = IEEE802154_HW_AFILT |
	                   IEEE802154_HW_OMIT_CKSUM |
	                   IEEE802154_HW_FRAME_RETRIES |
	                   IEEE802154_HW_CSMA_PARAMS;
	ca8210_hw->phy->flags = WPAN_PHY_FLAG_TXPOWER |
	                        WPAN_PHY_FLAG_CCA_ED_LEVEL |
	                        WPAN_PHY_FLAG_CCA_MODE;
#else
	ca8210_hw->phy->channels_supported[0] = CA8210_VALID_CHANNELS;
	ca8210_hw->flags = IEEE802154_HW_TXPOWER |
	                   IEEE802154_HW_AFILT |
	                   IEEE802154_HW_OMIT_CKSUM |
	                   IEEE802154_HW_ARET;
#endif
}

/**
 * ca8210_test_interface_init() - Initialise the test file interface
 * @priv:  Pointer to private data structure
 *
 * Provided as an alternative to the standard linux network interface, the test
 * interface exposes a file in the filesystem (ca8210_test) that allows
 * 802.15.4 SAP Commands and Cascoda EVBME commands to be sent directly to
 * the stack.
 *
 * Return: 0 or linux error code
 */
static int ca8210_test_interface_init(struct ca8210_priv *priv)
{
	struct ca8210_test *test = &priv->test;
	char node_name[32];

	snprintf(
		node_name,
		sizeof(node_name),
		"ca8210@%d_%d",
		priv->spi->master->bus_num,
		priv->spi->chip_select
	);

	test->ca8210_dfs_spi_int = debugfs_create_file(
		node_name,
		S_IRUSR | S_IWUSR,
		NULL,
		priv,
		&test_int_fops
	);
	if (IS_ERR(test->ca8210_dfs_spi_int)) {
		dev_err(
			&priv->spi->dev,
			"Error %ld when creating debugfs node\n",
			PTR_ERR(test->ca8210_dfs_spi_int)
		);
		return PTR_ERR(test->ca8210_dfs_spi_int);
	}
	debugfs_create_symlink("ca8210", NULL, node_name);
	init_waitqueue_head(&test->readq);
	return kfifo_alloc(
		&test->up_fifo,
		CA8210_TEST_INT_FIFO_SIZE,
		GFP_KERNEL
	);
}

/**
 * ca8210_test_interface_clear() - Deinitialise the test file interface
 * @priv:  Pointer to private data structure
 */
static void ca8210_test_interface_clear(struct ca8210_priv *priv)
{
	struct ca8210_test *test = &priv->test;

	if (!IS_ERR(test->ca8210_dfs_spi_int)) {
		debugfs_remove(test->ca8210_dfs_spi_int);
	}
	kfifo_free(&test->up_fifo);
	dev_info(&priv->spi->dev, "Test interface removed\n");
}

/**
 * ca8210_remove() - Shut down a ca8210 upon being disconnected
 * @priv:  Pointer to private data structure
 *
 * Return: 0 or linux error code
 */
static int ca8210_remove(struct spi_device *spi_device)
{
	struct ca8210_priv *priv;
	struct ca8210_platform_data *pdata;

	dev_info(&spi_device->dev, "Removing ca8210\n");

	pdata = spi_device->dev.platform_data;
	if (pdata) {
		if (pdata->extclockenable) {
			ca8210_unregister_ext_clock(spi_device);
			ca8210_config_extern_clk(pdata, spi_device, 0);
		}
		free_irq(pdata->irq_id, spi_device->dev.driver_data);
		kfree(pdata);
		spi_device->dev.platform_data = NULL;
	}
	/* get spi_device private data */
	priv = spi_get_drvdata(spi_device);
	if (priv) {
		dev_info(
			&spi_device->dev,
			"sync_down = %d, sync_up = %d\n",
			priv->sync_down,
			priv->sync_up
		);
		ca8210_dev_com_clear(spi_device->dev.driver_data);
		if (priv->hw) {
			if (priv->hw_registered) {
				ieee802154_unregister_hw(priv->hw);
			}
			ieee802154_free_hw(priv->hw);
			priv->hw = NULL;
			dev_info(
				&spi_device->dev,
				"Unregistered & freed ieee802154_hw.\n"
			);
		}
		ca8210_test_interface_clear(priv);
	}

	return 0;
}

/**
 * ca8210_probe() - Set up a connected ca8210 upon being detected by the system
 * @priv:  Pointer to private data structure
 *
 * Return: 0 or linux error code
 */
static int ca8210_probe(struct spi_device *spi_device)
{
	struct ca8210_priv *priv;
	struct ieee802154_hw *hw;
	struct ca8210_platform_data *pdata;
	int ret;

	dev_info(&spi_device->dev, "Inserting ca8210\n");

	/* allocate ieee802154_hw and private data */
	hw = ieee802154_alloc_hw(sizeof(struct ca8210_priv), &ca8210_phy_ops);
	if (hw == NULL) {
		dev_crit(&spi_device->dev, "ieee802154_alloc_hw failed\n");
		ret = -ENOMEM;
		goto error;
	}

	priv = hw->priv;
	priv->hw = hw;
	priv->spi = spi_device;
	hw->parent = &spi_device->dev;
	spin_lock_init(&priv->lock);
	priv->async_tx_pending = false;
	priv->sync_command_pending = false;
	priv->hw_registered = false;
	priv->spi_errno = 0;
	priv->sync_up = 0;
	priv->sync_down = 0;
	mutex_init(&priv->sync_command_mutex);
	atomic_set(&priv->ca8210_is_awake, 0);
	spi_set_drvdata(priv->spi, priv);

	if (IS_ENABLED(CONFIG_IEEE802154_CA8210_DEBUGFS)) {
		ca8210_test_interface_init(priv);
	}
	ca8210_hw_setup(hw);
	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

	pdata = kmalloc(sizeof(struct ca8210_platform_data), GFP_KERNEL);
	if (pdata == NULL) {
		dev_crit(
			&spi_device->dev,
			"Could not allocate platform data\n"
		);
		ret = -ENOMEM;
		goto error;
	}

	ret = ca8210_get_platform_data(priv->spi, pdata);
	if (ret) {
		dev_crit(&spi_device->dev, "ca8210_get_platform_data failed\n");
		goto error;
	}
	priv->spi->dev.platform_data = pdata;

	ret = ca8210_dev_com_init(priv);
	if (ret) {
		dev_crit(&spi_device->dev, "ca8210_dev_com_init failed\n");
		goto error;
	}
	ret = ca8210_reset_init(priv->spi);
	if (ret) {
		dev_crit(&spi_device->dev, "ca8210_reset_init failed\n");
		goto error;
	}

	ret = ca8210_interrupt_init(priv->spi);
	if (ret) {
		dev_crit(&spi_device->dev, "ca8210_interrupt_init failed\n");
		goto error;
	}

	msleep(100);

	ca8210_reset_send(priv->spi, 1);

	ret = tdme_chipinit(priv->spi);
	if (ret) {
		dev_crit(&spi_device->dev, "tdme_chipinit failed\n");
		goto error;
	}

	if (pdata->extclockenable) {
		ret = ca8210_config_extern_clk(pdata, priv->spi, 1);
		if (ret) {
			dev_crit(
				&spi_device->dev,
				"ca8210_config_extern_clk failed\n"
			);
			goto error;
		}
		ret = ca8210_register_ext_clock(priv->spi);
		if (ret) {
			dev_crit(
				&spi_device->dev,
				"ca8210_register_ext_clock failed\n"
			);
			goto error;
		}
	}

	ret = ieee802154_register_hw(hw);
	if (ret) {
		dev_crit(&spi_device->dev, "ieee802154_register_hw failed\n");
		goto error;
	}
	priv->hw_registered = true;

	return 0;
error:
	msleep(100); /* wait for pending spi transfers to complete */
	ca8210_remove(spi_device);
	return link_to_linux_err(ret);
}

static const struct of_device_id ca8210_of_ids[] = {
	{.compatible = "cascoda,ca8210", },
	{},
};
MODULE_DEVICE_TABLE(of, ca8210_of_ids);

static struct spi_driver ca8210_spi_driver = {
	.driver = {
		.name =                 DRIVER_NAME,
		.owner =                THIS_MODULE,
		.of_match_table =       of_match_ptr(ca8210_of_ids),
	},
	.probe  =                       ca8210_probe,
	.remove =                       ca8210_remove
};

static int __init ca8210_init(void)
{
	pr_info("Starting module ca8210\n");

	if (IS_ENABLED(CONFIG_IEEE802154_CA8210_DEBUGFS))
		cascoda_api_upstream = ca8210_test_int_driver_write;
	else
		cascoda_api_upstream = NULL;

	spi_register_driver(&ca8210_spi_driver);
	pr_info("ca8210 module started\n");
	return 0;
}
module_init(ca8210_init);

static void __exit ca8210_exit(void)
{
	pr_info("Stopping module ca8210\n");
	spi_unregister_driver(&ca8210_spi_driver);
	pr_info("ca8210 module stopped\n");
}
module_exit(ca8210_exit);

MODULE_AUTHOR("Harry Morris <h.morris@cascoda.com>");
MODULE_DESCRIPTION("CA-8210 SoftMAC driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("1.0");
