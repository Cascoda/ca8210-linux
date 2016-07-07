/*
 * Copyright (c) 2016, Cascoda
 * All rights reserved.
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
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/ieee802154.h>
#include <linux/kfifo.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include <net/ieee802154_netdev.h>
#include <net/mac802154.h>

/******************************************************************************/

#define DRIVER_NAME "ca8210"
#define FUNCTION_VERSION "Linux Complete Kernel Module"

/* external clock frequencies */
#define ONE_MHZ      1000000
#define TWO_MHZ      2*ONE_MHZ
#define FOUR_MHZ     4*ONE_MHZ
#define EIGHT_MHZ    8*ONE_MHZ
#define SIXTEEN_MHZ  16*ONE_MHZ

/* spi constants */
#define SPI_BUF_SIZE 256
#define SPI_T_TIMEOUT 500     /* Timeout for NACKs [ms] */

/* api constants */
#define DATA_CNF_TIMEOUT_MS 300

/* test interface constants */
#define TEST_INT_FILE_NAME "ca8210_test"
#define TEST_INT_FIFO_SIZE 256

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
#define HWME_POWERCON          (0x00)
#define HWME_CHIPID            (0x01)
#define HWME_TXPOWER           (0x02)
#define HWME_CCAMODE           (0x03)
#define HWME_EDTHRESHOLD       (0x04)
#define HWME_CSTHRESHOLD       (0x05)
#define HWME_EDVALUE           (0x06)
#define HWME_CSVALUE           (0x07)
#define HWME_EDVALLP           (0x08)
#define HWME_CSVALLP           (0x09)
#define HWME_FREQOFFS          (0x0A)
#define HWME_MACTIMER          (0x0B)
#define HWME_RANDOMNUM         (0x0C)
#define HWME_TEMPERATURE       (0x0D)
#define HWME_HSKEY             (0x0E)
#define HWME_SYSCLKOUT         (0x0F)
#define HWME_LQIMODE           (0x10)
#define HWME_MIN_ATTRIBUTE     HWME_POWERCON
#define HWME_MAX_ATTRIBUTE     HWME_LQIMODE

/* TDME attribute IDs */
#define TDME_CHANNEL          (0x00)
#define TDME_TX_CONFIG        (0x01)
#define TDME_ED_CONFIG        (0x02)
#define TDME_RX_CONFIG        (0x03)
#define TDME_LO_1_CONFIG      (0x04)
#define TDME_LO_2_CONFIG      (0x05)
#define TDME_ATM_CONFIG       (0x06)
#define TDME_MIN_ATTRIBUTE    TDME_CHANNEL
#define TDME_MAX_ATTRIBUTE    TDME_ATM_CONFIG

#define MAX_HWME_ATTRIBUTE_SIZE  16
#define MAX_TDME_ATTRIBUTE_SIZE  2

/* PHY/MAC PIB Attribute Enumerations */
#define phyCurrentChannel               (0x00)
#define phyChannelsSupport              (0x01)
#define phyTransmitPower                (0x02)
#define phyCCAMode                      (0x03)
#define phyCurrentPage                  (0x04)
#define phyMaxFrameDuration             (0x05)
#define phySHRDuration                  (0x06)
#define phySymbolsPerOctet              (0x07)

#define phyPibFirst                     (phyCurrentChannel)
#define phyPibLast                      (phySymbolsPerOctet)

#define macAckWaitDuration              (0x40)
#define macAssociationPermit            (0x41)
#define macAutoRequest                  (0x42)
#define macBattLifeExt                  (0x43)
#define macBattLifeExtPeriods           (0x44)
#define macBeaconPayload                (0x45)
#define macBeaconPayloadLength          (0x46)
#define macBeaconOrder                  (0x47)
#define macBeaconTxTime                 (0x48)
#define macBSN                          (0x49)
#define macCoordExtendedAddress         (0x4a)
#define macCoordShortAddress            (0x4b)
#define macDSN                          (0x4c)
#define macGTSPermit                    (0x4d)
#define macMaxCSMABackoffs              (0x4e)
#define macMinBE                        (0x4f)
#define macPANId                        (0x50)
#define macPromiscuousMode              (0x51)
#define macRxOnWhenIdle                 (0x52)
#define macShortAddress                 (0x53)
#define macSuperframeOrder              (0x54)
#define macTransactionPersistenceTime	(0x55)
#define macAssociatedPANCoord           (0x56)
#define macMaxBE                        (0x57)
#define macMaxFrameTotalWaitTime        (0x58)
#define macMaxFrameRetries              (0x59)
#define macResponseWaitTime             (0x5A)
#define macSyncSymbolOffset             (0x5B)
#define macTimestampSupported           (0x5C)
#define macSecurityEnabled              (0x5D)

#define macPibFirst                     (macAckWaitDuration)
#define macPibLast                      (macSecurityEnabled)

#define macKeyTable                     (0x71)
#define macKeyTableEntries              (0x72)
#define macDeviceTable                  (0x73)
#define macDeviceTableEntries           (0x74)
#define macSecurityLevelTable           (0x75)
#define macSecurityLevelTableEntries    (0x76)
#define macFrameCounter                 (0x77)
#define macAutoRequestSecurityLevel     (0x78)
#define macAutoRequestKeyIdMode         (0x79)
#define macAutoRequestKeySource         (0x7A)
#define macAutoRequestKeyIndex          (0x7B)
#define macDefaultKeySource             (0x7C)
#define macPANCoordExtendedAddress      (0x7D)
#define macPANCoordShortAddress         (0x7E)

#define macSecPibFirst                  (macKeyTable)
#define macSecPibLast                   (macPANCoordShortAddress)

#define nsIEEEAddress                   (0xFF)    /* Non-standard IEEE address */

/* MAC Address Mode Definitions */
#define MAC_MODE_NO_ADDR                ((unsigned)0x00)
#define MAC_MODE_RESERVED               ((unsigned)0x01)
#define MAC_MODE_SHORT_ADDR             ((unsigned)0x02)
#define MAC_MODE_LONG_ADDR              ((unsigned)0x03)

/* MAC constants */
#define aMaxPHYPacketSize               (127)
#define aMaxBeaconOverhead              (75)
#define aMaxBeaconPayloadLength         (aMaxPHYPacketSize-aMaxBeaconOverhead)

#define MAX_ATTRIBUTE_SIZE              (250)
#define MAX_DATA_SIZE                   (114)

#define M_MinimumChannel                (11)
#define M_MaximumChannel                (26)
#define M_ValidChannels                 (0x07FFF800)

/* Data Request includes 1 phylen, 2 fc, 1 dsn, 12 address, 1 payload, 2 fcs */
#define BASELEN_DATA_REQUEST            (19)

/* MAC workarounds for V1.1 and MPW silicon (V0.x) */
#define MAC_Workarounds (0)
#define MAC_MPW         (0)

/* memory manipulation macros */
#define LS_BYTE(x)     ((uint8_t)((x)&0xFF))
#define MS_BYTE(x)     ((uint8_t)(((x)>>8)&0xFF))
#define LS0_BYTE(x)    ((uint8_t)((x)&0xFF))
#define LS1_BYTE(x)    ((uint8_t)(((x)>>8)&0xFF))
#define LS2_BYTE(x)    ((uint8_t)(((x)>>16)&0xFF))
#define LS3_BYTE(x)    ((uint8_t)(((x)>>24)&0xFF))
#define GETLE16(x)     (((uint16_t)(x)[1]<<8)+(x)[0])
#define GETLE32(x)     (((uint32_t)(x)[3]<<24)+((uint32_t)(x)[2]<<16)+((uint32_t)(x)[1]<<8)+(x)[0])
#define PUTLE16(x,y)   {(y)[0]=((x)&0xff);(y)[1]=((x)>>8);}
#define PUTLE32(x,y)   {(y)[0]=((x)&0xff);(y)[1]=(((x)>>8)&0xff);(y)[2]=(((x)>>16)&0xff);(y)[3]=(((x)>>24)&0xff);}

/* message ID codes in SPI commands */
/* downstream */
#define MCPS_DATA_REQUEST                     (0x00)
#define MCPS_PURGE_REQUEST                    (0x01)
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
#define MLME_SYNC_REQUEST                     (0x0C)
#define MLME_POLL_REQUEST                     (0x0D)
#define HWME_SET_REQUEST                      (0x0E)
#define HWME_GET_REQUEST                      (0x0F)
#define HWME_HAES_REQUEST                     (0x10)
#define TDME_SETSFR_REQUEST                   (0x11)
#define TDME_GETSFR_REQUEST                   (0x12)
#define TDME_TESTMODE_REQUEST                 (0x13)
#define TDME_SET_REQUEST                      (0x14)
#define TDME_TXPKT_REQUEST                    (0x15)
#define TDME_LOTLK_REQUEST                    (0x16)
/* upstream */
#define MCPS_DATA_INDICATION                  (0x00)
#define MCPS_DATA_CONFIRM                     (0x01)
#define MCPS_PURGE_CONFIRM                    (0x02)
#define MLME_ASSOCIATE_INDICATION             (0x03)
#define MLME_ASSOCIATE_CONFIRM                (0x04)
#define MLME_DISASSOCIATE_INDICATION          (0x05)
#define MLME_DISASSOCIATE_CONFIRM             (0x06)
#define MLME_BEACON_NOTIFY_INDICATION         (0x07)
#define MLME_GET_CONFIRM                      (0x08)
#define MLME_ORPHAN_INDICATION                (0x09)
#define MLME_RESET_CONFIRM                    (0x0A)
#define MLME_RX_ENABLE_CONFIRM                (0x0B)
#define MLME_SCAN_CONFIRM                     (0x0C)
#define MLME_COMM_STATUS_INDICATION           (0x0D)
#define MLME_SET_CONFIRM                      (0x0E)
#define MLME_START_CONFIRM                    (0x0F)
#define MLME_SYNC_LOSS_INDICATION             (0x10)
#define MLME_POLL_CONFIRM                     (0x11)
#define HWME_SET_CONFIRM                      (0x12)
#define HWME_GET_CONFIRM                      (0x13)
#define HWME_HAES_CONFIRM                     (0x14)
#define HWME_WAKEUP_INDICATION                (0x15)
#define TDME_MESSAGE_INDICATION               (0x16)
#define TDME_SETSFR_CONFIRM                   (0x17)
#define TDME_GETSFR_CONFIRM                   (0x18)
#define TDME_TESTMODE_CONFIRM                 (0x19)
#define TDME_SET_CONFIRM                      (0x1A)
#define TDME_TXPKT_CONFIRM                    (0x1B)
#define TDME_RXPKT_INDICATION                 (0x1C)
#define TDME_EDDET_INDICATION                 (0x1D)
#define TDME_ERROR_INDICATION                 (0x1E)
#define TDME_LOTLK_CONFIRM                    (0x1F)


/* SPI command IDs */
/* mask to derive the message ID code from the command ID */
#define SPI_MID_MASK                       (0x1F)
/* bit indicating a confirm or indication from slave to master */
#define SPI_S2M                            (0x20)
/* bit indicating a synchronous message */
#define SPI_SYN                            (0x40)

/* SPI command definitions */
#define SPI_IDLE                           (0xFF)
#define SPI_NACK                           (0xF0)

#define SPI_MCPS_DATA_REQUEST              (MCPS_DATA_REQUEST)
#define SPI_MCPS_PURGE_REQUEST             (MCPS_PURGE_REQUEST+SPI_SYN)
#define SPI_MCPS_DATA_INDICATION           (MCPS_DATA_INDICATION+SPI_S2M)
#define SPI_MCPS_DATA_CONFIRM              (MCPS_DATA_CONFIRM+SPI_S2M)
#define SPI_MCPS_PURGE_CONFIRM             (MCPS_PURGE_CONFIRM+SPI_S2M+SPI_SYN)

#define SPI_MLME_ASSOCIATE_REQUEST         (MLME_ASSOCIATE_REQUEST)
#define SPI_MLME_ASSOCIATE_RESPONSE        (MLME_ASSOCIATE_RESPONSE)
#define SPI_MLME_DISASSOCIATE_REQUEST      (MLME_DISASSOCIATE_REQUEST)
#define SPI_MLME_GET_REQUEST               (MLME_GET_REQUEST+SPI_SYN)
#define SPI_MLME_ORPHAN_RESPONSE           (MLME_ORPHAN_RESPONSE)
#define SPI_MLME_RESET_REQUEST             (MLME_RESET_REQUEST+SPI_SYN)
#define SPI_MLME_RX_ENABLE_REQUEST         (MLME_RX_ENABLE_REQUEST+SPI_SYN)
#define SPI_MLME_SCAN_REQUEST              (MLME_SCAN_REQUEST)
#define SPI_MLME_SET_REQUEST               (MLME_SET_REQUEST+SPI_SYN)
#define SPI_MLME_START_REQUEST             (MLME_START_REQUEST+SPI_SYN)
#define SPI_MLME_SYNC_REQUEST              (MLME_SYNC_REQUEST)
#define SPI_MLME_POLL_REQUEST              (MLME_POLL_REQUEST+SPI_SYN)
#define SPI_MLME_ASSOCIATE_INDICATION      (MLME_ASSOCIATE_INDICATION+SPI_S2M)
#define SPI_MLME_ASSOCIATE_CONFIRM         (MLME_ASSOCIATE_CONFIRM+SPI_S2M)
#define SPI_MLME_DISASSOCIATE_INDICATION   (MLME_DISASSOCIATE_INDICATION+SPI_S2M)
#define SPI_MLME_DISASSOCIATE_CONFIRM      (MLME_DISASSOCIATE_CONFIRM+SPI_S2M)
#define SPI_MLME_BEACON_NOTIFY_INDICATION  (MLME_BEACON_NOTIFY_INDICATION+SPI_S2M)
#define SPI_MLME_GET_CONFIRM               (MLME_GET_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_MLME_ORPHAN_INDICATION         (MLME_ORPHAN_INDICATION+SPI_S2M)
#define SPI_MLME_RESET_CONFIRM             (MLME_RESET_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_MLME_RX_ENABLE_CONFIRM         (MLME_RX_ENABLE_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_MLME_SCAN_CONFIRM              (MLME_SCAN_CONFIRM+SPI_S2M)
#define SPI_MLME_COMM_STATUS_INDICATION    (MLME_COMM_STATUS_INDICATION+SPI_S2M)
#define SPI_MLME_SET_CONFIRM               (MLME_SET_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_MLME_START_CONFIRM             (MLME_START_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_MLME_SYNC_LOSS_INDICATION      (MLME_SYNC_LOSS_INDICATION+SPI_S2M)
#define SPI_MLME_POLL_CONFIRM              (MLME_POLL_CONFIRM+SPI_S2M+SPI_SYN)

#define SPI_HWME_SET_REQUEST               (HWME_SET_REQUEST+SPI_SYN)
#define SPI_HWME_GET_REQUEST               (HWME_GET_REQUEST+SPI_SYN)
#define SPI_HWME_HAES_REQUEST              (HWME_HAES_REQUEST+SPI_SYN)
#define SPI_HWME_SET_CONFIRM               (HWME_SET_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_HWME_GET_CONFIRM               (HWME_GET_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_HWME_HAES_CONFIRM              (HWME_HAES_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_HWME_WAKEUP_INDICATION         (HWME_WAKEUP_INDICATION+SPI_S2M)

#define SPI_TDME_SETSFR_REQUEST            (TDME_SETSFR_REQUEST+SPI_SYN)
#define SPI_TDME_GETSFR_REQUEST            (TDME_GETSFR_REQUEST+SPI_SYN)
#define SPI_TDME_TESTMODE_REQUEST          (TDME_TESTMODE_REQUEST+SPI_SYN)
#define SPI_TDME_SET_REQUEST               (TDME_SET_REQUEST+SPI_SYN)
#define SPI_TDME_TXPKT_REQUEST             (TDME_TXPKT_REQUEST+SPI_SYN)
#define SPI_TDME_LOTLK_REQUEST             (TDME_LOTLK_REQUEST+SPI_SYN)
#define SPI_TDME_MESSAGE_INDICATION        (TDME_MESSAGE_INDICATION+SPI_S2M)
#define SPI_TDME_SETSFR_CONFIRM            (TDME_SETSFR_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_TDME_GETSFR_CONFIRM            (TDME_GETSFR_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_TDME_TESTMODE_CONFIRM          (TDME_TESTMODE_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_TDME_SET_CONFIRM               (TDME_SET_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_TDME_TXPKT_CONFIRM             (TDME_TXPKT_CONFIRM+SPI_S2M+SPI_SYN)
#define SPI_TDME_RXPKT_INDICATION          (TDME_RXPKT_INDICATION+SPI_S2M)
#define SPI_TDME_EDDET_INDICATION          (TDME_EDDET_INDICATION+SPI_S2M)
#define SPI_TDME_ERROR_INDICATION          (TDME_ERROR_INDICATION+SPI_S2M)
#define SPI_TDME_LOTLK_CONFIRM             (TDME_LOTLK_CONFIRM+SPI_S2M+SPI_SYN)

union MacAddr;
struct SecSpec;

/******************************************************************************/
/* Prototypes */

static int ca8210_probe(struct spi_device *spi_device);
static int ca8210_remove(struct spi_device *spi_device);

static void ca8210_reset_send(struct spi_device *spi, int ms);
static int ca8210_spi_exchange(
	const uint8_t *buf,
	size_t len,
	uint8_t *response,
	void *pDeviceRef
);
static irqreturn_t ca8210_interrupt_handler(int irq, void *dev_id);
static void ca8210_irq_worker(struct work_struct *work);
static int ca8210_spi_write(struct spi_device *spi, const uint8_t *buf, size_t len);
static void ca8210_spi_startRead(struct spi_device *spi);
static void ca8210_spi_continueRead(void *arg);
static void ca8210_spi_finishRead(void *arg);
static void ca8210_rx_done(struct work_struct *work);

static int ca8210_net_rx(struct ieee802154_hw *hw, uint8_t *msg, size_t len);
static int ca8210_skb_rx(
	struct ieee802154_hw *hw,
	size_t len,
	uint8_t *data_ind
);

static int ca8210_start(struct ieee802154_hw *hw);
static void ca8210_stop(struct ieee802154_hw *hw);
static int ca8210_xmit_sync(struct ieee802154_hw *hw, struct sk_buff *skb);
static int ca8210_xmit_async(struct ieee802154_hw *hw, struct sk_buff *skb);
static int ca8210_async_xmit_complete(
	struct ieee802154_hw *hw,
	uint8_t msduhandle,
	uint8_t status
);
static int ca8210_get_ed(struct ieee802154_hw *hw, uint8_t *level);
static int ca8210_set_channel(
	struct ieee802154_hw *hw,
	uint8_t page,
	uint8_t channel
);
static int ca8210_set_hw_addr_filt(struct ieee802154_hw *hw,
                             struct ieee802154_hw_addr_filt *filt,
                             unsigned long changed);
static int ca8210_set_tx_power(struct ieee802154_hw *hw, s32 dbm);
static int ca8210_set_lbt(struct ieee802154_hw *hw, bool on);
static int ca8210_set_cca_mode(
	struct ieee802154_hw *hw,
	const struct wpan_phy_cca *cca
);
static int ca8210_set_cca_ed_level(struct ieee802154_hw *hw, int32_t level);
static int ca8210_set_csma_params(
	struct ieee802154_hw *hw,
	uint8_t min_be,
	uint8_t max_be,
	uint8_t retries
);
static int ca8210_set_frame_retries(struct ieee802154_hw *hw, s8 retries);

static int ca8210_test_int_open(struct inode *inodp, struct file *filp);
static ssize_t ca8210_test_int_user_write(
	struct file *filp,
	const char *in_buf,
	size_t len,
	loff_t *off
);
static ssize_t ca8210_test_int_user_read(
	struct file *filp,
	char __user *buf,
	size_t len,
	loff_t *offp
);
static int ca8210_test_int_driver_write(const uint8_t *buf, size_t len, void *spi);

static uint8_t MCPS_DATA_request(
	uint8_t          SrcAddrMode,
	uint8_t          DstAddrMode,
	uint16_t         DstPANId,
	union MacAddr  *pDstAddr,
	uint8_t          MsduLength,
	uint8_t         *pMsdu,
	uint8_t          MsduHandle,
	uint8_t          TxOptions,
	struct SecSpec  *pSecurity,
	void            *pDeviceRef
);
static uint8_t MLME_RESET_request_sync(uint8_t SetDefaultPIB, void *pDeviceRef);
static uint8_t MLME_SET_request_sync(
	uint8_t       PIBAttribute,
	uint8_t       PIBAttributeIndex,
	uint8_t       PIBAttributeLength,
	const void   *pPIBAttributeValue,
	void         *pDeviceRef
);
static uint8_t HWME_SET_request_sync(
	uint8_t      HWAttribute,
	uint8_t      HWAttributeLength,
	uint8_t     *pHWAttributeValue,
	void        *pDeviceRef
);
static uint8_t HWME_GET_request_sync(
    uint8_t      HWAttribute,
    uint8_t     *HWAttributeLength,
    uint8_t     *pHWAttributeValue,
    void        *pDeviceRef
);
static uint8_t TDME_SETSFR_request_sync(
    uint8_t      SFRPage,
    uint8_t      SFRAddress,
    uint8_t      SFRValue,
    void         *pDeviceRef
);
static uint8_t TDME_ChipInit(void *pDeviceRef);
static uint8_t TDME_ChannelInit(uint8_t channel, void *pDeviceRef);
static uint8_t TDME_CheckPIBAttribute(
    uint8_t      PIBAttribute,
    uint8_t      PIBAttributeLength,
    const void   *pPIBAttributeValue
);
static uint8_t TDME_SetTxPower(uint8_t txp, void *pDeviceRef);

/******************************************************************************/
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
 * @spi_sem:              semaphore protecting spi interface
 * @data_received_so_far  number of bytes received
 * @data_left_to_receive  number of bytes still to receive
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

	struct semaphore spi_sem;

	int data_received_so_far;
	int data_left_to_receive;
};

/**
 * struct ca8210_test - ca8210 test interface structure
 * @char_dev_num:   device id number
 * @char_dev_cdev:  device object for this char driver
 * @cl:             class of this device
 * @de:
 * @up_fifo:        fifo for upstream messages
 *
 * This structure stores all the data pertaining to the test file interface and
 * char driver.
 */
struct ca8210_test {
	dev_t char_dev_num;
	struct cdev char_dev_cdev;
	struct class* cl;
	struct device* de;
	struct kfifo up_fifo;
};

/**
 * struct ca8210_priv - ca8210 private data structure
 * @spi:                    pointer to the ca8210 spi device object
 * @hw:                     pointer to the ca8210 ieee802154_hw object
 * @lock:                   spinlock protecting the private data area
 * @async_tx_workqueue:     workqueue for asynchronous transmission
 * @rx_workqueue:           workqueue for receive processing
 * @async_tx_work:          work object for a single asynchronous transmission
 * @rx_work:                work object for processing a single received packet
 * @irq_work:               work object for a single irq
 * @async_tx_timeout_work:  delayed work object for a single asynchronous
 *                          transmission timeout
 * @tx_skb:                 current socket buffer to transmit
 * @nextmsduhandle:         msdu handle to pass to the 15.4 MAC layer for the
 *                          next transmission
 * @clk:                    external clock provided by the ca8210
 * @clear_next_spi_rx:      whether the next received spi packet should be
 *                          cleared from the higher layer buffer, used for
 *                          packets received at inopportune times (duplex mode
 *                          during a synchronous command)
 * @cas_ctl:                spi control data section for this instance
 * @lastDSN:                sequence number of last data packet received, for
 *                          resend detection
 * @test:                   test interface data section for this instance
 * @async_tx_pending:       true if an asynchronous transmission was started and
 *                          is not complete
 * @sync_tx_pending:        true if a synchronous transmission was started and
 *                          is not complete
 *
 */
struct ca8210_priv {
	struct spi_device *spi;
	struct ieee802154_hw *hw;
	spinlock_t lock;
	struct workqueue_struct *async_tx_workqueue, *rx_workqueue;
	struct work_struct async_tx_work, rx_work, irq_work;
	struct delayed_work async_tx_timeout_work;
	struct sk_buff *tx_skb;
	uint8_t nextmsduhandle;
	struct clk *clk;
	bool clear_next_spi_rx;
	struct cas_control cas_ctl;
	int lastDSN;
	struct ca8210_test test;
	bool async_tx_pending, sync_tx_pending;
	bool sync_command_pending;
	struct mutex sync_command_mutex;
	uint8_t *sync_command_response;
};

/**
 * struct ca8210_platform_data - ca8210 platform data structure
 * @extclockfreq:  frequency of the external clock
 * @extclockgpio:  ca8210 output gpio of the external clock
 * @registerclk:   true if the external clock should be registered
 * @gpio_reset:    gpio number of ca8210 reset line
 * @gpio_irq:      gpio number of ca8210 interrupt line
 * @irq_id:        identifier for the ca8210 irq
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

struct FullAddr {
	uint8_t         AddressMode;
	uint8_t         PANId[2];
	uint8_t         Address[8];
};

union MacAddr {
	uint16_t        ShortAddress;
	uint8_t         IEEEAddress[8];
};

struct SecSpec {
	uint8_t         SecurityLevel;
	uint8_t         KeyIdMode;
	uint8_t         KeySource[8];
	uint8_t         KeyIndex;
};

struct PanDescriptor {
	struct FullAddr Coord;
	uint8_t         LogicalChannel;
	uint8_t         SuperframeSpec[2];
	uint8_t         GTSPermit;
	uint8_t         LinkQuality;
	uint8_t         TimeStamp[4];
	uint8_t         SecurityFailure;
	struct SecSpec  Security;
};

/* downlink functions parameter set definitions */
struct MCPS_DATA_request_pset {
	uint8_t         SrcAddrMode;
	struct FullAddr Dst;
	uint8_t         MsduLength;
	uint8_t         MsduHandle;
	uint8_t         TxOptions;
	uint8_t         Msdu[MAX_DATA_SIZE];
};

struct MLME_ASSOCIATE_request_pset {
	uint8_t         LogicalChannel;
	struct FullAddr Dst;
	uint8_t         CapabilityInfo;
	struct SecSpec  Security;
};

struct MLME_ASSOCIATE_response_pset {
	uint8_t         DeviceAddress[8];
	uint8_t         AssocShortAddress[2];
	uint8_t         Status;
	struct SecSpec  Security;
};

struct MLME_DISASSOCIATE_request_pset {
	struct FullAddr DevAddr;
	uint8_t         DisassociateReason;
	uint8_t         TxIndirect;
 	struct SecSpec  Security;
};

struct MLME_GET_request_pset {
	uint8_t         PIBAttribute;
	uint8_t         PIBAttributeIndex;
};

struct MLME_ORPHAN_response_pset {
	uint8_t         OrphanAddress[8];
	uint8_t         ShortAddress[2];
	uint8_t         AssociatedMember;
	struct SecSpec  Security;
};

struct MLME_POLL_request_pset {
	struct FullAddr CoordAddress;
	uint8_t         Interval[2];      /* polling interval in 0.1 seconds res */
	                                  /* 0 means poll once */
	                                  /* 0xFFFF means stop polling */
	struct SecSpec  Security;
};

struct MLME_RX_ENABLE_request_pset {
	uint8_t         DeferPermit;
	uint8_t         RxOnTime[4];
	uint8_t         RxOnDuration[4];
};

struct MLME_SCAN_request_pset {
	uint8_t         ScanType;
	uint8_t         ScanChannels[4];
	uint8_t         ScanDuration;
	struct SecSpec  Security;
};

struct MLME_SET_request_pset {
	uint8_t         PIBAttribute;
	uint8_t         PIBAttributeIndex;
	uint8_t         PIBAttributeLength;
	uint8_t         PIBAttributeValue[MAX_ATTRIBUTE_SIZE];
};

struct MLME_START_request_pset {
	uint8_t         PANId[2];
	uint8_t         LogicalChannel;
	uint8_t         BeaconOrder;
	uint8_t         SuperframeOrder;
	uint8_t         PANCoordinator;
	uint8_t         BatteryLifeExtension;
	uint8_t         CoordRealignment;
	struct SecSpec  CoordRealignSecurity;
	struct SecSpec  BeaconSecurity;
};

struct HWME_SET_request_pset {
	uint8_t         HWAttribute;
	uint8_t         HWAttributeLength;
	uint8_t         HWAttributeValue[MAX_HWME_ATTRIBUTE_SIZE];
};

struct HWME_GET_request_pset {
	uint8_t         HWAttribute;
};

struct TDME_SETSFR_request_pset {
	uint8_t         SFRPage;
	uint8_t         SFRAddress;
	uint8_t         SFRValue;
};

struct TDME_GETSFR_request_pset {
	uint8_t         SFRPage;
	uint8_t         SFRAddress;
};

struct TDME_SET_request_pset {
	uint8_t         TDAttribute;
	uint8_t         TDAttributeLength;
	uint8_t         TDAttributeValue[MAX_TDME_ATTRIBUTE_SIZE];
};

/* uplink functions parameter set definitions */
struct HWME_SET_confirm_pset {
	uint8_t         Status;
	uint8_t         HWAttribute;
};

struct HWME_GET_confirm_pset {
	uint8_t         Status;
	uint8_t         HWAttribute;
	uint8_t         HWAttributeLength;
	uint8_t         HWAttributeValue[MAX_HWME_ATTRIBUTE_SIZE];
};

struct TDME_SETSFR_confirm_pset {
	uint8_t         Status;
	uint8_t         SFRPage;
	uint8_t         SFRAddress;
};

struct MAC_Message {
	uint8_t      CommandId;
	uint8_t      Length;
	union {
		struct MCPS_DATA_request_pset                      DataReq;
		struct MLME_ASSOCIATE_request_pset                 AssocReq;
		struct MLME_ASSOCIATE_response_pset                AssocRsp;
		struct MLME_DISASSOCIATE_request_pset              DisassocReq;
		struct MLME_GET_request_pset                       GetReq;
		struct MLME_ORPHAN_response_pset                   OrphanRsp;
		struct MLME_POLL_request_pset                      PollReq;
		struct MLME_RX_ENABLE_request_pset                 RxEnableReq;
		struct MLME_SCAN_request_pset                      ScanReq;
		struct MLME_SET_request_pset                       SetReq;
		struct MLME_START_request_pset                     StartReq;
		struct HWME_SET_request_pset                       HWMESetReq;
		struct HWME_GET_request_pset                       HWMEGetReq;
		struct TDME_SETSFR_request_pset                    TDMESetSFRReq;
		struct TDME_GETSFR_request_pset                    TDMEGetSFRReq;
		struct TDME_SET_request_pset                       TDMESetReq;
		struct HWME_SET_confirm_pset                       HWMESetCnf;
		struct HWME_GET_confirm_pset                       HWMEGetCnf;
		struct TDME_SETSFR_confirm_pset                    TDMESetSFRCnf;
		uint8_t                                            u8Param;
		uint8_t                                            Status;
		uint8_t                                            Payload[254];
	} PData;
};

/******************************************************************************/
/* Globals */

static const struct of_device_id ca8210_of_ids[] = {
	{.compatible = "cascoda,ca8210", },
	{},
};
MODULE_DEVICE_TABLE(of, ca8210_of_ids);

static struct spi_driver ca8210_spi_driver = {
	.driver = {
		.name =			DRIVER_NAME,
		.owner = 		THIS_MODULE,
		.of_match_table =       of_match_ptr(ca8210_of_ids),
	},
	.probe  = 			ca8210_probe,
	.remove = 			ca8210_remove
};

static const struct ieee802154_ops ca8210_phy_ops = {
	.start = ca8210_start,
	.stop = ca8210_stop,
	.xmit_sync = ca8210_xmit_sync,
	.xmit_async = ca8210_xmit_async,
	.ed = ca8210_get_ed,
	.set_channel = ca8210_set_channel,
	.set_hw_addr_filt = ca8210_set_hw_addr_filt,
	.set_txpower = ca8210_set_tx_power,
	.set_lbt = ca8210_set_lbt,
	.set_cca_mode = ca8210_set_cca_mode,
	.set_cca_ed_level = ca8210_set_cca_ed_level,
	.set_csma_params = ca8210_set_csma_params,
	.set_frame_retries = ca8210_set_frame_retries
};

static const struct file_operations test_int_fops = {
	.read = 			ca8210_test_int_user_read,
	.write = 			ca8210_test_int_user_write,
	.open = 			ca8210_test_int_open,
	.release = 			NULL
};

static int (*cascoda_api_downstream)(
	const uint8_t *buf,
	size_t len,
	uint8_t *response,
	void *pDeviceRef
) = ca8210_spi_exchange;

static int (*cascoda_api_upstream)(
	const uint8_t *buf,
	size_t len,
	void *pDeviceRef
) = ca8210_test_int_driver_write;

/******************************************************************************/
/* Utility */

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
	switch(link_status){
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

/******************************************************************************/
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
		pr_info("[ca8210] Switching external clock on\n");
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
			pr_crit("[ca8210] Invalid extclock-freq\n");
			return -EINVAL;
		}
		clkparam[1] = pdata->extclockgpio;
	} else {
		pr_info("[ca8210] Switching external clock off\n");
		clkparam[0] = 0; /* off */
		clkparam[1] = 0;
	}
	return link_to_linux_err(
		HWME_SET_request_sync(HWME_SYSCLKOUT, 2, clkparam, spi)
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
	                                    CLK_IS_ROOT,
	                                    pdata->extclockfreq);

	if (IS_ERR(priv->clk)) {
		pr_crit("[ca8210] Failed to register external clk\n");
		return PTR_ERR(priv->clk);
	}
	ret = of_clk_add_provider(np, of_clk_src_simple_get, priv->clk);
	if (ret) {
		clk_unregister(priv->clk);
		pr_crit("[ca8210] Failed to register external clock as clock provider\n");
	} else {
		pr_info("[ca8210] External clock set as clock provider\n");
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
	pr_info("[ca8210] External clock unregistered\n");
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
		pr_crit("[ca8210] Reset GPIO %d did not set to output mode\n",
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
	if (pdata->irq_id < 0){
		pr_crit("[ca8210] Could not get irq for gpio pin %d\n",
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
		pr_crit("[ca8210] request_irq %d failed\n", pdata->irq_id);
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

	sema_init (&priv->cas_ctl.spi_sem, 1);

	priv->cas_ctl.tx_buf = kmalloc(SPI_BUF_SIZE, GFP_DMA | GFP_KERNEL);
	if (priv->cas_ctl.tx_buf == NULL) {
		status = -EFAULT;
		goto error;
	}
	memset(priv->cas_ctl.tx_buf, '\xff', SPI_BUF_SIZE);

	priv->cas_ctl.tx_in_buf = kmalloc(SPI_BUF_SIZE, GFP_DMA | GFP_KERNEL);
	if (priv->cas_ctl.tx_in_buf == NULL) {
		status = -EFAULT;
		goto error;
	}
	memset(priv->cas_ctl.tx_in_buf, '\xff', SPI_BUF_SIZE);

	priv->cas_ctl.rx_buf = kmalloc(SPI_BUF_SIZE, GFP_DMA | GFP_KERNEL);
	if (priv->cas_ctl.rx_buf == NULL) {
		status = -EFAULT;
		goto error;
	}
	memset(priv->cas_ctl.rx_buf, '\xff', SPI_BUF_SIZE);

	priv->cas_ctl.rx_out_buf = kmalloc (SPI_BUF_SIZE, GFP_DMA | GFP_KERNEL);
	if (priv->cas_ctl.rx_out_buf == NULL) {
		status = -EFAULT;
		goto error;
	}
	memset(priv->cas_ctl.rx_out_buf, '\xff', SPI_BUF_SIZE);

	priv->cas_ctl.rx_final_buf = kmalloc (SPI_BUF_SIZE, GFP_DMA | GFP_KERNEL);
	if (priv->cas_ctl.rx_final_buf == NULL) {
		status = -EFAULT;
		goto error;
	}
	memset(priv->cas_ctl.rx_final_buf, '\xff', SPI_BUF_SIZE);

	priv->cas_ctl.tx_transfer.tx_nbits = 1; /* 1 MOSI line */
	priv->cas_ctl.tx_transfer.rx_nbits = 1; /* 1 MISO line */
	priv->cas_ctl.tx_transfer.speed_hz = 0; /* Use device setting */
	priv->cas_ctl.tx_transfer.bits_per_word = 0; /* Use device setting */
	priv->cas_ctl.rx_transfer.tx_nbits = 1; /* 1 MOSI line */
	priv->cas_ctl.rx_transfer.rx_nbits = 1; /* 1 MISO line */
	priv->cas_ctl.rx_transfer.speed_hz = 0; /* Use device setting */
	priv->cas_ctl.rx_transfer.bits_per_word = 0; /* Use device setting */

	priv->rx_workqueue = alloc_ordered_workqueue("ca8210 rx worker", 0);
	if (priv->rx_workqueue == NULL) {
		pr_crit("[ca8210] alloc of rx_workqueue failed!\n");
	}
	INIT_WORK(&priv->rx_work, ca8210_rx_done);
	INIT_WORK(&priv->irq_work, ca8210_irq_worker);

	return 0;

	error:
	if (priv->cas_ctl.rx_buf) {
		kfree(priv->cas_ctl.rx_buf);
		priv->cas_ctl.rx_buf = NULL;
	}

	if (priv->cas_ctl.tx_in_buf) {
		kfree(priv->cas_ctl.tx_in_buf);
		priv->cas_ctl.tx_in_buf = NULL;
	}

	if (priv->cas_ctl.tx_buf) {
		kfree(priv->cas_ctl.tx_buf);
		priv->cas_ctl.tx_buf = NULL;
	}

	if (priv->cas_ctl.rx_out_buf) {
		kfree(priv->cas_ctl.rx_out_buf);
		priv->cas_ctl.rx_out_buf = NULL;
	}

	if (priv->cas_ctl.rx_final_buf) {
		kfree(priv->cas_ctl.rx_final_buf);
		priv->cas_ctl.rx_final_buf = NULL;
	}

	return status;
}

/**
 * ca8210_dev_com_clear() - Deinitialise the spi communication component
 * @priv:  Pointer to private data structure
 */
static void ca8210_dev_com_clear(struct ca8210_priv *priv)
{
	if (priv->cas_ctl.rx_buf) {
		kfree(priv->cas_ctl.rx_buf);
		priv->cas_ctl.rx_buf = NULL;
	}

	if (priv->cas_ctl.tx_in_buf) {
		kfree(priv->cas_ctl.tx_in_buf);
		priv->cas_ctl.tx_in_buf = NULL;
	}

	if (priv->cas_ctl.tx_buf) {
		kfree(priv->cas_ctl.tx_buf);
		priv->cas_ctl.tx_buf = NULL;
	}

	if (priv->cas_ctl.rx_out_buf) {
		kfree(priv->cas_ctl.rx_out_buf);
		priv->cas_ctl.rx_out_buf = NULL;
	}

	if (priv->cas_ctl.rx_final_buf) {
		kfree(priv->cas_ctl.rx_final_buf);
		priv->cas_ctl.rx_final_buf = NULL;
	}

	flush_workqueue(priv->rx_workqueue);
	destroy_workqueue(priv->rx_workqueue);
}

/**
 * ca8210_hw_setup() - Populate the ieee802154_hw phy attributes with the
 *                     ca8210's defaults
 * @ca8210_hw:  Pointer to ieee802154_hw to populate
 */
static void ca8210_hw_setup(struct ieee802154_hw *ca8210_hw)
{
	/* Support channels 11-26 */
	ca8210_hw->phy->supported.channels[0] = M_ValidChannels;
	ca8210_hw->phy->current_channel = 18;
	ca8210_hw->phy->current_page = 0;
	ca8210_hw->phy->transmit_power = 8;
	ca8210_hw->phy->cca.mode = NL802154_CCA_ENERGY_CARRIER;
	ca8210_hw->phy->cca.opt = NL802154_CCA_OPT_ENERGY_CARRIER_AND;
	ca8210_hw->phy->cca_ed_level = 0x3C;
	ca8210_hw->phy->symbol_duration = 16;
	ca8210_hw->phy->lifs_period = 40;
	ca8210_hw->phy->sifs_period = 12;
	ca8210_hw->flags = /*IEEE802154_HW_LBT |*/ /*TODO: Remove this?*/
	                   IEEE802154_HW_AFILT |
	                   IEEE802154_HW_OMIT_CKSUM |
			   IEEE802154_HW_FRAME_RETRIES |
	                   IEEE802154_HW_CSMA_PARAMS;
	ca8210_hw->phy->flags = WPAN_PHY_FLAG_TXPOWER |
				WPAN_PHY_FLAG_CCA_ED_LEVEL |
				WPAN_PHY_FLAG_CCA_MODE;
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
	int status;
	struct ca8210_test *test = &priv->test;

	status = alloc_chrdev_region(&test->char_dev_num, 0, 1, TEST_INT_FILE_NAME);
	if(status < 0){
		pr_crit("[ca8210] test_interface: Could not allocate a major number\n");
		return status;
	}

	cdev_init(&test->char_dev_cdev, &test_int_fops);
	test->char_dev_cdev.owner = THIS_MODULE;

	status = cdev_add(&test->char_dev_cdev, test->char_dev_num, 1);
	if(status < 0){
		unregister_chrdev_region(test->char_dev_num, 1);
		pr_crit("[ca8210] test_interface: Unable to register char dev\n");
		return status;
	}

	test->cl = class_create(THIS_MODULE, TEST_INT_FILE_NAME);
	if(IS_ERR(test->cl)){
		cdev_del(&test->char_dev_cdev);
		unregister_chrdev_region(test->char_dev_num, 1);

		pr_crit("[ca8210] test_interface: Could not create device class\n");
		return PTR_ERR(test->cl);
	}

	test->de = device_create(test->cl, NULL, test->char_dev_num, NULL, TEST_INT_FILE_NAME);
	if(IS_ERR(test->de)){
		cdev_del(&test->char_dev_cdev);
		unregister_chrdev_region(test->char_dev_num, 1);
		class_destroy(test->cl);

		pr_crit("[ca8210] test_interface: Could not create device\n");
		return PTR_ERR(test->de);
	}

	return kfifo_alloc(&test->up_fifo, TEST_INT_FIFO_SIZE, GFP_KERNEL);
}

/**
 * ca8210_test_interface_clear() - Deinitialise the test file interface
 * @priv:  Pointer to private data structure
 */
static void ca8210_test_interface_clear(struct ca8210_priv *priv)
{
	struct ca8210_test *test = &priv->test;
	cdev_del(&test->char_dev_cdev);
	unregister_chrdev_region(test->char_dev_num, 1);
	device_destroy(test->cl, test->char_dev_num);
	class_destroy(test->cl);
	kfifo_free(&test->up_fifo);
	pr_info("[ca8210] Test interface removed\n");
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

	pr_info("[ca8210] Inserting SPI protocol driver\n");

	/* allocate ieee802154_hw and private data */
	hw = ieee802154_alloc_hw(sizeof(struct ca8210_priv), &ca8210_phy_ops);
	if (hw == NULL) {
		pr_crit("[ca8210] ieee802154_alloc_hw failed\n");
		ret = -ENOMEM;
		goto error;
	}

	priv = hw->priv;
	priv->hw = hw;
	priv->spi = spi_device;
	hw->parent = &spi_device->dev;
	spin_lock_init(&priv->lock);
	priv->clear_next_spi_rx = false;
	priv->async_tx_pending = false;
	priv->sync_command_pending = false;
	mutex_init(&priv->sync_command_mutex);
	spi_set_drvdata(priv->spi, priv);

	ca8210_test_interface_init(priv);
	ca8210_hw_setup(hw);
	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

	ret = ieee802154_register_hw(hw);
	if (ret) {
		pr_crit("[ca8210] ieee802154_register_hw failed\n");
		goto error;
	}

	pdata = kmalloc(sizeof(struct ca8210_platform_data), GFP_KERNEL);
	if (pdata == NULL) {
		pr_crit("[ca8210] Could not allocate platform data\n");
		ret = -ENOMEM;
		goto error;
	}

	ret = ca8210_get_platform_data(priv->spi, pdata);
	if (ret) {
		pr_crit("[ca8210] ca8210_get_platform_data failed\n");
		goto error;
	}
	priv->spi->dev.platform_data = pdata;

	ret = ca8210_dev_com_init(priv);
	if (ret) {
		pr_crit("[ca8210] ca8210_dev_com_init failed\n");
		goto error;
	}
	ret = ca8210_reset_init(priv->spi);
	if (ret) {
		pr_crit("[ca8210] ca8210_reset_init failed\n");
		goto error;
	}

	ca8210_reset_send(priv->spi, 50);

	ret = ca8210_interrupt_init(priv->spi);
	if (ret) {
		pr_crit("[ca8210] ca8210_interrupt_init failed\n");
		goto error;
	}

	mdelay(1000);	/* Time to process wakeup indication */
	ret = TDME_ChipInit(priv->spi);
	if (ret) {
		pr_crit("[ca8210] TDME_ChipInit failed\n");
		goto error;
	}

	if (pdata->extclockenable) {
		ret = ca8210_config_extern_clk(pdata, priv->spi, 1);
		if (ret) {
			pr_crit("[ca8210] ca8210_config_extern_clk failed\n");
			goto error;
		}
		ret = ca8210_register_ext_clock(priv->spi);
		if (ret) {
			pr_crit("[ca8210] ca8210_register_ext_clock failed\n");
			goto error;
		}
	}

	return 0;
error:
	ca8210_remove(spi_device);
	return link_to_linux_err(ret);
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

	pr_info("[ca8210] Removing SPI protocol driver\n");

	pdata = spi_device->dev.platform_data;
	if (pdata) {
		if (pdata->extclockenable) {
			ca8210_unregister_ext_clock(spi_device);
			ca8210_config_extern_clk(pdata, spi_device, 0);
		}
		kfree(pdata);
		free_irq(pdata->irq_id, spi_device->dev.driver_data);
		spi_device->dev.platform_data = NULL;
	}
	/* get spi_device private data */
	priv = spi_get_drvdata(spi_device);
	if(priv){
		ca8210_dev_com_clear(spi_device->dev.driver_data);
		if(priv->hw){
			ieee802154_unregister_hw(priv->hw);
			ieee802154_free_hw(priv->hw);
			priv->hw = NULL;
			pr_info("[ca8210] Unregistered & freed ieee802154_hw.\n");
		}
		ca8210_test_interface_clear(priv);
	}

	return 0;
}

/******************************************************************************/
/* SPI Operation */

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

	pr_debug("[ca8210] irq: Interrupt occured\n");

	queue_work(priv->rx_workqueue, &priv->irq_work);

	return IRQ_HANDLED;
}

/**
 * ca8210_irq_worker() - Starts the spi read process after having the work
 *                       handed off by the interrupt handler
 * @work:  Pointer to work being executed
 */
static void ca8210_irq_worker(struct work_struct *work)
{
	struct ca8210_priv *priv = container_of(work, struct ca8210_priv, irq_work);

	ca8210_spi_startRead(priv->spi);
}

/**
 * ca8210_reset_send() - Hard resets the ca8210 for a given time
 * @spi:  Pointer to target ca8210 spi device
 * @ms:   Milliseconds to hold the reset line low for
 */
static void ca8210_reset_send(struct spi_device *spi, int ms)
{
	struct ca8210_platform_data *pdata = spi->dev.platform_data;
	#define RESET_OFF 0
	#define RESET_ON 1
	gpio_set_value(pdata->gpio_reset, RESET_OFF);
	msleep(ms);
	gpio_set_value(pdata->gpio_reset, RESET_ON);
	pr_debug("[ca8210] Reset the device\n");
	#undef RESET_OFF
	#undef RESET_ON
}

static int ca8210_spi_exchange(
	const uint8_t *buf,
	size_t len,
	uint8_t *response,
	void *pDeviceRef
)
{
	int status;
	unsigned long startjiffies, currentjiffies;
	struct spi_device *spi = pDeviceRef;
	struct ca8210_priv *priv = spi->dev.driver_data;

	do {
		status = ca8210_spi_write(priv->spi, buf, len);
	} while (status == -EBUSY);

	if (status < 0) {
		return status;
	}

	if ((buf[0] & SPI_SYN) && response) { /* if synchronous wait for confirm */
		if (mutex_lock_interruptible(&priv->sync_command_mutex)) {
			return -ERESTARTSYS;
		}
		priv->sync_command_response = response;
		priv->sync_command_pending = true;
		mutex_unlock(&priv->sync_command_mutex);
		startjiffies = jiffies;
		while (1) {
			if (mutex_lock_interruptible(&priv->sync_command_mutex)) {
				return -ERESTARTSYS;
			}
			if (!priv->sync_command_pending) {
				priv->sync_command_response = NULL;
				mutex_unlock(&priv->sync_command_mutex);
				break;
			}
			mutex_unlock(&priv->sync_command_mutex);
			currentjiffies = jiffies;
			if ((currentjiffies - startjiffies) > msecs_to_jiffies(SPI_T_TIMEOUT)) {
				return -ETIME;
			}
		}
	}

	return 0;
}

/**
 * ca8210_spi_writeDummy() - Write a "dummy" packet to the ca8210
 * @spi:  Pointer to spi device to write to
 *
 * This functions exists solely to toggle the spi chip select to the ca8210. The
 * ca8210 sends and receives variable length spi packets to its host processor.
 * This means the chip select must be held asserted inbetween processing
 * individual bytes of messages. Using the current spi framework the only way to
 * toggle the chip select is through an spi transfer so this functions writes
 * an "IDLE" 0xFF byte to the ca8210 for the sole purpose of de-asserting the
 * chip select when the write has finished.
 *
 * Return: 0 or linux error code
 */
static int ca8210_spi_writeDummy(struct spi_device *spi)
{
	int ret;
	uint8_t idle = 0xFF;

	pr_debug("[ca8210] spi: writing dummy packet\n");
	ret =  ca8210_spi_write(spi, &idle, 1);
	pr_debug("[ca8210] spi: wrote dummy packet\n");
	return ret;
}

/**
 * ca8210_spi_write() - Write a message to the ca8210 over spi
 * @spi:     Pointer to spi device to write to
 * @buf:     Octet array to send
 * @length:  Length of the buffer being sent
 *
 * Return: 0 or linux error code
 */
static int ca8210_spi_write(struct spi_device *spi, const uint8_t *buf, size_t len)
{
	int status = 0;
	int i;
	bool dummy;
	struct ca8210_priv *priv = spi_get_drvdata(spi);
	unsigned long flags;
	unsigned cpu = smp_processor_id();

	if (spi == NULL) {
		pr_crit("[ca8210] NULL spi device passed to ca8210_spi_write\n");
		return -ENODEV;
	}

	if (buf[0] == 0xFF && len == 1) {
		dummy = true;
	} else {
		dummy = false;
		/* Hold off on servicing interrupts, will get read during write anyway */
		local_irq_save(flags);
		if (down_interruptible(&priv->cas_ctl.spi_sem)) {
			return -ERESTARTSYS;
		}
	}

	pr_debug("[ca8210] SPI write function -ca8210_spi_write- called\n");

	/* Set in/out buffers to idle, copy over data to send */
	memset(priv->cas_ctl.tx_buf, 0xFF, SPI_BUF_SIZE);
	memset(priv->cas_ctl.tx_in_buf, 0xFF, SPI_BUF_SIZE);
	memcpy(priv->cas_ctl.tx_buf, buf, len);

	pr_debug("[ca8210] device_comm: Command ID = %#03x Length = %#03x	Data:\n",
		priv->cas_ctl.tx_buf[0], priv->cas_ctl.tx_buf[1]);

	for (i = 2; i < len; i++) {
		pr_debug("%#03x\n", priv->cas_ctl.tx_buf[i]);
	}

	#define SPLIT_TX_LEN 64
	for (i = 0; i < len; i += SPLIT_TX_LEN) {
		spi_message_init(&priv->cas_ctl.tx_msg);
		priv->cas_ctl.tx_msg.spi = spi;
		priv->cas_ctl.tx_msg.is_dma_mapped = false;

		priv->cas_ctl.tx_transfer.tx_buf = priv->cas_ctl.tx_buf + i;
		priv->cas_ctl.tx_transfer.rx_buf = priv->cas_ctl.tx_in_buf + i;
		priv->cas_ctl.tx_transfer.delay_usecs = 200;
		if (len-i < SPLIT_TX_LEN) {
			priv->cas_ctl.tx_transfer.len = len - i;
			priv->cas_ctl.tx_msg.frame_length = len - i;
		} else {
			priv->cas_ctl.tx_transfer.len = SPLIT_TX_LEN;
			priv->cas_ctl.tx_msg.frame_length = SPLIT_TX_LEN;
		}

		if (!dummy) {
			/* Regular transmission, keep CS asserted in case of incomplete concurrent
			   read */
			priv->cas_ctl.tx_transfer.cs_change = 1;
		} else {
			/* dummy transmission, de-assert CS */
			priv->cas_ctl.tx_transfer.cs_change = 0;
		}

		spi_message_add_tail(&priv->cas_ctl.tx_transfer, &priv->cas_ctl.tx_msg);

		status = spi_sync(spi, &priv->cas_ctl.tx_msg);
		if(status < 0) {
			pr_crit("[ca8210] Status %d from spi_sync in write\n", status);
		} else if (!dummy && priv->cas_ctl.tx_in_buf[0] == '\xF0') {
			/* ca8210 is busy */
			ca8210_spi_writeDummy(spi);
			up(&priv->cas_ctl.spi_sem);
			local_irq_restore(flags);
			return -EBUSY;
		}
	}
	#undef SPLIT_TX_LEN

	if (dummy)
		return status;

	pr_debug("[ca8210] spi received during transfer:\n");
	for (i = 0; i < len; i++) {
		pr_debug("%#03x\n", priv->cas_ctl.tx_in_buf[i]);
	}

	if (priv->cas_ctl.tx_in_buf[0] != 0xFF /* Not idle */ &&
	    priv->cas_ctl.tx_in_buf[0] != 0xF0 /* Not NACK */) {
		/* Received start of rx packet during transfer */
		pr_debug("[ca8210] Trying to get spinlock on CPU%d\n", cpu);
		spin_lock(&priv->lock);
		pr_debug("[ca8210] Got spinlock on CPU%d\n", cpu);

		priv->clear_next_spi_rx = true;

		pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
		spin_unlock(&priv->lock);
		pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);

		#define NUM_DATABYTES_SO_FAR (len-2)
		if (priv->cas_ctl.tx_in_buf[1] > NUM_DATABYTES_SO_FAR) {
			/* Need to read rest of data of packet */
			/* Buffer what we have so far and set up the rest of the transfer */
			pr_debug("[ca8210] READ CMDID & LEN DURING TX, NEED TO READ REST OF DATA\n");
			memcpy(priv->cas_ctl.rx_final_buf, &priv->cas_ctl.tx_in_buf[0], 2);
			memcpy(priv->cas_ctl.rx_buf, &priv->cas_ctl.tx_in_buf[2], NUM_DATABYTES_SO_FAR);
			priv->cas_ctl.rx_transfer.len = priv->cas_ctl.tx_in_buf[1] - NUM_DATABYTES_SO_FAR;
			priv->cas_ctl.rx_transfer.rx_buf = priv->cas_ctl.rx_buf + NUM_DATABYTES_SO_FAR;
			priv->cas_ctl.rx_transfer.tx_buf = priv->cas_ctl.rx_out_buf;
			priv->cas_ctl.rx_transfer.cs_change = 0; /* de-assert CS */
			spi_message_init(&priv->cas_ctl.rx_msg);
			priv->cas_ctl.rx_msg.complete = ca8210_spi_finishRead;
			/* Pass the spi reference to the complete */
			priv->cas_ctl.rx_msg.context = spi;
			spi_message_add_tail(&priv->cas_ctl.rx_transfer, &priv->cas_ctl.rx_msg);
			status = spi_async(spi, &priv->cas_ctl.rx_msg);

			if (status) {
				pr_crit("[ca8210] Status %d from spi_async in write\n", status);
			}
			local_irq_restore(flags);
			return status;
		}
		else {
			pr_debug("[ca8210] READ WHOLE CMD DURING TX\n");
			/* whole packet read during transfer */
			memcpy(priv->cas_ctl.rx_final_buf, priv->cas_ctl.tx_in_buf, SPI_BUF_SIZE);
			INIT_WORK(&priv->rx_work, ca8210_rx_done);
			queue_work(priv->rx_workqueue, &priv->rx_work);
		}
		#undef NUM_DATABYTES_SO_FAR
	}
	ca8210_spi_writeDummy(spi);
	up(&priv->cas_ctl.spi_sem);
	return status;
}

/**
 * ca8210_spi_startRead() - Start the process of reading a SAP command from the
 *                          ca8210
 * @spi:  Pointer to spi device to write to
 *
 * Reads the command id and command length of a pending SAP command.
 */
static void ca8210_spi_startRead(struct spi_device *spi)
{
	int status;
	struct ca8210_priv *priv = spi_get_drvdata(spi);
	unsigned cpu = smp_processor_id();

	pr_debug("[ca8210] SPI read function -ca8210_spi_startRead- called\n");

	do {
		pr_debug("[ca8210] Trying to get spinlock on CPU%d\n", cpu);
		spin_lock(&priv->lock);
		pr_debug("[ca8210] Got spinlock on CPU%d\n", cpu);
		if (priv->cas_ctl.rx_final_buf[0] == 0xFF) {
			/* spi receive buffer cleared of last rx */
			pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
			spin_unlock(&priv->lock);
			pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);
			break;
		} else {
			/* spi receive buffer still in use */
			pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
			spin_unlock(&priv->lock);
			pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);
			msleep(1);
		}
	} while(1);

	if (down_interruptible(&priv->cas_ctl.spi_sem))
		return;

	memset(priv->cas_ctl.rx_buf, '\xff', SPI_BUF_SIZE);
	memset(priv->cas_ctl.rx_out_buf, '\xff', SPI_BUF_SIZE);

	/* Read the first 2 bytes: CMD and LENGTH */
	priv->cas_ctl.rx_transfer.tx_buf = priv->cas_ctl.rx_out_buf;
	priv->cas_ctl.rx_transfer.rx_buf = priv->cas_ctl.rx_buf;
	priv->cas_ctl.rx_transfer.len = 2;
	/* Keep chip select asserted after reading the bytes */
	priv->cas_ctl.rx_transfer.cs_change = 1;
	spi_message_init(&priv->cas_ctl.rx_msg);
	priv->cas_ctl.rx_msg.complete = ca8210_spi_continueRead;
	priv->cas_ctl.rx_msg.context = spi;
	spi_message_add_tail(&priv->cas_ctl.rx_transfer, &priv->cas_ctl.rx_msg);

	status = spi_async(spi, &priv->cas_ctl.rx_msg);
	if (status) {
		pr_crit("[ca8210] Status %d from spi_async in start read\n", status);
	}
}

/**
 * ca8210_spi_continueRead() - Continue the process of reading a SAP command
 *                             from the ca8210
 * @arg:  Pointer to the spi device read is occuring on
 *
 * Having read the command id and command length from the ca8210, read the
 * variable length payload of the message.
 */
static void ca8210_spi_continueRead(void *arg)
{
	int status;
	struct spi_device *spi = arg;
	struct ca8210_priv *priv = spi_get_drvdata(spi);

	pr_debug("[ca8210] SPI read function -ca8210_spi_continueRead- called\n");

	spi_message_init(&priv->cas_ctl.rx_msg);
	priv->cas_ctl.rx_msg.spi = spi;
	priv->cas_ctl.rx_msg.is_dma_mapped = false;

	if (priv->cas_ctl.rx_final_buf[1] == 0xFF) {
		/* Start of data read */
		priv->cas_ctl.data_received_so_far = 0;

		pr_debug("[ca8210] spi received cmdid: %d, len: %d\n",
			priv->cas_ctl.rx_buf[0], priv->cas_ctl.rx_buf[1]);
		if ( (priv->cas_ctl.rx_buf[0] == 0xFF /* IDLE */) ||
		     (priv->cas_ctl.rx_buf[0] == 0xF0 /* NACK */) ) {
			ca8210_spi_writeDummy(spi);
			up(&priv->cas_ctl.spi_sem);
			return;
		}

		/* Regular data read start */
		priv->cas_ctl.data_left_to_receive = (int) priv->cas_ctl.rx_buf[1];
		priv->cas_ctl.rx_final_buf[0] = priv->cas_ctl.rx_buf[0];
		priv->cas_ctl.rx_final_buf[1] = priv->cas_ctl.rx_buf[1];
	}

	#define SPLIT_RX_LEN 64
	priv->cas_ctl.rx_transfer.rx_buf = priv->cas_ctl.rx_buf + priv->cas_ctl.data_received_so_far;
	if (priv->cas_ctl.data_left_to_receive > SPLIT_RX_LEN) {
		/* Middle of data */
		priv->cas_ctl.rx_transfer.len = SPLIT_RX_LEN;
		priv->cas_ctl.rx_msg.frame_length = SPLIT_RX_LEN;
		priv->cas_ctl.rx_transfer.cs_change = 1;
		priv->cas_ctl.rx_transfer.delay_usecs = 200;
		priv->cas_ctl.rx_msg.complete = ca8210_spi_continueRead;
		priv->cas_ctl.rx_msg.context = spi;
		priv->cas_ctl.data_received_so_far += SPLIT_RX_LEN;
		priv->cas_ctl.data_left_to_receive -= SPLIT_RX_LEN;
	} else {
		/* End of data */
		priv->cas_ctl.rx_transfer.len = priv->cas_ctl.data_left_to_receive;
		priv->cas_ctl.rx_msg.frame_length = priv->cas_ctl.data_left_to_receive;
		priv->cas_ctl.rx_transfer.cs_change = 0;
		priv->cas_ctl.rx_transfer.delay_usecs = 0;
		priv->cas_ctl.rx_msg.complete = ca8210_spi_finishRead;
		priv->cas_ctl.rx_msg.context = spi;
		priv->cas_ctl.data_received_so_far += priv->cas_ctl.data_left_to_receive;
		priv->cas_ctl.data_left_to_receive = 0;
	}
	#undef SPLIT_RX_LEN

	spi_message_add_tail(&priv->cas_ctl.rx_transfer, &priv->cas_ctl.rx_msg);

	status = spi_async(spi, &priv->cas_ctl.rx_msg);

	if (status) {
		pr_crit("[ca8210] Status %d from spi_async in continue read\n", status);
	}
}

/**
 * ca8210_spi_finishRead() - Finishes the process of reading a SAP command from
 *                           the ca8210
 * @arg:  Pointer to the spi device read is occuring on
 *
 * With possession of the complete packet, hand message processing and response
 * off to a workqueue.
 */
static void ca8210_spi_finishRead(void *arg)
{
	int i;
	struct spi_device *spi = arg;
	struct ca8210_priv *priv = spi_get_drvdata(spi);
	unsigned long flags;
	unsigned cpu = smp_processor_id();

	pr_debug("[ca8210] SPI read function -ca8210_spi_finishRead- called\n");

	up(&priv->cas_ctl.spi_sem);

	pr_debug("[ca8210] Trying to get spinlock on CPU%d\n", cpu);
	spin_lock_irqsave(&priv->lock, flags);
	pr_debug("[ca8210] Got spinlock on CPU%d\n", cpu);

	for (i = 0; i < priv->cas_ctl.rx_final_buf[1]; i++) {
		priv->cas_ctl.rx_final_buf[2+i] = priv->cas_ctl.rx_buf[i];
	}

	pr_debug("[ca8210] device_comm: Command ID = %#03x Length = %#03x	Data:",
		priv->cas_ctl.rx_final_buf[0], priv->cas_ctl.rx_final_buf[1]);

	for (i = 2; i < priv->cas_ctl.rx_final_buf[1] + 2; i++) {
		pr_debug("%#03x\n", priv->cas_ctl.rx_final_buf[i]);
	}

	/* Offload rx processing to workqueue */
	queue_work(priv->rx_workqueue, &priv->rx_work);


 	pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
	spin_unlock_irqrestore(&priv->lock, flags);
	pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);
}

/**
 * ca8210_rx_done() - Calls various message dispatches responding to a received
 *                    command
 * @arg:  Pointer to work being executed
 *
 * Presents a received SAP command from the ca8210 to the Cascoda EVBME, test
 * interface and network driver.
 */
static void ca8210_rx_done(struct work_struct *work)
{
	struct ca8210_priv *priv = container_of(work, struct ca8210_priv, rx_work);
	uint8_t buf[SPI_BUF_SIZE];
	uint8_t len;
	unsigned long flags;
	unsigned cpu = smp_processor_id();

	pr_debug("[ca8210] Trying to get spinlock on CPU%d\n", cpu);
	spin_lock_irqsave(&priv->lock, flags);
	pr_debug("[ca8210] Got spinlock on CPU%d\n", cpu);

	len = priv->cas_ctl.rx_final_buf[1] + 2;
	if (len > SPI_BUF_SIZE)
		pr_crit("[ca8210] Received packet len (%d) erroneously long\n", len);

	memcpy(buf, priv->cas_ctl.rx_final_buf, len);
	memset(priv->cas_ctl.rx_final_buf, '\xff', SPI_BUF_SIZE);

	pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
	spin_unlock_irqrestore(&priv->lock, flags);
	pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);

	if (mutex_lock_interruptible(&priv->sync_command_mutex)) {
		return;
	}
	if (priv->sync_command_pending) {
		if (priv->sync_command_response == NULL) {
			priv->sync_command_pending = false;
			mutex_unlock(&priv->sync_command_mutex);
			pr_crit("[ca8210] Sync command provided no response buffer\n");
			return;
		}
		memcpy(priv->sync_command_response, buf, len);
		priv->sync_command_pending = false;
	} else {
		ca8210_test_int_driver_write(buf, len, priv->spi);
	}
	mutex_unlock(&priv->sync_command_mutex);

	ca8210_net_rx(priv->hw, buf, len);

	pr_debug("[ca8210] Trying to get spinlock on CPU%d\n", cpu);
	spin_lock_irqsave(&priv->lock, flags);
	pr_debug("[ca8210] Got spinlock on CPU%d\n", cpu);

	pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
	spin_unlock_irqrestore(&priv->lock, flags);
	pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);
}

/******************************************************************************/
/* Network driver operation */

/**
 * ca8210_net_rx() - Acts upon received SAP commands relevant to the network
 *                   driver
 * @hw:       ieee802154_hw that command was received by
 * @command:  Octet array of received command
 * @len:      Length of the received command
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
	unsigned cpu = smp_processor_id();

	pr_debug("[ca8210] ca8210_net_rx(), CmdID = %d\n", command[0]);


	if (command[0] == SPI_MCPS_DATA_INDICATION) {
		/* Received data */
		pr_debug("[ca8210] Trying to get spinlock on CPU%d\n", cpu);
		spin_lock_irqsave(&priv->lock, flags);
		pr_debug("[ca8210] Got spinlock on CPU%d\n", cpu);
		if (command[26] == priv->lastDSN) {
			pr_debug("[ca8210] DSN %d resend received, ignoring...\n", command[26]);
			pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
			spin_unlock_irqrestore(&priv->lock, flags);
			pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);
			return 0;
		}
		priv->lastDSN = command[26];
		pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
		spin_unlock_irqrestore(&priv->lock, flags);
		pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);
		return ca8210_skb_rx(hw, len-2, command+2);
	} else if (command[0] == SPI_MCPS_DATA_CONFIRM) {
		if (priv->async_tx_pending) {
			return ca8210_async_xmit_complete(hw, command[2], command[3]);
		} else if (priv->sync_tx_pending) {
			priv->sync_tx_pending = false;
		}
	}

	return 0;
}

/**
 * ca8210_skb_rx() - Contructs a properly framed socket buffer from a received
 *                   MCPS_DATA_indication
 * @hw:        ieee802154_hw that MCPS_DATA_indication was received by
 * @len:       Length of MCPS_DATA_indication
 * @data_ind:  Octet array of MCPS_DATA_indication
 *
 * Called by the spi driver whenever a SAP command is received, this function
 * will ascertain whether the command is of interest to the network driver and
 * take necessary action.
 *
 * Return: 0 or linux error code
 */
static int ca8210_skb_rx(struct ieee802154_hw *hw, size_t len, uint8_t *data_ind)
{
	struct ieee802154_hdr hdr;
	int msdulen;
	int hlen;
	struct sk_buff *skb;

	/* Allocate mtu size buffer for every rx packet */
	skb = dev_alloc_skb(IEEE802154_MTU + sizeof(hdr));
	if (skb == NULL) {
		pr_crit("[ca8210] dev_alloc_skb failed\n");
		return -ENOMEM;
	}
	skb_reserve(skb, sizeof(hdr));

	msdulen = data_ind[22]; /* MsduLength */
	pr_debug("[ca8210] skb buffer length = %d\n", msdulen);

	/* Populate hdr */
	hdr.sec.level = data_ind[29 + msdulen];
	pr_debug("[ca8210] security level: %#03x\n", hdr.sec.level);
	if (hdr.sec.level > 0) {
		hdr.sec.key_id_mode = data_ind[30 + msdulen];
		memcpy(&hdr.sec.extended_src, &data_ind[31 + msdulen], 8);
		hdr.sec.key_id = data_ind[39 + msdulen];
	}
	hdr.source.mode = data_ind[0];
	pr_debug("[ca8210] srcAddrMode: %#03x\n", hdr.source.mode);
	hdr.source.pan_id = *(uint16_t*)&data_ind[1];
	pr_debug("[ca8210] srcPanId: %#06x\n", hdr.source.pan_id);
	memcpy(&hdr.source.extended_addr, &data_ind[3], 8);
	hdr.dest.mode = data_ind[11];
	pr_debug("[ca8210] dstAddrMode: %#03x\n", hdr.dest.mode);
	hdr.dest.pan_id = *(uint16_t*)&data_ind[12];
	pr_debug("[ca8210] dstPanId: %#06x\n", hdr.dest.pan_id);
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

	if (hlen < 0)
		return hlen;

	skb_reset_mac_header(skb);
	skb->mac_len = hlen;

	/* Add <msdulen> bytes of space to the back of the buffer */
	/* Copy Msdu to skb */
	memcpy(skb_put(skb, msdulen), &data_ind[29], msdulen);

	ieee802154_rx_irqsafe(hw, skb, data_ind[23]/*LQI*/);
	/* TODO: Set protocol & checksum? */
	/* TODO: update statistics */
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
static int ca8210_skb_tx(struct sk_buff *skb, uint8_t msduhandle, struct ca8210_priv *priv)
{
	int status;
	struct ieee802154_hdr header = { 0 };
	struct SecSpec secspec;

	pr_debug("[ca8210] ca8210_skb_tx() called\n");

	/* Get addressing info from skb - ieee802154 layer creates a full packet*/
	ieee802154_hdr_peek_addrs(skb, &header);

	secspec.SecurityLevel = header.sec.level;
	secspec.KeyIdMode = header.sec.key_id_mode;
	if (secspec.KeyIdMode == 2)
  		memcpy(secspec.KeySource, &header.sec.short_src, 4);
	else if (secspec.KeyIdMode == 3)
		memcpy(secspec.KeySource, &header.sec.extended_src, 8);
	secspec.KeyIndex = header.sec.key_id;

	/* Pass to Cascoda API */
	status =  MCPS_DATA_request(header.source.mode,
	                            header.dest.mode,
	                            header.dest.pan_id,
	                            (union MacAddr*)&header.dest.extended_addr,
	                            skb->len - skb->mac_len,
	                            &skb->data[skb->mac_len],
	                            msduhandle,
	                            header.fc.ack_request,
	                            &secspec,
	                            priv->spi);
	return link_to_linux_err(status);;
}

/**
 * ca8210_async_tx_worker() - Dispatched to handle asynchronous data
 *                            transmission
 * @work:  Work being executed
 */
static void ca8210_async_tx_worker(struct work_struct *work)
{
	struct ca8210_priv *priv = container_of(work, struct ca8210_priv, async_tx_work);
	unsigned long flags;
	unsigned cpu = smp_processor_id();

	if (priv->tx_skb == NULL) {
		return;
	}

	ca8210_skb_tx(priv->tx_skb, priv->nextmsduhandle, priv);

	queue_delayed_work(priv->async_tx_workqueue,
	                   &priv->async_tx_timeout_work,
	                   msecs_to_jiffies(DATA_CNF_TIMEOUT_MS));

	pr_debug("[ca8210] Trying to get spinlock on CPU%d\n", cpu);
	spin_lock_irqsave(&priv->lock, flags);
	pr_debug("[ca8210] Got spinlock on CPU%d\n", cpu);

	priv->async_tx_pending = true;

	pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
	spin_unlock_irqrestore(&priv->lock, flags);
	pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);
}

/**
 * ca8210_async_tx_timeout_worker() - Executed when an asynchronous transmission
 *                                    times out
 * @work:  Work being executed
 */
static void ca8210_async_tx_timeout_worker(struct work_struct *work)
{
	struct delayed_work *del_work = container_of(work, struct delayed_work, work);
	struct ca8210_priv *priv = container_of(del_work, struct ca8210_priv, async_tx_timeout_work);
	unsigned long flags;
	unsigned cpu = smp_processor_id();

	pr_err("[ca8210] data confirm timed out\n");

	pr_debug("[ca8210] Trying to get spinlock on CPU%d\n", cpu);
	spin_lock_irqsave(&priv->lock, flags);
	pr_debug("[ca8210] Got spinlock on CPU%d\n", cpu);

	priv->async_tx_pending = false;

	pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
	spin_unlock_irqrestore(&priv->lock, flags);
	pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);

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
	uint8_t RxOnWhenIdle;
	struct ca8210_priv *priv = hw->priv;

	priv->async_tx_workqueue = alloc_ordered_workqueue("ca8210 tx worker", 0);
	if (priv->async_tx_workqueue == NULL) {
		pr_crit("[ca8210] alloc_ordered_workqueue failed\n");
		return -ENOMEM;
	}
	INIT_WORK(&priv->async_tx_work, ca8210_async_tx_worker);
	INIT_DELAYED_WORK(&priv->async_tx_timeout_work, ca8210_async_tx_timeout_worker);

	priv->lastDSN = -1;
	/* Turn receiver on when idle for now just to test rx */
	RxOnWhenIdle = 1;
	status = MLME_SET_request_sync(macRxOnWhenIdle, 0, 1, &RxOnWhenIdle, priv->spi);
	if (status) {
		pr_crit("[ca8210] Setting RxOnWhenIdle failed, Status = %d\n", status);
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

	pr_debug("[ca8210] calling ca8210_xmit_sync()\n");

	status = ca8210_skb_tx(skb, priv->nextmsduhandle++, priv);
	if (status)
		return status;

	priv->sync_tx_pending = true;

	while(priv->sync_tx_pending){
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

	pr_debug("[ca8210] calling ca8210_xmit_async()\n");

	priv->tx_skb = skb;
	queue_work(priv->async_tx_workqueue, &priv->async_tx_work);

	return 0;
}

/**
 * ca8210_async_xmit_complete() - Called to announce that an asynchronous
 *                                transmission has finished
 * @hw:          ieee802154_hw of ca8210 that has finished exchange
 * @msduhandle:  Identifier of transmission that has completed
 * @status:      Returned 802.15.4 status code of the transmission
 *
 * Return: 0 or linux error code
 */
static int ca8210_async_xmit_complete(struct ieee802154_hw *hw, uint8_t msduhandle, uint8_t status)
{
	struct ca8210_priv *priv = hw->priv;
	unsigned long flags;
	unsigned cpu = smp_processor_id();

	if (status) {
		pr_err("[ca8210] Link transmission unsuccessful, Status = %d\n", status);
		if (status == 0xF1) {
			MLME_RESET_request_sync(0, priv->spi);
		}
	}

	if (priv->nextmsduhandle != msduhandle) {
		pr_crit("[ca8210] Unexpected MsduHandle on data confirm, Expected %d, got %d\n",
			priv->nextmsduhandle,
			msduhandle);
		priv->nextmsduhandle = 0;
		return -EIO;
	}

	/* stop timeout work */
	if (!cancel_delayed_work_sync(&priv->async_tx_timeout_work)) {
		pr_err("[ca8210] async tx timeout wasn't pending when transfer complete\n");
	}

	pr_debug("[ca8210] Trying to get spinlock on CPU%d\n", cpu);
	spin_lock_irqsave(&priv->lock, flags);
	pr_debug("[ca8210] Got spinlock on CPU%d\n", cpu);

	priv->async_tx_pending = false;

	pr_debug("[ca8210] Releasing spinlock on CPU%d\n", cpu);
	spin_unlock_irqrestore(&priv->lock, flags);
	pr_debug("[ca8210] Released spinlock on CPU%d\n", cpu);

	priv->nextmsduhandle++;
	ieee802154_xmit_complete(priv->hw, priv->tx_skb, true);

	return 0;
}

/**
 * ca8210_get_ed() - Returns the measured energy on the current channel at this
 *                   instant in time
 * @hw:  ieee802154_hw of target ca8210
 * @level:  Measured Energy Detect level
 *
 * Return: 0 or linux error code
 */
static int ca8210_get_ed(struct ieee802154_hw *hw, uint8_t *level)
{
	uint8_t LenVar;
	struct ca8210_priv *priv = hw->priv;
	return link_to_linux_err(
		HWME_GET_request_sync(HWME_EDVALUE, &LenVar, level, priv->spi)
	);
}

/**
 * ca8210_set_channel() - Sets the current operating 802.15.4 channel of the
 *                        ca8210
 * @hw:  ieee802154_hw of target ca8210
 * @page:     Channel page to set
 * @channel:  Channel number to set
 *
 * Return: 0 or linux error code
 */
static int ca8210_set_channel(struct ieee802154_hw *hw, uint8_t page, uint8_t channel)
{
	uint8_t status;
	struct ca8210_priv *priv = hw->priv;
	status = MLME_SET_request_sync(phyCurrentChannel, 0, 1, &channel, priv->spi);
	if (status) {
		pr_err("[ca8210] problem setting channel, MLME-SET.confirm status = %d\n", status);
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
static int ca8210_set_hw_addr_filt(struct ieee802154_hw *hw,
                                   struct ieee802154_hw_addr_filt *filt,
                                   unsigned long changed)
{
	uint8_t status = 0;
	struct ca8210_priv *priv = hw->priv;

	if (changed&IEEE802154_AFILT_PANID_CHANGED) {
		status = MLME_SET_request_sync(macPANId, 0, 2, &filt->pan_id, priv->spi);
		if (status) {
			pr_err("[ca8210] problem setting pan id, MLME-SET.confirm status = %d", status);
			return link_to_linux_err(status);
		}
	}
	if (changed&IEEE802154_AFILT_SADDR_CHANGED) {
		status = MLME_SET_request_sync(macShortAddress, 0, 2, &filt->short_addr, priv->spi);
		if (status) {
			pr_err("[ca8210] problem setting short address, MLME-SET.confirm status = %d", status);
			return link_to_linux_err(status);
		}
	}
	if (changed&IEEE802154_AFILT_IEEEADDR_CHANGED) {
		status = MLME_SET_request_sync(nsIEEEAddress, 0, 8, &filt->ieee_addr, priv->spi);
		if (status) {
			pr_err("[ca8210] problem setting ieee address, MLME-SET.confirm status = %d", status);
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
static int ca8210_set_tx_power(struct ieee802154_hw *hw, s32 dbm)
{
	struct ca8210_priv *priv = hw->priv;
	return link_to_linux_err(
		MLME_SET_request_sync(phyTransmitPower, 0, 1, &dbm, priv->spi)
	);
}

/**
 * ca8210_set_lbt() - Sets the listen before transmit capability of the ca8210
 * @hw:  ieee802154_hw of target ca8210
 * @on:  Whether lbt should be turned on or not
 *
 * Return: 0 or linux error code
 */
static int ca8210_set_lbt(struct ieee802154_hw *hw, bool on)
{
	uint8_t Backoffs;
	struct ca8210_priv *priv = hw->priv;
	if (on) {
		Backoffs = 4; /* Default TODO: Does this make sense? */
	} else {
		Backoffs = 0; /* Effectively off */
	}
	return link_to_linux_err(
		MLME_SET_request_sync(macMaxCSMABackoffs, 0, 1, &Backoffs, priv->spi)
	);
}

/**
 * ca8210_set_cca_mode() - Sets the clear channel assessment mode of the ca8210
 * @hw:   ieee802154_hw of target ca8210
 * @cca:  CCA mode to set
 *
 * Return: 0 or linux error code
 */
static int ca8210_set_cca_mode(struct ieee802154_hw *hw, const struct wpan_phy_cca *cca)
{
	uint8_t status;
	uint8_t CCAMode;
	struct ca8210_priv *priv = hw->priv;
	CCAMode = cca->mode & 3;
	if (CCAMode == 3 && cca->opt == NL802154_CCA_OPT_ENERGY_CARRIER_OR) {
		/* CCAMode 0 == CS OR ED, 3 == CS AND ED */
		CCAMode = 0;
	}
	status = MLME_SET_request_sync(phyCCAMode, 0, 1, &CCAMode, priv->spi);
	if (status) {
		pr_err("[ca8210] problem setting cca mode, MLME-SET.confirm status = %d", status);
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
	uint8_t EDTHRESHOLD = level * 2 + 256;
	struct ca8210_priv *priv = hw->priv;
	status = HWME_SET_request_sync(HWME_EDTHRESHOLD, 1, &EDTHRESHOLD, priv->spi);
	if (status) {
		pr_err("[ca8210] problem setting ed threshold, HWME-SET.confirm status = %d", status);
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
static int ca8210_set_csma_params(struct ieee802154_hw *hw, uint8_t min_be, uint8_t max_be, uint8_t retries)
{
	uint8_t status;
	struct ca8210_priv *priv = hw->priv;
	status = MLME_SET_request_sync(macMinBE, 0, 1, &min_be, priv->spi);
	if (status) {
		pr_err("[ca8210] problem setting min be, MLME-SET.confirm status = %d", status);
		return link_to_linux_err(status);
	}
	status = MLME_SET_request_sync(macMaxBE, 0, 1, &max_be, priv->spi);
	if (status) {
		pr_err("[ca8210] problem setting max be, MLME-SET.confirm status = %d", status);
		return link_to_linux_err(status);
	}
	status = MLME_SET_request_sync(macMaxCSMABackoffs, 0, 1, &retries, priv->spi);
	if (status) {
		pr_err("[ca8210] problem setting max csma backoffs, MLME-SET.confirm status = %d", status);
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
	status = MLME_SET_request_sync(macMaxFrameRetries, 0, 1, &retries, priv->spi);
	if (status) {
		pr_err("[ca8210] problem setting panid, MLME-SET.confirm status = %d", status);
	}
	return link_to_linux_err(status);
}

/******************************************************************************/
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
	struct ca8210_test *test = container_of(inodp->i_cdev, struct ca8210_test, char_dev_cdev);
	struct ca8210_priv *priv = container_of(test, struct ca8210_priv, test);
	filp->private_data = priv;
	return 0;
}

/**
 * ca8210_test_check_upstream() - Checks a command received from the upstream
 *                                testing interface for required action
 * @buf:        Buffer containing command to check
 * @pDeviceRef:
 *
 * Return: 0 or linux error code
 */
static int ca8210_test_check_upstream(uint8_t *buf, void *pDeviceRef)
{
	int ret;
	uint8_t response[SPI_BUF_SIZE];
	if (buf[0] == SPI_MLME_SET_REQUEST) {
		ret = TDME_CheckPIBAttribute(buf[2], buf[4], buf + 5);
		if (ret) {
			response[0]  = SPI_MLME_SET_CONFIRM;
			response[1] = 3;
			response[2] = MAC_INVALID_PARAMETER;
			response[3] = buf[2];
			response[4] = buf[3];
			cascoda_api_upstream(response, 5, pDeviceRef);
			return ret;
		}
	}
	if (        buf[0] == SPI_MLME_ASSOCIATE_REQUEST) {
		return TDME_ChannelInit(buf[2], pDeviceRef);
	} else if ( buf[0] == SPI_MLME_START_REQUEST) {
		return TDME_ChannelInit(buf[4], pDeviceRef);
	} else if ((buf[0] == SPI_MLME_SET_REQUEST) && (buf[2] == phyCurrentChannel)) {
		return TDME_ChannelInit(buf[5], pDeviceRef);
	} else if ((buf[0] == SPI_TDME_SET_REQUEST) && (buf[2] == TDME_CHANNEL)) {
		return TDME_ChannelInit(buf[4], pDeviceRef);
	} else if ((MAC_Workarounds) && (buf[0] == SPI_MLME_RESET_REQUEST) && (buf[2] == 1)) {
		/* reset COORD Bit for Channel Filtering as Coordinator */
		return TDME_SETSFR_request_sync(0, 0xD8, 0, pDeviceRef);
	}
	return 0;
} /* End of EVBMECheckSerialCommand() */

/**
 * ca8210_test_int_user_write() - Called by a process in userspace to send a
 *                                message to the ca8210 drivers
 * @filp:    file interface
 * @in_buf:  Buffer containing message to write
 * @len:     Length of message
 * @off:     file offset
 *
 * Return: 0 or linux error code
 */
static ssize_t ca8210_test_int_user_write(struct file *filp, const char *in_buf, size_t len, loff_t *off)
{
	int ret;
	struct ca8210_priv *priv = filp->private_data;
	uint8_t command[SPI_BUF_SIZE];

	if (copy_from_user(command, in_buf, len))
		return 0;

	ret = ca8210_test_check_upstream(command, priv->spi);
	if (ret == 0) {
		ca8210_spi_exchange(command, command[1] + 2, NULL, priv->spi);
	}

	return len;
}

/**
 * ca8210_test_int_user_read() - Called by a process in userspace to read a
 *                               message from the ca8210 drivers
 * @filp:  file interface
 * @buf:   Buffer to write message to
 * @len:   Length of message to read (ignored)
 * @offp:  file offset
 *
 * Return: 0 or linux error code
 */
static ssize_t ca8210_test_int_user_read(struct file *filp, char __user *buf, size_t len, loff_t *offp)
{
	int i, cmdlen;
	struct ca8210_priv *priv = filp->private_data;
	unsigned char *fifo_buffer;

	if (kfifo_is_empty(&priv->test.up_fifo))
		return 0;

	if (kfifo_out(&priv->test.up_fifo, &fifo_buffer, 4) != 4) {
		pr_err("[ca8210] test_interface: Wrong number of elements popped from upstream fifo\n");
		return 0;
	}
	cmdlen = fifo_buffer[1];
	memcpy(buf, fifo_buffer, cmdlen + 2);

	kfree(fifo_buffer);

	pr_debug("[ca8210] test_interface: Cmd len = %d\n", cmdlen);

	pr_debug("[ca8210] test_interface: Read\n");
	for (i = 0; i < cmdlen+2; i++) {
		pr_debug("%#03x\n", buf[i]);
	}

	return cmdlen+2;
}

/**
 * ca8210_test_int_driver_write() - Writes a message to the test interface to be
 *                                  read by the userspace
 * @buf:  Buffer containing upstream message
 * @len:  Length of message to write
 * @spi:  SPI device of message originator
 *
 * Return: 0 or linux error code
 */
static int ca8210_test_int_driver_write(const uint8_t *buf, size_t len, void *spi)
{
	struct ca8210_priv *priv = spi_get_drvdata(spi);
	struct ca8210_test *test = &priv->test;
	char *fifo_buffer;
	int i;

	pr_debug("[ca8210] test_interface: Buffering upstream message:\n");
	for (i = 0; i < len; i++) {
		pr_debug("%#03x\n", buf[i]);
	}

	fifo_buffer = kmalloc(len, GFP_KERNEL);
	memcpy(fifo_buffer, buf, len);
	kfifo_in(&test->up_fifo, &fifo_buffer, 4);

	return 0;
}

/******************************************************************************/
/* 15.4 SAP Primitives */

/**
 * MCPS_DATA_request() - MCPS_DATA_request (Send Data) according to API Spec
 * @SrcAddrMode: Source Addressing Mode
 * @DstAddrMode: Destination Addressing Mode
 * @DstPANId:    Destination PAN ID
 * @pDstAddr:    Pointer to Destination Address
 * @MsduLength:  Length of Data
 * @pMsdu:       Pointer to Data
 * @MsduHandle:  Handle of Data
 * @TxOptions:   Tx Options Bit Field
 * @pSecurity:   Pointer to Security Structure or NULL
 * @pDeviceRef:  Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of action
 */
static uint8_t MCPS_DATA_request(
	uint8_t          SrcAddrMode,
	uint8_t          DstAddrMode,
	uint16_t         DstPANId,
	union MacAddr  *pDstAddr,
	uint8_t          MsduLength,
	uint8_t         *pMsdu,
	uint8_t          MsduHandle,
	uint8_t          TxOptions,
	struct SecSpec  *pSecurity,
	void            *pDeviceRef
)
{
	struct SecSpec *pSec;
	struct MAC_Message Command;
	#define DATAREQ (Command.PData.DataReq)
	Command.CommandId = SPI_MCPS_DATA_REQUEST;
	DATAREQ.SrcAddrMode = SrcAddrMode;
	DATAREQ.Dst.AddressMode = DstAddrMode;
	if (DstAddrMode != MAC_MODE_NO_ADDR) {
		DATAREQ.Dst.PANId[0] = LS_BYTE(DstPANId);
		DATAREQ.Dst.PANId[1] = MS_BYTE(DstPANId);
		if (DstAddrMode == MAC_MODE_SHORT_ADDR) {
			DATAREQ.Dst.Address[0] = LS_BYTE(pDstAddr->ShortAddress);
			DATAREQ.Dst.Address[1] = MS_BYTE(pDstAddr->ShortAddress);
		} else {   /* MAC_MODE_LONG_ADDR*/
			memcpy(DATAREQ.Dst.Address, pDstAddr->IEEEAddress, 8);
		}
	}
	DATAREQ.MsduLength = MsduLength;
	DATAREQ.MsduHandle = MsduHandle;
	DATAREQ.TxOptions = TxOptions;
	memcpy(DATAREQ.Msdu, pMsdu, MsduLength);
	pSec = (struct SecSpec*)(DATAREQ.Msdu + MsduLength);
	Command.Length = sizeof(struct MCPS_DATA_request_pset) - MAX_DATA_SIZE + MsduLength;
	if ( (pSecurity == NULL) || (pSecurity->SecurityLevel == 0) ) {
		pSec->SecurityLevel = 0;
		Command.Length += 1;
	} else {
		*pSec = *pSecurity;
		Command.Length += sizeof(struct SecSpec);
	}

	if (cascoda_api_downstream(&Command.CommandId, Command.Length + 2, NULL, pDeviceRef))
		return MAC_SYSTEM_ERROR;

	return MAC_SUCCESS;
	#undef DATAREQ
}

/**
 * MLME_RESET_request_sync() - MLME_RESET_request/confirm according to API Spec
 * @SetDefaultPIB: Set defaults in PIB
 * @pDeviceRef:    Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of MLME-RESET.confirm
 */
static uint8_t MLME_RESET_request_sync(uint8_t SetDefaultPIB, void *pDeviceRef)
{
	uint8_t status;
	struct MAC_Message Command, Response;
	#define SIMPLEREQ (Command.PData)
	#define SIMPLECNF (Response.PData)
	Command.CommandId = SPI_MLME_RESET_REQUEST;
	Command.Length = 1;
	SIMPLEREQ.u8Param = SetDefaultPIB;

	if (cascoda_api_downstream(&Command.CommandId, Command.Length + 2, &Response.CommandId, pDeviceRef))
		return MAC_SYSTEM_ERROR;

	if (Response.CommandId != SPI_MLME_RESET_CONFIRM)
		return MAC_SYSTEM_ERROR;

	status = SIMPLECNF.Status;

	/* reset COORD Bit for Channel Filtering as Coordinator */
	if (MAC_Workarounds && SetDefaultPIB && (!status))
		status = TDME_SETSFR_request_sync(0, 0xD8, 0, pDeviceRef);

	return status;
	#undef SIMPLEREQ
	#undef SIMPLECNF
}

/**
 * MLME_SET_request_sync() - MLME_SET_request/confirm according to API Spec
 * @PIBAttribute:       Attribute Number
 * @PIBAttributeIndex:  Index within Attribute if an Array
 * @PIBAttributeLength: Attribute Length
 * @pPIBAttributeValue: Pointer to Attribute Value
 * @pDeviceRef:         Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of MLME-SET.confirm
 */
static uint8_t MLME_SET_request_sync(
	uint8_t       PIBAttribute,
	uint8_t       PIBAttributeIndex,
	uint8_t       PIBAttributeLength,
	const void   *pPIBAttributeValue,
	void         *pDeviceRef
)
{
	uint8_t status;
	struct MAC_Message Command, Response;
	#define SETREQ    (Command.PData.SetReq)
	#define SIMPLECNF (Response.PData)
	/* pre-check the validity of PIBAttribute values that are not checked in MAC */
	if (TDME_CheckPIBAttribute(PIBAttribute, PIBAttributeLength, pPIBAttributeValue))
		return MAC_INVALID_PARAMETER;

	if (PIBAttribute == phyCurrentChannel) {
		status = TDME_ChannelInit(*((uint8_t*)pPIBAttributeValue), pDeviceRef);
		if (status) {
			return status;
		}
	}

	if (PIBAttribute == phyTransmitPower)
		return(TDME_SetTxPower(*((uint8_t*)pPIBAttributeValue), pDeviceRef));

	Command.CommandId = SPI_MLME_SET_REQUEST;
	Command.Length = sizeof(struct MLME_SET_request_pset) - MAX_ATTRIBUTE_SIZE + PIBAttributeLength;
	SETREQ.PIBAttribute = PIBAttribute;
	SETREQ.PIBAttributeIndex = PIBAttributeIndex;
	SETREQ.PIBAttributeLength = PIBAttributeLength;
	memcpy( SETREQ.PIBAttributeValue, pPIBAttributeValue, PIBAttributeLength );

	if (cascoda_api_downstream(&Command.CommandId, Command.Length + 2, &Response.CommandId, pDeviceRef))
		return MAC_SYSTEM_ERROR;

	if (Response.CommandId != SPI_MLME_SET_CONFIRM)
		return MAC_SYSTEM_ERROR;

	return SIMPLECNF.Status;
	#undef SETREQ
	#undef SIMPLECNF
}

/**
 * HWME_SET_request_sync() - HWME_SET_request/confirm according to API Spec
 * @HWAttribute:       Attribute Number
 * @HWAttributeLength: Attribute Length
 * @pHWAttributeValue: Pointer to Attribute Value
 * @pDeviceRef:        Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of HWME-SET.confirm
 */
static uint8_t HWME_SET_request_sync(
	uint8_t      HWAttribute,
	uint8_t      HWAttributeLength,
	uint8_t     *pHWAttributeValue,
	void        *pDeviceRef
)
{
	struct MAC_Message Command, Response;
	Command.CommandId = SPI_HWME_SET_REQUEST;
	Command.Length = 2 + HWAttributeLength;
	Command.PData.HWMESetReq.HWAttribute = HWAttribute;
	Command.PData.HWMESetReq.HWAttributeLength = HWAttributeLength;
	memcpy(Command.PData.HWMESetReq.HWAttributeValue, pHWAttributeValue, HWAttributeLength);

	if (cascoda_api_downstream(&Command.CommandId, Command.Length + 2, &Response.CommandId, pDeviceRef))
		return MAC_SYSTEM_ERROR;

	if (Response.CommandId != SPI_HWME_SET_CONFIRM)
		return MAC_SYSTEM_ERROR;

	return Response.PData.HWMESetCnf.Status;
}

/**
 * HWME_GET_request_sync() - HWME_GET_request/confirm according to API Spec
 * @HWAttribute:       Attribute Number
 * @HWAttributeLength: Attribute Length
 * @pHWAttributeValue: Pointer to Attribute Value
 * @pDeviceRef:        Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of HWME-GET.confirm
 */
static uint8_t HWME_GET_request_sync(
	uint8_t      HWAttribute,
	uint8_t     *HWAttributeLength,
	uint8_t     *pHWAttributeValue,
	void        *pDeviceRef
)
{
	struct MAC_Message Command, Response;
	Command.CommandId = SPI_HWME_GET_REQUEST;
	Command.Length = 1;
	Command.PData.HWMEGetReq.HWAttribute = HWAttribute;

	if (cascoda_api_downstream(&Command.CommandId, Command.Length + 2, &Response.CommandId, pDeviceRef))
		return MAC_SYSTEM_ERROR;

	if (Response.CommandId != SPI_HWME_GET_CONFIRM)
		return MAC_SYSTEM_ERROR;

	if (Response.PData.HWMEGetCnf.Status == MAC_SUCCESS) {
		*HWAttributeLength = Response.PData.HWMEGetCnf.HWAttributeLength;
		memcpy(pHWAttributeValue, Response.PData.HWMEGetCnf.HWAttributeValue, *HWAttributeLength);
	}

	return Response.PData.HWMEGetCnf.Status;
}

/**
 * TDME_SETSFR_request_sync() - TDME_SETSFR_request/confirm according to API Spec
 * @SFRPage:    SFR Page
 * @SFRAddress: SFR Address
 * @SFRValue:   SFR Value
 * @pDeviceRef: Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of TDME-SETSFR.confirm
 */
static uint8_t TDME_SETSFR_request_sync(
	uint8_t      SFRPage,
	uint8_t      SFRAddress,
	uint8_t      SFRValue,
	void         *pDeviceRef
)
{
	int ret;
	struct MAC_Message Command, Response;
	Command.CommandId = SPI_TDME_SETSFR_REQUEST;
	Command.Length = 3;
	Command.PData.TDMESetSFRReq.SFRPage    = SFRPage;
	Command.PData.TDMESetSFRReq.SFRAddress = SFRAddress;
	Command.PData.TDMESetSFRReq.SFRValue   = SFRValue;
	Response.CommandId = 0xFF;
	ret = cascoda_api_downstream(&Command.CommandId, Command.Length + 2, &Response.CommandId, pDeviceRef);
	if (ret) {
		pr_crit("cascoda_api_downstream returned %d", ret);
		return MAC_SYSTEM_ERROR;
	}

	if (Response.CommandId != SPI_TDME_SETSFR_CONFIRM) {
		pr_crit("sync response to SPI_TDME_SETSFR_REQUEST was not SPI_TDME_SETSFR_CONFIRM, it was %d\n", Response.CommandId);
		return MAC_SYSTEM_ERROR;
	}

	return Response.PData.TDMESetSFRCnf.Status;
}

/**
 * TDME_ChipInit() - TDME Chip Register Default Initialisation Macro
 * @pDeviceRef: Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of API calls
 */
static uint8_t TDME_ChipInit(void *pDeviceRef)
{
	uint8_t status;

	if((status = TDME_SETSFR_request_sync(1, 0xE1, 0x29, pDeviceRef)))  /* LNA Gain Settings */
		return(status);
	if((status = TDME_SETSFR_request_sync(1, 0xE2, 0x54, pDeviceRef)))
		return(status);
	if((status = TDME_SETSFR_request_sync(1, 0xE3, 0x6C, pDeviceRef)))
		return(status);
	if((status = TDME_SETSFR_request_sync(1, 0xE4, 0x7A, pDeviceRef)))
		return(status);
	if((status = TDME_SETSFR_request_sync(1, 0xE5, 0x84, pDeviceRef)))
		return(status);
	if((status = TDME_SETSFR_request_sync(1, 0xE6, 0x8B, pDeviceRef)))
		return(status);
	if((status = TDME_SETSFR_request_sync(1, 0xE7, 0x92, pDeviceRef)))
		return(status);
	if((status = TDME_SETSFR_request_sync(1, 0xE9, 0x96, pDeviceRef)))
		return(status);
	if((status = TDME_SETSFR_request_sync(1, 0xD3, 0x5B, pDeviceRef))) /* Preamble Timing Config */
		return(status);
	if((status = TDME_SETSFR_request_sync(1, 0xD1, 0x5A, pDeviceRef))) /* Preamble Threshold High */
		return(status);
	if((status = TDME_SETSFR_request_sync(0, 0xFE, 0x3F, pDeviceRef))) /* Tx Output Power 8 dBm */
		return(status);

	return MAC_SUCCESS;
}

/**
 * TDME_ChannelInit() - TDME Channel Register Default Initialisation Macro (Tx)
 * @channel:    802.15.4 channel to initialise chip for
 * @pDeviceRef: Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of API calls
 */
static uint8_t TDME_ChannelInit(uint8_t channel, void *pDeviceRef)
{
	uint8_t txcalval;

	if (       channel >= 25) {
		txcalval = 0xA7;
	} else if (channel >= 23) {
		txcalval = 0xA8;
	} else if (channel >= 22) {
		txcalval = 0xA9;
	} else if (channel >= 20) {
		txcalval = 0xAA;
	} else if (channel >= 17) {
		txcalval = 0xAB;
	} else if (channel >= 16) {
		txcalval = 0xAC;
	} else if (channel >= 14) {
		txcalval = 0xAD;
	} else if (channel >= 12) {
		txcalval = 0xAE;
	} else {
		txcalval = 0xAF;
	}

	return TDME_SETSFR_request_sync(1, 0xBF, txcalval, pDeviceRef);  /* LO Tx Cal */
}

/**
 * TDME_CheckPIBAttribute() - Checks Attribute Values that are not checked in
 *                            MAC
 * @PIBAttribute:       Attribute Number
 * @PIBAttributeLength: Attribute Length
 * @pPIBAttributeValue: Pointer to Attribute Value
 * @pDeviceRef:         Nondescript pointer to target device
 *
 * Return: 802.15.4 status code of checks
 */
static uint8_t TDME_CheckPIBAttribute(
	uint8_t      PIBAttribute,
	uint8_t      PIBAttributeLength,
	const void   *pPIBAttributeValue
)
{
	uint8_t status = MAC_SUCCESS;
	uint8_t value;

	value  = *((uint8_t*)pPIBAttributeValue);

	switch (PIBAttribute) {
	/* PHY */
	case phyTransmitPower:
		if (value > 0x3F)
			status = MAC_INVALID_PARAMETER;
		break;
	case phyCCAMode:
		if (value > 0x03)
			status = MAC_INVALID_PARAMETER;
		break;
	/* MAC */
	case macBattLifeExtPeriods:
		if ((value < 6) || (value > 41))
			status = MAC_INVALID_PARAMETER;
		break;
	case macBeaconPayload:
		if (PIBAttributeLength > aMaxBeaconPayloadLength)
			status = MAC_INVALID_PARAMETER;
		break;
	case macBeaconPayloadLength:
		if (value > aMaxBeaconPayloadLength)
			status = MAC_INVALID_PARAMETER;
		break;
	case macBeaconOrder:
		if (value > 15)
			status = MAC_INVALID_PARAMETER;
		break;
	case macMaxBE:
		if ((value < 3) || (value > 8))
			status = MAC_INVALID_PARAMETER;
		break;
	case macMaxCSMABackoffs:
		if (value > 5)
			status = MAC_INVALID_PARAMETER;
		break;
	case macMaxFrameRetries:
		if (value > 7)
			status = MAC_INVALID_PARAMETER;
		break;
	case macMinBE:
		if (value > 8)
			status = MAC_INVALID_PARAMETER;
		break;
	case macResponseWaitTime:
		if ((value < 2) || (value > 64))
			status = MAC_INVALID_PARAMETER;
		break;
	case macSuperframeOrder:
		if (value > 15)
			status = MAC_INVALID_PARAMETER;
		break;
	/* boolean */
	case macAssociatedPANCoord:
	case macAssociationPermit:
	case macAutoRequest:
	case macBattLifeExt:
	case macGTSPermit:
	case macPromiscuousMode:
	case macRxOnWhenIdle:
	case macSecurityEnabled:
		if (value > 1)
			status = MAC_INVALID_PARAMETER;
		break;
	/* MAC SEC */
	case macAutoRequestSecurityLevel:
		if (value > 7)
			status = MAC_INVALID_PARAMETER;
		break;
	case macAutoRequestKeyIdMode:
		if (value > 3)
			status = MAC_INVALID_PARAMETER;
		break;
	default:
		break;
	}

	return status;
}

/**
 * TDME_SetTxPower() - Sets the tx power for MLME_SET phyTransmitPower
 * @txp:        Transmit Power
 * @pDeviceRef: Nondescript pointer to target device
 *
 * Normalised to 802.15.4 Definition (6-bit, signed):
 * Bit 7-6: not used
 * Bit 5-0: tx power (-32 - +31 dB)
 *
 * Return: 802.15.4 status code of api calls
 */
static uint8_t TDME_SetTxPower(uint8_t txp, void *pDeviceRef)
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

	if (MAC_MPW) {
		if (txp_val > 0) {
			paib = 0xD3; /* 8 dBm: ptrim = 5, itrim = +3 => +4 dBm */
		} else {
			paib = 0x73; /* 0 dBm: ptrim = 7, itrim = +3 => -6 dBm */
		}
		/* write PACFG */
		status = TDME_SETSFR_request_sync(0, 0xB1, paib, pDeviceRef);
	} else {
		/* Look-Up Table for Setting Current and Frequency Trim values for desired Output Power */
		if(       txp_val  >  8) {
			paib = 0x3F;
		} else if(txp_val ==  8) {
			paib = 0x32;
		} else if(txp_val ==  7) {
			paib = 0x22;
		} else if(txp_val ==  6) {
			paib = 0x18;
		} else if(txp_val ==  5) {
			paib = 0x10;
		} else if(txp_val ==  4) {
			paib = 0x0C;
		} else if(txp_val ==  3) {
			paib = 0x08;
		} else if(txp_val ==  2) {
			paib = 0x05;
		} else if(txp_val ==  1) {
			paib = 0x03;
		} else if(txp_val ==  0) {
			paib = 0x01;
		} else         /*  <  0 */ {
			paib = 0x00;
		}
		/* write PACFGIB */
		status = TDME_SETSFR_request_sync(0, 0xFE, paib, pDeviceRef);
	}

	return status;
}

/******************************************************************************/
/* Module */

static int __init ca8210_init(void)
{
	pr_info("[ca8210] Starting module ca8210\n");
	spi_register_driver(&ca8210_spi_driver);
	pr_info("[ca8210] Module started\n");
	return 0;
}
module_init(ca8210_init);

static void __exit ca8210_exit(void)
{
	pr_info("[ca8210] Stopping module ca8210\n");
	spi_unregister_driver(&ca8210_spi_driver);
	pr_info("[ca8210] Module stopped\n");
}
module_exit(ca8210_exit);

MODULE_AUTHOR("Harry Morris");
MODULE_DESCRIPTION("CA-8210 SoftMAC driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("1.0");
