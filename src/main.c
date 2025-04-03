/*
 * Copyright (c) 2025 Br1Ge9se
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * 
 * Author: Bruno Genovese <br1ge9se>
 *
 */

/**
 * @file
 * @brief Application main entry point and machine loop
 *
 */

/********************************* Includes *************************************/
#include "util.h"
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
//#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/sensor.h>
#ifdef DONGLE
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/uart.h>
#endif
/*
#ifndef DONGLE
#include <zephyr/console/console.h>
#endif
*/

#include "util.h"

/********************************* Defines *************************************/

/* 250 msec */
#define SLEEP_TIME_MS   250

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define NUM_OF_LEDS 4
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
/* The devicetree node identifier for the "led1" alias. */
#define LED1_NODE DT_ALIAS(led1)
/* The devicetree node identifier for the "led2" alias. */
#define LED2_NODE DT_ALIAS(led2)
/* The devicetree node identifier for the "led3" alias. */
#define LED3_NODE DT_ALIAS(led3)

#define BLUE_LED 3

#define CLIENT_MANUFACTER	"Br1Ge9se"
#define CLIENT_MODEL		"nRF52840_Acclient"
#define CLIENT_NAME			"BLE Throttle Transmitter V"

#define SAMPLE_PRODUCT      "BLE Throttle Central"
#define SAMPLE_MFR          "Br1Ge9se"
#define SAMPLE_SN           "0123456789ABCDEF"


/* SEVICE UUID: 0x75c276c3, 0x8f97, 0x20bc, 0xa143, 0xb354244886d4 */
#define MY_SERVICE_UUID 0xd4, 0x86, 0x48, 0x24, 0x54, 0xB3, 0x43, 0xA1, \
			            0xBC, 0x20, 0x97, 0x8F, 0xC3, 0x76, 0xC2, 0x75
#define TX_CHARACTERISTIC_UUID  0xED, 0xAA, 0x20, 0x11, 0x92, 0xE7, 0x43, 0x5A, \
			                    0xAA, 0xE9, 0x94, 0x43, 0x35, 0x6A, 0xD4, 0xD3
#define RX_CHARACTERISTIC_UUID 0xA6, 0xE8, 0xC4, 0x60, 0x7E, 0xAA, 0x41, 0x6B, \
								0x95, 0xD4, 0x9D, 0xCC, 0x08, 0x4F, 0xCF, 0x6A

#define NAME_LEN            30

#define SERVICE_UUID BT_UUID_DECLARE_128(MY_SERVICE_UUID)

#define BT_UUID_MY_SERVICE_TX BT_UUID_DECLARE_128(TX_CHARACTERISTIC_UUID)

#define BT_UUID_MY_SERVICE_RX BT_UUID_DECLARE_128(RX_CHARACTERISTIC_UUID)

#define RING_BUF_SIZE 1024

/**
 * Button to read the battery value
 */
#define KEY_READVAL_MASK DK_BTN1_MSK

/**
 * Chip temperature
 */
#define SENSOR_NODE DT_NODELABEL(temp)
#define SENSOR_DATA_TYPE SENSOR_CHAN_DIE_TEMP


/********************************* Prototypes *************************************/

static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void sync_cb(struct bt_le_per_adv_sync *sync,
	struct bt_le_per_adv_sync_synced_info *info);
static void term_cb(struct bt_le_per_adv_sync *sync,
	const struct bt_le_per_adv_sync_term_info *info);
static void recv_cb(struct bt_le_per_adv_sync *sync,
	const struct bt_le_per_adv_sync_recv_info *info,
	struct net_buf_simple *buf);

	
/********************************* Storages *************************************/

static bool per_adv_found;
static bt_addr_le_t per_addr;
static uint8_t per_sid;
static struct bt_conn *default_conn;

/* Semaphores */
static K_SEM_DEFINE(sem_conn, 0, 1);
static K_SEM_DEFINE(sem_conn_lost, 0, 1);
static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led[NUM_OF_LEDS] = 
{
	GPIO_DT_SPEC_GET(LED0_NODE, gpios),
	GPIO_DT_SPEC_GET(LED1_NODE, gpios),
	GPIO_DT_SPEC_GET(LED2_NODE, gpios),
	GPIO_DT_SPEC_GET(LED3_NODE, gpios)
};

static struct bt_conn *default_conn;
static struct bt_uuid_128 uuid = BT_UUID_INIT_128(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

static const struct device *console_dev = NULL;

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb
};

static const struct device *temp_dev = DEVICE_DT_GET(SENSOR_NODE);

static bool connected_ok = false;

static bool leds_initialized = false;

static bool shell_flag = false;

static int in_data = '\n';

static uint8_t rx_val;

static uint32_t timercnt = 0;
static uint32_t timeout;

static char per[BT_ADDR_LE_STR_LEN];

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params);

#ifdef DONGLE
static uint8_t ring_buffer[RING_BUF_SIZE];

static struct ring_buf ringbuf;

USBD_CONFIGURATION_DEFINE(config_1,
	USB_SCD_SELF_POWERED,
	200);
USBD_DESC_LANG_DEFINE(sample_lang);
USBD_DESC_STRING_DEFINE(sample_mfr, SAMPLE_MFR, 1);
USBD_DESC_STRING_DEFINE(sample_product, SAMPLE_PRODUCT, 2);
USBD_DESC_STRING_DEFINE(sample_sn, SAMPLE_SN, 3);
USBD_DEVICE_DEFINE(sample_usbd,
    DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
    0x2fe3, 0x0001);
#endif

extern void run_shell (void);

/********************************* Functions *************************************/


/**
 * @brief Key pressed handler
 *
 */
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	uint32_t button = button_state & has_changed;

	if (button & KEY_READVAL_MASK) {
		
		my_printf("!!! System restart !!!");
		k_sleep(K_MSEC(100));
		sys_reboot(SYS_REBOOT_WARM);

//		in_data = '\n';
//		run_shell();
	}
}

/**
 * @brief Print out chip temperature value
 *
 */
int print_temp_value(void)
{
	int r;
	struct sensor_value temp_value;
	char bf[32];

	r = sensor_sample_fetch(temp_dev);
	if (r) {
		my_printf("Temperature sensor sample fetch failed return: %d\n", r);
		return 1;
	}

	r = sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP,	&temp_value);
	if (r) {
		my_printf("Temperature sensor channel get failed return: %d\n", r);
		return 1;
	}

	sprintf(bf, "Chip temperature is %g°C\n", sensor_value_to_double(&temp_value));
	my_printf(bf);

	return 0;
}

#ifdef DONGLE
/**
 * @brief Uart Interrupt Handler
 *
 * @param[in] 
 */
static void uart_interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				//LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				//LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}
		}
	}
}
#endif

/**
 * @brief Function for starting console.
 *
 */
static int start_console(void) {
#ifdef DONGLE
    uint32_t baudrate, dtr = 0U;
	int cnt, ret = 0;
    
    console_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(console_dev)) {
		//LOG_ERR("CDC ACM device not ready");
		return 1;
	}

	usb_enable(NULL);

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	//LOG_INF("Wait for DTR");
	cnt = 1;
    while (true) {
		uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
			cnt--;
			if (cnt == 0)
				break;
		}
	}

	//LOG_INF("DTR set");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(console_dev, UART_LINE_CTRL_DCD, 1);
_err:
	if (ret != 0) {
		//LOG_ERR("Failed to enable USB");
		return ret;
	}

	ret = uart_line_ctrl_set(console_dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		//LOG_WRN("Failed to set DSR, ret code %d", ret);
		goto _err;
	}

	/* Wait 1 sec for the host to do all settings */
	k_busy_wait(1000000);

	ret = uart_line_ctrl_get(console_dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		//LOG_WRN("Failed to get baudrate, ret code %d", ret);
		goto _err;
	} else {
		//LOG_INF("Baudrate detected: %d", baudrate);
	}

	uart_irq_callback_set(console_dev, uart_interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(console_dev);
    
    /* Init console */
	init_console(console_dev);
    
    return ret;
#else
	return 0;
#endif
}

/**
 * @brief Function for checking DTR.
 *
 */
void check_dtr (void) {
#ifdef DONGLE
	uint32_t dtr = 0U;

	console_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(console_dev)) {
		set_print_flag(false);
		return;
	}

	uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
	if (dtr) {
		if (chk_print_flag() != true) {
			usb_enable(NULL);
			ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);
			/* Wait 1 sec for the host to do all settings */
			k_busy_wait(1000000);
			/* Init my_printf */
			init_console(console_dev);
			set_print_flag(true);
		}
	} else if (chk_print_flag() == true) {
		for (int tmr=0; tmr<10; tmr++) {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
			uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) 
				break;
		}
		set_print_flag(false);
	}
#endif
}

/**
 * @brief Function for 
 *
 */
static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

/**
 * @brief Function for 
 *
 * @param
 */
static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

/**
 * @brief Function for 
 *
 * @param
 */
static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN];
	int err;

	/* only parse devices in close proximity */
	if (info->rssi < -65) {
		return;
	}

	(void)memset(name, 0, sizeof(name));

	bt_data_parse(buf, data_cb, name);

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	/* If connectable */
	if (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) {
		my_printf("[DEVICE]: %s, AD evt type %u, Tx Pwr: %i, RSSI %i, name: %s "
	       "C:%u S:%u D:%u SR:%u E:%u Prim: %s, Secn: %s, "
	       "Interval: 0x%04x (%u ms), SID: %u\n",
	       le_addr, info->adv_type, info->tx_power, info->rssi, name,
	       (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0,
	       phy2str(info->primary_phy), phy2str(info->secondary_phy),
	       info->interval, info->interval * 5 / 4, info->sid);

		/* connect */
		if ((default_conn) || (strncmp(name, CLIENT_NAME, 26) != 0)) {
			return;
		}

		my_printf("Connecting to %s\n", le_addr);

		err = bt_le_scan_stop();
		if (err != 0) {
			my_printf("Stop LE scan failed (err %d)\n", err);
			return;
		}

		err = bt_conn_le_create(info->addr, BT_CONN_LE_CREATE_CONN,
					BT_LE_CONN_PARAM_DEFAULT,
					&default_conn);
		if (err != 0) {
			my_printf("Failed to connect (err %d)\n", err);
			return;
		}
	} else {
		/* If info->interval it is a periodic advertiser, mark for sync */
		if (!per_adv_found && info->interval) {
			per_adv_found = true;

			per_sid = info->sid;
			bt_addr_le_copy(&per_addr, info->addr);

			k_sem_give(&sem_per_adv);
		}
	}
}

/**
 * @brief Function for 
 *
 * @param
 */
static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

/**
 * @brief Function for 
 *
 * @param
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int bt_err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err != 0) {
		my_printf("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;


		bt_err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
		if (bt_err) {
			my_printf("Failed to start scan (err %d)\n", bt_err);
			return;
		}

		return;
	}

	if (conn != default_conn) {
		return;
	}

	my_printf("Connected: %s\n", addr);

	k_sem_give(&sem_conn);
	memcpy(&uuid, SERVICE_UUID, sizeof(uuid));

    discover_params.uuid = &uuid.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    err = bt_gatt_discover(default_conn, &discover_params);
    if (err) {
        my_printf("Discover failed(err %d)\n", err);
        return;
    }
    
	connected_ok = true;
}

/**
 * @brief Function for 
 *
 * @param
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	my_printf("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;
	connected_ok = false;

	k_sem_give(&sem_conn_lost);

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
	if (err != 0) {
		my_printf("Failed to start scan (err %d)\n", err);
		return;
	}
}

/**
 * @brief Function for 
 *
 * @param
 */
static void sync_cb(struct bt_le_per_adv_sync *sync,
		    struct bt_le_per_adv_sync_synced_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	my_printf("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
	       "Interval 0x%04x (%u ms), PHY %s\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr,
	       info->interval, info->interval * 5 / 4, phy2str(info->phy));

	k_sem_give(&sem_per_sync);
}

/**
 * @brief Function for 
 *
 * @param
 */
static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	my_printf("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr);
}

/**
 * @brief Function for 
 *
 * @param
 */
static void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char data_str[129];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	bin2hex(buf->data, buf->len, data_str, sizeof(data_str));

	my_printf("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
	       "RSSI %i, CTE %u, data length %u, data: %s\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power,
	       info->rssi, info->cte_type, buf->len, data_str);
}

/**
 * @brief Function to display Rx value from client
 *
 */
int print_rx_value(void) {
	if (connected_ok) {
		my_printf("\nRx Value = %d\n", rx_val);
	}
	else
		my_printf("\nNo client connected\n");
	return 0;
}

/**
 * @brief Function for 
 *
 * @param
 */
static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		my_printf("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}
	rx_val = ((uint8_t *)data)[0];

	timeout = timercnt;
	return BT_GATT_ITER_CONTINUE;
}

/**
 * @brief Function for 
 *
 * @param
 */
static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		my_printf("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	my_printf("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, SERVICE_UUID)) {

		memcpy(&uuid, BT_UUID_MY_SERVICE_TX, sizeof(uuid));

		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		my_printf("SERVICE UUID OK\n");

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			my_printf("Discover 1 failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_MY_SERVICE_TX)) {
		
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));

		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			my_printf("Discover 2 failed (err %d)\n", err);
		}

		my_printf("SERVICE TX UUID OK\n");
	}
	else {
		subscribe_params.notify = notify_func;
		subscribe_params.value = BT_GATT_CCC_NOTIFY;
		subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params);
		if (err && err != -EALREADY) {
			my_printf("Subscribe failed (err %d)\n", err);
		} else {
			my_printf("[SUBSCRIBED]\n");
			(void)memset(per, 0, BT_ADDR_LE_STR_LEN);
		}
	}
	return BT_GATT_ITER_STOP;
}

/**
 * @brief Print out device info
 *
 *
 */
void ble_info(void) {
	bt_addr_le_t my_addrs[3];
	size_t 	my_addrs_count;
	char dev[BT_ADDR_LE_STR_LEN];

	my_printf("\nPRODUCT      :        %s\n", CONFIG_USB_DEVICE_PRODUCT);
	my_printf("DEVICE NAME  :        %s\n", CONFIG_BT_DEVICE_NAME);
	my_printf("MODEL        :        %s\n", CONFIG_BT_DIS_MODEL);
	my_printf("MANUFACTER   :        %s\n", CONFIG_BT_DIS_MANUF);
	my_printf("SERIAL NUMBER:        %s\n", CONFIG_BT_DIS_SERIAL_NUMBER_STR);

	bt_id_get(NULL, &my_addrs_count);
	if (my_addrs_count>3) my_addrs_count = 3;
	bt_id_get(my_addrs, &my_addrs_count);
	for (size_t i=0; i<my_addrs_count; i++)
	{
		bt_addr_le_to_str(&my_addrs[i], dev, sizeof(dev));
		my_printf("ADDR (%d)     :        %s, type %u\n",
	       i+1, dev, my_addrs[i].type);
	}
	my_printf("\n");
	return 0;
}

/**
 * @brief main task
 *
 *
 */
void main_task(void)
{
	int err;
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	char le_addr[BT_ADDR_LE_STR_LEN];

	set_print_flag((start_console() != 0) ? false : true);

	if (!device_is_ready(led[0].port)) {
		return;
	}

	for (int i=0; i<NUM_OF_LEDS; i++)
	{
		err = gpio_pin_configure_dt(&led[i], GPIO_OUTPUT_ACTIVE);
		if (err < 0) {
			my_printf("gpio configuration error\n");
			return;
		}
	}

	/* Initialize LEDs */
    gpio_pin_set_dt(&led[0], 0);
    gpio_pin_set_dt(&led[1], 0);
    gpio_pin_set_dt(&led[2], 0);
    gpio_pin_set_dt(&led[3], 0);

	leds_initialized = true;

	
	err = dk_buttons_init(button_handler);
	if (err) {
		my_printf("Failed to initialize buttons (err %d)\n", err);
		return;
	}

//	temp_dev = device_get_binding("temp0");
	if (!temp_dev) {
		my_printf("error: no temp device\n");
//		return;
	}

	my_printf("Starting Central Periodic Advertising Synchronization Transfer (PAST)\n");
//	ble_info();

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err != 0) {
		my_printf("failed to enable BT (err %d)\n", err);
		return;
	}

	my_printf("Connection callbacks register\n");
	bt_conn_cb_register(&conn_callbacks);

	my_printf("Scan callbacks register\n");
	bt_le_scan_cb_register(&scan_callbacks);

	my_printf("Periodic Advertising callbacks register\n");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	my_printf("Start scanning...");
	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
	if (err != 0) {
		my_printf("failed (err %d)\n", err);
		return;
	}
	my_printf("success.\n");

	shell_flag = true;

	do {
		my_printf("Waiting for connection...\n");
		err = k_sem_take(&sem_conn, K_FOREVER);
		if (err != 0) {
			my_printf("Could not take sem_conn (err %d)\n", err);
			return;
		}
		my_printf("Connected.\n");

		my_printf("Start scanning for PA...\n");
		per_adv_found = false;
		err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
		if (err != 0) {
			my_printf("failed (err %d)\n", err);
			return;
		}
		my_printf("Scan started.\n");

		my_printf("Waiting for periodic advertising...\n");
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err != 0) {
			my_printf("Could not take sem_per_adv (err %d)\n", err);
			return;
		}
		my_printf("Found periodic advertising.\n");

		bt_addr_le_to_str(&per_addr, le_addr, sizeof(le_addr));
		my_printf("Creating Periodic Advertising Sync to %s...\n", le_addr);
		bt_addr_le_copy(&sync_create_param.addr, &per_addr);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid;
		sync_create_param.skip = 0;
		sync_create_param.timeout = 0xaa;
		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err != 0) {
			my_printf("failed (err %d)\n", err);
			return;
		}
		my_printf("success.\n");
/*
		my_printf("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, K_FOREVER);
		if (err != 0) {
			my_printf("failed (err %d)\n", err);
			return;
		}
		my_printf("Periodic sync established.\n");

		my_printf("Transferring sync\n");
		err = bt_le_per_adv_sync_transfer(sync, default_conn, 0);
		if (err != 0) {
			my_printf("Could not transfer sync (err %d)\n", err);
			return;
		}
*/
		my_printf("Waiting for connection lost...\n");
		err = k_sem_take(&sem_conn_lost, K_FOREVER);
		if (err != 0) {
			my_printf("Could not take sem_conn_lost (err %d)\n", err);
			return;
		}
		my_printf("Connection lost.\n");
	} while (true);
}

/**
 * @brief leds task
 *
 *
 */
void led_task(void) {

	int32_t sleepTime = SLEEP_TIME_MS;

	while (true)
	{
		if (leds_initialized == true) { 		
			if (connected_ok == true)
			{
				sleepTime = (SLEEP_TIME_MS - rx_val)*2;
				gpio_pin_toggle_dt(&led[BLUE_LED]);
				gpio_pin_set_dt(&led[0], 1);
				check_dtr();
			}
			else
			{
				sleepTime = SLEEP_TIME_MS*2;
				gpio_pin_set_dt(&led[BLUE_LED], 0);
				gpio_pin_toggle_dt(&led[0]);
			}
		}
		k_msleep(sleepTime);
		timercnt++;
	}
}

/**
 * @brief user task
 *
 *
 */
void user_task(void) {
	uint8_t buffer[64];
	int rb_len;

	while (true)
	{
		if (chk_print_flag() == true) 
		{
#ifdef DONGLE
			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (rb_len > 0) {
				in_data = buffer[0];
			}
			else
				in_data = -1;
			if (shell_flag == true)
#endif
				run_shell();
		}
		k_msleep(100);
	}
}

/**
 * @brief 
 *
 * 
 */
int nc_getchar(void) {
	return (in_data);
}

K_THREAD_DEFINE(main_id, STACKSIZE, main_task, NULL, NULL, NULL,
		PRIORITY, 0, 0);
K_THREAD_DEFINE(leds_id, STACKSIZE, led_task, NULL, NULL, NULL,
		PRIORITY, 0, 0);
#ifdef DONGLE
K_THREAD_DEFINE(user_id, 2048, user_task, NULL, NULL, NULL,
		PRIORITY, 0, 0);
#endif