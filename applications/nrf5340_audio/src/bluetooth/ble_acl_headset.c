// /*
//  * Copyright (c) 2018 Nordic Semiconductor ASA
//  *
//  * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
//  */

// #include "ble_acl_headset.h"

// #include <zephyr.h>
// #include <bluetooth/hci.h>
// #include <bluetooth/conn.h>

// #include "macros_common.h"
// #include "board.h"
// #include "ble_acl_common.h"
// #include "channel_assignment.h"

// #include <logging/log.h>
// LOG_MODULE_DECLARE(ble, CONFIG_LOG_BLE_LEVEL);

// #define BT_LE_ADV_FAST_CONN                                                                        \
// 	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_1,                      \
// 			BT_GAP_ADV_FAST_INT_MAX_1, NULL)

// /* Advertising data for peer connection */
// static const struct bt_data ad_peer_l[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME_PEER_L, DEVICE_NAME_PEER_L_LEN),
// };

// static const struct bt_data ad_peer_r[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME_PEER_R, DEVICE_NAME_PEER_R_LEN),
// };

// /* Connection to the gateway device - the other nRF5340 Audio device
//  * This is the device we are streaming audio to/from.
//  */
// static struct bt_conn *headset_conn_peer;

// void work_adv_start(struct k_work *item)
// {
// 	enum audio_channel channel;
// 	int ret;

// 	ret = channel_assignment_get(&channel);
// 	if (ret) {
// 		/* Channel is not assigned yet: use default */
// 		channel = AUDIO_CHANNEL_DEFAULT;
// 	}

// 	if (channel != AUDIO_CHANNEL_RIGHT) {
// 		/* If anything else than right, default to left */
// 		ret = bt_le_adv_start(BT_LE_ADV_FAST_CONN, ad_peer_l, ARRAY_SIZE(ad_peer_l), NULL,
// 				      0);
// 	} else {
// 		ret = bt_le_adv_start(BT_LE_ADV_FAST_CONN, ad_peer_r, ARRAY_SIZE(ad_peer_r), NULL,
// 				      0);
// 	}

// 	if (ret) {
// 		LOG_ERR("Advertising failed to start (ret %d)", ret);
// 	}
// }

// void ble_acl_headset_on_connected(struct bt_conn *conn)
// {
// 	LOG_DBG("Connected - nRF5340 Audio headset");
// 	headset_conn_peer = bt_conn_ref(conn);
// 	ble_acl_common_conn_peer_set(headset_conn_peer);
// }

//============================================================================================================




/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "ble_acl_headset.h"
#include "ble_acl_common.h"
#include "ble_audio_services.h"

#include <zephyr.h>
#include <bluetooth/hci.h>
#include <bluetooth/gatt.h>

#include "macros_common.h"
#include "ble_acl_common.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(ble, CONFIG_LOG_BLE_LEVEL);

K_WORK_DEFINE(start_scan_work, work_scan_start);

#define BT_LE_CONN_PARAM_MULTI                                                                     \
	BT_LE_CONN_PARAM(CONFIG_BLE_ACL_CONN_INTERVAL, CONFIG_BLE_ACL_CONN_INTERVAL,               \
			 CONFIG_BLE_ACL_SLAVE_LATENCY, CONFIG_BLE_ACL_SUP_TIMEOUT)

#define BT_LE_ADV_FAST_CONN                                                                        \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_1,                      \
			BT_GAP_ADV_FAST_INT_MAX_1, NULL)


/* Advertising data */
static const struct bt_data ad_data_l[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME_PEER_L, DEVICE_NAME_PEER_L_LEN),
};


static struct bt_gatt_exchange_params exchange_params;

/* Connection to the headset device - the other nRF5340 Audio device
 * This is the device we are streaming audio to/from.
 */
#define MAX_CONNECTED_PEERS 1
static struct bt_conn *headset_conn_peer[MAX_CONNECTED_PEERS];

/** @brief BLE data stream device found handler.
 *
 * This function is called as part of the processing of a device found
 * during scanning (i.e. as a result of on_device_found() being called).
 *
 * This is done by checking whether the device name is the peer
 * connection device name.
 *
 * If so, this function will stop scanning and try to create
 * a connection to the peer.
 */
static int device_found(uint8_t type, const uint8_t *data, uint8_t data_len,
			const bt_addr_le_t *addr)
{
	int ret;
	struct bt_conn *conn;
	char addr_str[BT_ADDR_LE_STR_LEN];

	if (ble_acl_headset_all_links_connected()) {
		return 0;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	if ((data_len == DEVICE_NAME_GATEWAY_LEN) &&
	    (strncmp(DEVICE_NAME_GATEWAY, data, DEVICE_NAME_GATEWAY_LEN) == 0)) {
		bt_le_scan_stop();

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_MULTI,
					&conn);
		if (ret) {
			LOG_ERR("Could not init connection");
			return ret;
		}

		ret = ble_acl_headset_conn_peer_set(0, &conn);
		ERR_CHK_MSG(ret, "Connection peer set error");

		return 0;
	}

	return -ENOENT;
}

/** @brief  BLE parse advertisement package.
 */
static void ad_parse(struct net_buf_simple *p_ad, const bt_addr_le_t *addr)
{
	while (p_ad->len > 1) {
		uint8_t len = net_buf_simple_pull_u8(p_ad);
		uint8_t type;

		/* Check for early termination */
		if (len == 0) {
			return;
		}

		if (len > p_ad->len) {
			LOG_ERR("AD malformed");
			return;
		}

		type = net_buf_simple_pull_u8(p_ad);

		if (device_found(type, p_ad->data, len - 1, addr) == 0) {
			return;
		}

		(void)net_buf_simple_pull(p_ad, len - 1);
	}
}

/** @brief Handle devices found during scanning
 *
 * (Will only be called for peer connections, as we do not
 *  scan for control connections.)
 */
static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			    struct net_buf_simple *p_ad)
{
	/* We're only interested in general connectable events */
	if (type == BT_HCI_ADV_IND) {
		/* Note: May lead to connection creation */
		ad_parse(p_ad, addr);
	}
}

/** @brief Callback handler for GATT exchange MTU procedure
 *
 *  This handler will be triggered after MTU exchange procedure finished.
 */
static void mtu_exchange_handler(struct bt_conn *conn, uint8_t err,
				 struct bt_gatt_exchange_params *params)
{
	int ret;
	struct bt_conn *conn_active;

	if (err) {
		LOG_ERR("MTU exchange failed, err = %d", err);
		ret = bt_conn_disconnect(conn, BT_HCI_ERR_LOCALHOST_TERM_CONN);
		if (ret) {
			LOG_ERR("Failed to disconnected, %d", ret);
		}
	} else {
		LOG_DBG("MTU exchange success");
		if (!ble_acl_headset_all_links_connected()) {
			k_work_submit(&start_scan_work);
		} else {
			LOG_INF("All ACL links are connected");
			bt_le_scan_stop();
		}
		
		ret = ble_trans_iso_cis_connect(conn);
		ERR_CHK_MSG(ret, "Failed to connect to ISO CIS channel");
	}
}

int ble_acl_headset_conn_peer_get(uint8_t chan_number, struct bt_conn **p_conn)
{
	if (chan_number >= MAX_CONNECTED_PEERS) {
		return -EINVAL;
	}
	*p_conn = headset_conn_peer[chan_number];
	return 0;
}

int ble_acl_headset_conn_peer_set(uint8_t chan_number, struct bt_conn **p_conn)
{
	if (chan_number >= MAX_CONNECTED_PEERS) {
		return -EINVAL;
	}

	if (headset_conn_peer[chan_number] != NULL) {
		if (*p_conn == NULL) {
			headset_conn_peer[chan_number] = NULL;
		} else {
			LOG_WRN("Already have a connection for peer: %d", chan_number);
		}
		/* Ignore duplicates as several peripherals might be
		 * advertising at the same time
		 */
		return 0;
	}

	headset_conn_peer[chan_number] = *p_conn;
	return 0;
}

bool ble_acl_headset_all_links_connected(void)
{
	for (int i = 0; i < MAX_CONNECTED_PEERS; i++) {
		if (headset_conn_peer[i] == NULL) {
			return false;
		}
	}
	return true;
}

void work_scan_start(struct k_work *item)
{
	int ret;

	ret = bt_le_scan_start(BT_LE_SCAN_PASSIVE, on_device_found);
	if (ret) {
		LOG_WRN("Scanning failed to start (ret %d)", ret);
		return;
	}

	LOG_DBG("Scanning successfully started");
}

void work_headset_adv_start(struct k_work *item)
{
	int ret = bt_le_adv_start(BT_LE_ADV_FAST_CONN, ad_data_l, ARRAY_SIZE(ad_data_l), NULL,
				      0);

	if (ret) {
		LOG_ERR("Advertising failed to start (ret %d)", ret);
	}
}


void ble_acl_headset_on_connected(struct bt_conn *conn)
{
	LOG_DBG("Connected - nRF5340 Audio headset");
}

int ble_acl_headset_mtu_exchange(struct bt_conn *conn)
{
	exchange_params.func = mtu_exchange_handler;

	return bt_gatt_exchange_mtu(conn, &exchange_params);
}
