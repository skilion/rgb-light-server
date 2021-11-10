#include <host/ble_hs.h>
#include <host/util/util.h>
#include <modlog/modlog.h>
#include <nimble/ble.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <esp_nimble_hci.h>

#include "bl.h"
#include "io.h"
#include "misc.h"


uint32_t rgb_color;
static uint8_t own_addr_type;

void ble_store_config_init(); // not exposed in any header file

static const char *BL_DEVICE_NAME = "RgbLedTest";
static const uint16_t BL_APPEARANCE_CODE = 0x1F9; // Multi-Color LED Array
static const int32_t BL_ADVERTISING_TIMEOUT_MS = BLE_HS_FOREVER;

// 3d09ac99-1200-63ad-b8ab-ff18f6545762
static const ble_uuid128_t BL_GATT_LED_SVC = BLE_UUID128_INIT(
	0x62, 0x57, 0x54, 0xF6, 0x18, 0xFF, 0xAB, 0xB8,
	0xAD, 0x63, 0x00, 0x12, 0x99, 0xAC, 0x09, 0x3D
	);
// 3d09ac99-1201-63ad-b8ab-ff18f6545762
static const ble_uuid128_t BL_GATT_CHR_RGB = BLE_UUID128_INIT(
	0x62, 0x57, 0x54, 0xF6, 0x18, 0xFF, 0xAB, 0xB8,
	0xAD, 0x63, 0x01, 0x12, 0x99, 0xAC, 0x09, 0x3D
	);
static const char BL_GATT_CHR_RGB_USER_DESC[] = "RGB Value";
static const ble_uuid16_t GATT_DSC_CHR_USER_DESC = BLE_UUID16_INIT(0x2901);

static void set_nimble_host_cfg();
static void nimble_host_task(void *param);
static int  on_gatt_attr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static void on_gatt_attr_register(struct ble_gatt_register_ctxt *ctxt, void *arg);
static void on_host_reset(int reason);
static void on_host_sync(void);
static void register_gatt_services();
static int write_characteristic(struct os_mbuf *om, uint16_t min_len, uint16_t max_len, void *dst, uint16_t *out_copy_len);

static const struct ble_gatt_svc_def gatt_services_definition[] =
{
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &BL_GATT_LED_SVC.u,
		.characteristics = (struct ble_gatt_chr_def[])
		{
			{
				.uuid = &BL_GATT_CHR_RGB.u,
				.access_cb = on_gatt_attr_access,
				.flags = BLE_GATT_CHR_F_READ /*| BLE_GATT_CHR_F_READ_ENC*/ | BLE_GATT_CHR_F_WRITE /*| BLE_GATT_CHR_F_WRITE_ENC*/,
				.descriptors = (struct ble_gatt_dsc_def[])
				{
					{
						.uuid = &GATT_DSC_CHR_USER_DESC.u,
						.att_flags = BLE_GATT_CHR_F_READ,
						.access_cb = on_gatt_attr_access,
					},
					{
						0
					}
				}
			},
			{
				0
			}
		},
	},
	{
		0
	},
};


/**
 * Logs information about a connection to the console.
 */
static void
bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
	MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
				desc->conn_handle, desc->our_ota_addr.type);
	print_addr(desc->our_ota_addr.val);
	MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
				desc->our_id_addr.type);
	print_addr(desc->our_id_addr.val);
	MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
				desc->peer_ota_addr.type);
	print_addr(desc->peer_ota_addr.val);
	MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
				desc->peer_id_addr.type);
	print_addr(desc->peer_id_addr.val);
	MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
				"encrypted=%d authenticated=%d bonded=%d\n",
				desc->conn_itvl, desc->conn_latency,
				desc->supervision_timeout,
				desc->sec_state.encrypted,
				desc->sec_state.authenticated,
				desc->sec_state.bonded);
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
	struct ble_gap_conn_desc desc;
	int rc;

	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		/* A new connection was established or a connection attempt failed. */
		MODLOG_DFLT(INFO, "connection %s; status=%d ",
					event->connect.status == 0 ? "established" : "failed",
					event->connect.status);
		if (event->connect.status == 0) {
			rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
			assert(rc == 0);
			bleprph_print_conn_desc(&desc);
		}
		MODLOG_DFLT(INFO, "\n");

		if (event->connect.status != 0) {
			/* Connection failed; resume advertising. */
			bl_advertise();
		}
		return 0;

	case BLE_GAP_EVENT_DISCONNECT:
		MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
		bleprph_print_conn_desc(&event->disconnect.conn);
		MODLOG_DFLT(INFO, "\n");

		/* Connection terminated; resume advertising. */
		bl_advertise();
		return 0;

	case BLE_GAP_EVENT_CONN_UPDATE:
		/* The central has updated the connection parameters. */
		MODLOG_DFLT(INFO, "connection updated; status=%d ",
					event->conn_update.status);
		rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
		assert(rc == 0);
		bleprph_print_conn_desc(&desc);
		MODLOG_DFLT(INFO, "\n");
		return 0;

	case BLE_GAP_EVENT_ADV_COMPLETE:
		MODLOG_DFLT(INFO, "advertise complete; reason=%d",
					event->adv_complete.reason);
		bl_advertise();
		return 0;

	case BLE_GAP_EVENT_ENC_CHANGE:
		/* Encryption has been enabled or disabled for this connection. */
		MODLOG_DFLT(INFO, "encryption change event; status=%d ",
					event->enc_change.status);
		rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
		assert(rc == 0);
		bleprph_print_conn_desc(&desc);
		MODLOG_DFLT(INFO, "\n");
		return 0;

	case BLE_GAP_EVENT_SUBSCRIBE:
		MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
					"reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
					event->subscribe.conn_handle,
					event->subscribe.attr_handle,
					event->subscribe.reason,
					event->subscribe.prev_notify,
					event->subscribe.cur_notify,
					event->subscribe.prev_indicate,
					event->subscribe.cur_indicate);
		return 0;

	case BLE_GAP_EVENT_MTU:
		MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
					event->mtu.conn_handle,
					event->mtu.channel_id,
					event->mtu.value);
		return 0;

	case BLE_GAP_EVENT_REPEAT_PAIRING:
		/* We already have a bond with the peer, but it is attempting to
		 * establish a new secure link.  This app sacrifices security for
		 * convenience: just throw away the old bond and accept the new link.
		 */

		/* Delete the old bond. */
		rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
		assert(rc == 0);
		ble_store_util_delete_peer(&desc.peer_id_addr);

		/* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
		 * continue with the pairing operation.
		 */
		return BLE_GAP_REPEAT_PAIRING_RETRY;
	}

	return 0;
}

static int on_gatt_attr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	int rc;

	switch (ctxt->op)
	{
	case BLE_GATT_ACCESS_OP_READ_DSC:
		assert(ble_uuid_cmp(ctxt->dsc->uuid, &GATT_DSC_CHR_USER_DESC.u) == 0);
		MODLOG_DFLT(INFO, "BLE_GATT_ACCESS_OP_READ_DSC");
		rc = os_mbuf_append(ctxt->om, &BL_GATT_CHR_RGB_USER_DESC, strlen(BL_GATT_CHR_RGB_USER_DESC));
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

	case BLE_GATT_ACCESS_OP_READ_CHR:
		assert(ble_uuid_cmp(ctxt->chr->uuid, &BL_GATT_CHR_RGB.u) == 0);
		MODLOG_DFLT(INFO, "BLE_GATT_ACCESS_OP_READ_CHR");
		rc = os_mbuf_append(ctxt->om, &rgb_color, sizeof rgb_color);
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

	case BLE_GATT_ACCESS_OP_WRITE_CHR:
		assert(ble_uuid_cmp(ctxt->chr->uuid, &BL_GATT_CHR_RGB.u) == 0);
		MODLOG_DFLT(INFO, "BLE_GATT_ACCESS_OP_WRITE_CHR");
		rc = write_characteristic(ctxt->om,	sizeof rgb_color, sizeof rgb_color,	&rgb_color,	NULL);
		if (rc == 0) {
			io_set_rgb(rgb_color);
		}
		return rc;
	}

	assert(0);
	return BLE_ATT_ERR_UNLIKELY;
}


static int write_characteristic(struct os_mbuf *om, uint16_t min_len, uint16_t max_len, void *dst, uint16_t *out_copy_len)
{
	uint16_t om_len;
	int rc;

	om_len = OS_MBUF_PKTLEN(om);
	if (om_len < min_len || om_len > max_len) {
		return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
	}

	rc = ble_hs_mbuf_to_flat(om, dst, max_len, out_copy_len);
	if (rc != 0) {
		return BLE_ATT_ERR_UNLIKELY;
	}

	return 0;
}

/**
 * Initializes the Bluetooth stack
 */
void bl_init()
{
	ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());
	nimble_port_init();
	ble_store_config_init();
	set_nimble_host_cfg();
	ble_svc_gap_init();
	ble_svc_gatt_init();

	/* Set the default device name. */
	int rc = ble_svc_gap_device_name_set(BL_DEVICE_NAME);
	assert(rc == 0);
	ble_svc_gap_device_appearance_set(BL_APPEARANCE_CODE);

	register_gatt_services();
	nimble_port_freertos_init(nimble_host_task);
}

static void set_nimble_host_cfg()
{
	ble_hs_cfg.reset_cb = on_host_reset;
	ble_hs_cfg.sync_cb = on_host_sync;
	ble_hs_cfg.gatts_register_cb = on_gatt_attr_register;
	//ble_hs_cfg.store_status_cb = ble_store_util_status_rr; // TODO

	ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;

	ble_hs_cfg.sm_bonding = 1;
	ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
	ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

	ble_hs_cfg.sm_sc = 0;
}

static void register_gatt_services()
{
	int rc = ble_gatts_count_cfg(gatt_services_definition);
	assert(rc == 0);
	rc = ble_gatts_add_svcs(gatt_services_definition);
	assert(rc == 0);
}

static void nimble_host_task(void *param)
{
	nimble_port_run(); // This function will return only when nimble_port_stop() is executed
	nimble_port_freertos_deinit();
}

static void on_host_reset(int reason)
{
	MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void on_host_sync(void)
{
	int rc;

	rc = ble_hs_util_ensure_addr(0);
	assert(rc == 0);

	/* Figure out address to use while advertising (no privacy for now) */
	rc = ble_hs_id_infer_auto(0, &own_addr_type);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
		return;
	}

	uint8_t addr_val[6] = {0};
	rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

	MODLOG_DFLT(INFO, "Device Address: ");
	print_addr(addr_val);

	bl_advertise();
}

static void on_gatt_attr_register(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
	char buf[BLE_UUID_STR_LEN];

	switch (ctxt->op)
	{
	case BLE_GATT_REGISTER_OP_SVC:
		MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
					ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
					ctxt->svc.handle);
		break;

	case BLE_GATT_REGISTER_OP_CHR:
		MODLOG_DFLT(DEBUG, "registering characteristic %s with "
					"def_handle=%d val_handle=%d\n",
					ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
					ctxt->chr.def_handle,
					ctxt->chr.val_handle);
		break;

	case BLE_GATT_REGISTER_OP_DSC:
		MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
					ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
					ctxt->dsc.handle);
		break;

	default:
		assert(0);
		break;
	}
}

/**
 * Makes the device send advertising beacons
 */
void bl_advertise()
{
	struct ble_gap_adv_params adv_params;
	struct ble_hs_adv_fields fields = {0};
	const char *name;
	int rc;


	/* Advertise two flags:
	 *     o Discoverability in forthcoming advertisement (general)
	 *     o BLE-only (BR/EDR unsupported).
	 */
	fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

	/* Indicate that the TX power level field should be included; have the
	 * stack fill this value automatically.  This is done by assigning the
	 * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
	 */
	fields.tx_pwr_lvl_is_present = 1;
	fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

	name = ble_svc_gap_device_name();
	fields.name = (uint8_t *)name;
	fields.name_len = strlen(name);
	fields.name_is_complete = 1;

	fields.appearance = ble_svc_gap_device_appearance();
	fields.appearance_is_present = 1;

	rc = ble_gap_adv_set_fields(&fields);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
		return;
	}

	/* Begin advertising. */
	memset(&adv_params, 0, sizeof adv_params);
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	rc = ble_gap_adv_start(own_addr_type, NULL, BL_ADVERTISING_TIMEOUT_MS,
						   &adv_params, bleprph_gap_event, NULL);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
		return;
	}
}