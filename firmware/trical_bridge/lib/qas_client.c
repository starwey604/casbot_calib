#include "qas_client.h"

#include <errno.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/atomic.h>

enum stage { OFF, CONNECTING, SERVICE16, SERVICE128, CHARACTERISTIC, CCC,
	SUBSCRIBING, READY, CLOSING };

struct client_slot {
	struct bt_conn *conn;
	bt_addr_le_t address;
	enum stage stage;
	uint32_t epoch;
	uint64_t deadline_us, quiet_since_us;
	struct bt_gatt_discover_params discovery;
	struct bt_gatt_subscribe_params subscription;
	struct bt_gatt_write_params write;
	uint16_t service_end, value_handle, char_end;
	uint8_t properties;
	bool busy, request, variant128;
	int mod;
	uint32_t command_id;
	char command[QAS_COMMAND_MAX];
};

static struct client_slot slots[QAS_SLOTS];
K_MSGQ_DEFINE(events, sizeof(struct qas_event), 96, 8);
static atomic_t sequence, drops, critical_loss;
static bool terminal, scanning;
static uint64_t scan_end_us;
static const struct bt_uuid_16 service16 = BT_UUID_INIT_16(0xffb0);
static const struct bt_uuid_16 data16 = BT_UUID_INIT_16(0xffb2);
static const struct bt_uuid_128 service128 = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0783b03e, 0x8535, 0xb5a0, 0x7140, 0xa304d249ffb0));
static const struct bt_uuid_128 data128 = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x0783b03e, 0x8535, 0xb5a0, 0x7140, 0xa304d249ffb2));

uint64_t bridge_now_us(void) { return k_ticks_to_us_floor64(k_uptime_ticks()); }

static struct qas_event event_for(struct client_slot *s, enum qas_event_type type)
{
	struct qas_event e = { .type = type, .time_us = bridge_now_us(), .slot = 255 };
	if (s != NULL) {
		e.slot = (uint8_t)(s - slots); e.epoch = s->epoch;
	}
	return e;
}

static void emit(struct qas_event *event)
{
	event->sequence = (uint32_t)atomic_inc(&sequence);
	if (k_msgq_put(&events, event, K_NO_WAIT) != 0) {
		atomic_inc(&drops);
		if (event->type != QAS_NOTIFY && event->type != QAS_ADVERTISEMENT) {
			atomic_set(&critical_loss, 1);
		}
	}
}

static struct client_slot *find_slot(struct bt_conn *conn)
{
	for (size_t i = 0; i < QAS_SLOTS; i++) {
		if (slots[i].conn == conn) { return &slots[i]; }
	}
	return NULL;
}

static void fail(struct client_slot *s, int error)
{
	struct qas_event e = event_for(s, QAS_ERROR);
	e.result = error; emit(&e);
	(void)qas_disconnect((unsigned int)(s - slots));
}

static uint8_t discovered(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  struct bt_gatt_discover_params *params)
{
	struct client_slot *s = CONTAINER_OF(params, struct client_slot, discovery);
	struct qas_event e = event_for(s, QAS_DISCOVERED);
	ARG_UNUSED(conn);
	if (s->stage == CHARACTERISTIC) {
		if (attr != NULL) {
			const struct bt_gatt_chrc *ch = attr->user_data;
			if (s->value_handle != 0) {
				s->char_end = attr->handle - 1;
			} else if (!bt_uuid_cmp(ch->uuid, s->variant128 ? &data128.uuid : &data16.uuid)) {
				s->value_handle = ch->value_handle; s->properties = ch->properties;
				return BT_GATT_ITER_CONTINUE;
			} else { return BT_GATT_ITER_CONTINUE; }
		}
		e.handle = s->value_handle; e.end_handle = s->char_end; e.properties = s->properties;
		e.result = e.handle == 0 ? -ENOENT : 0;
	} else if (attr == NULL) { e.result = -ENOENT; }
	else if (s->stage == SERVICE16 || s->stage == SERVICE128) {
		const struct bt_gatt_service_val *service = attr->user_data;
		e.handle = attr->handle; e.end_handle = service->end_handle;
	} else if (s->stage == CCC) { e.handle = attr->handle; }
	else { return BT_GATT_ITER_STOP; }
	emit(&e);
	return BT_GATT_ITER_STOP;
}

static void subscribed(struct bt_conn *conn, uint8_t err,
		       struct bt_gatt_subscribe_params *params)
{
	struct client_slot *s = CONTAINER_OF(params, struct client_slot, subscription);
	struct qas_event e = event_for(s, QAS_SUBSCRIBED);
	ARG_UNUSED(conn);
	e.result = err;
	e.handle = params->value_handle;
	e.properties = s->properties;
	emit(&e);
}

static uint8_t notified(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			const void *data, uint16_t length)
{
	struct client_slot *s = CONTAINER_OF(params, struct client_slot, subscription);
	struct qas_event e = event_for(s, data == NULL ? QAS_ERROR : QAS_NOTIFY);
	ARG_UNUSED(conn);
	if (data == NULL) {
		e.result = -ENOTCONN; emit(&e); return BT_GATT_ITER_STOP;
	}
	e.length = length;
	if (length > sizeof(e.data)) { atomic_inc(&drops); }
	memcpy(e.data, data, MIN(length, sizeof(e.data)));
	emit(&e);
	return BT_GATT_ITER_CONTINUE;
}

static void write_complete(struct client_slot *s, int err)
{
	struct qas_event e = event_for(s, QAS_WRITE_DONE);
	e.command_id = s->command_id; e.result = err; e.request = s->request;
	emit(&e);
}

static void command_sent(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(conn); write_complete(user_data, 0);
}

static void request_sent(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
	ARG_UNUSED(conn);
	write_complete(CONTAINER_OF(params, struct client_slot, write), err);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	struct client_slot *s = find_slot(conn);
	if (s == NULL) { return; }
	struct qas_event e = event_for(s, QAS_CONNECTED);
	e.result = err; emit(&e);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct client_slot *s = find_slot(conn);
	if (s == NULL) { return; }
	struct qas_event e = event_for(s, QAS_DISCONNECTED);
	e.result = reason; emit(&e);
}

static void parameters(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	struct client_slot *s = find_slot(conn);
	if (s == NULL) { return; }
	struct qas_event e = event_for(s, QAS_LINK_PARAMS);
	e.interval = interval; e.latency = latency; e.timeout = timeout;
	emit(&e);
}

BT_CONN_CB_DEFINE(connection_callbacks) = {
	.connected = connected, .disconnected = disconnected, .le_param_updated = parameters,
};

static void advertisement(const bt_addr_le_t *address, int8_t rssi, uint8_t type,
			  struct net_buf_simple *ad)
{
	struct qas_event e = event_for(NULL, QAS_ADVERTISEMENT);
	bt_addr_le_to_str(address, e.address, sizeof(e.address));
	e.rssi = rssi; e.properties = type; e.length = ad->len;
	memcpy(e.data, ad->data, MIN(ad->len, sizeof(e.data)));
	emit(&e);
}

int qas_client_init(void)
{
	for (size_t i = 0; i < QAS_SLOTS; i++) { slots[i].mod = -1; }
	return bt_enable(NULL);
}

int qas_scan(uint32_t duration_ms)
{
	if (terminal) { return -EIO; }
	if (scanning) { return -EBUSY; }
	if (duration_ms < 100 || duration_ms > 30000) { return -EINVAL; }
	int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, advertisement);
	if (err == 0) { scanning = true; scan_end_us = bridge_now_us() + duration_ms * 1000ULL; }
	return err;
}

static int start_discovery(struct client_slot *s, enum stage stage,
			   const struct bt_uuid *uuid, uint16_t start, uint16_t end)
{
	s->stage = stage;
	s->discovery = (struct bt_gatt_discover_params) {
		.uuid = uuid, .func = discovered, .start_handle = start, .end_handle = end,
		.type = stage == CHARACTERISTIC ? BT_GATT_DISCOVER_CHARACTERISTIC :
			(stage == CCC ? BT_GATT_DISCOVER_DESCRIPTOR : BT_GATT_DISCOVER_PRIMARY),
	};
	s->deadline_us = bridge_now_us() + 10000000;
	return bt_gatt_discover(s->conn, &s->discovery);
}

int qas_connect(unsigned int slot, const char *address, const char *type)
{
	if (terminal) { return -EIO; }
	if (slot >= QAS_SLOTS) { return -EINVAL; }
	struct client_slot *s = &slots[slot];
	if (s->conn != NULL || scanning) { return -EBUSY; }
	bt_addr_le_t peer;
	if (strcmp(type, "public") != 0 && strcmp(type, "random") != 0) { return -EINVAL; }
	peer.type = strcmp(type, "public") == 0 ? BT_ADDR_LE_PUBLIC : BT_ADDR_LE_RANDOM;
	int err = bt_addr_from_str(address, &peer.a);
	if (err != 0) { return err; }
	for (size_t i = 0; i < QAS_SLOTS; i++) {
		if (slots[i].conn != NULL && bt_addr_le_cmp(&slots[i].address, &peer) == 0) {
			return -EALREADY;
		}
	}
	uint32_t epoch = s->epoch + 1;
	memset(s, 0, sizeof(*s));
	s->epoch = epoch; s->mod = -1; s->address = peer; s->stage = CONNECTING;
	s->deadline_us = bridge_now_us() + 10000000;
	err = bt_conn_le_create(&peer, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &s->conn);
	if (err != 0) { s->stage = OFF; }
	return err;
}

int qas_disconnect(unsigned int slot)
{
	if (slot >= QAS_SLOTS) { return -EINVAL; }
	struct client_slot *s = &slots[slot];
	if (s->conn == NULL) { return -ENOTCONN; }
	s->stage = CLOSING;
	return bt_conn_disconnect(s->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

int qas_send(unsigned int slot, uint32_t id, const char *command, bool request)
{
	if (terminal) { return -EIO; }
	if (!qas_command_valid(command) || slot >= QAS_SLOTS) { return -EINVAL; }
	struct client_slot *s = &slots[slot];
	if (s->stage != READY) { return -ENOTCONN; }
	if (s->busy) { return -EBUSY; }
	uint8_t capability = request ? BT_GATT_CHRC_WRITE : BT_GATT_CHRC_WRITE_WITHOUT_RESP;
	if ((s->properties & capability) == 0) { return -ENOTSUP; }
	size_t length = strlen(command);
	if (length > bt_gatt_get_mtu(s->conn) - 3U) { return -EMSGSIZE; }
	strcpy(s->command, command);
	s->busy = true; s->command_id = id; s->request = request;
	if (strncmp(command, "MOD:", 4) == 0) { s->mod = -1; }
	struct qas_event e = event_for(s, QAS_WRITE);
	e.command_id = id; e.request = request; e.length = length;
	if (length > sizeof(e.data)) { atomic_inc(&drops); }
	memcpy(e.data, command, length);
	int err;
	if (request) {
		s->write = (struct bt_gatt_write_params) {
			.func = request_sent, .handle = s->value_handle,
			.data = s->command, .length = length,
		};
		err = bt_gatt_write(s->conn, &s->write);
	} else {
		err = bt_gatt_write_without_response_cb(s->conn, s->value_handle, s->command,
			length, false, command_sent, s);
	}
	if (err != 0) { s->busy = false; }
	e.result = err; emit(&e);
	return err;
}

bool qas_next_event(struct qas_event *event) { return k_msgq_get(&events, event, K_NO_WAIT) == 0; }

void qas_process(const struct qas_event *e)
{
	if (e->slot >= QAS_SLOTS) { return; }
	struct client_slot *s = &slots[e->slot];
	if (s->epoch != e->epoch) { return; }
	int err = 0;
	if (e->type == QAS_DISCONNECTED || (e->type == QAS_CONNECTED && e->result != 0)) {
		if (s->conn != NULL) { bt_conn_unref(s->conn); s->conn = NULL; }
		s->stage = OFF; s->busy = false; s->mod = -1;
		return;
	}
	if (e->type == QAS_WRITE_DONE && e->command_id == s->command_id) { s->busy = false; }
	if (e->type == QAS_NOTIFY) {
		struct qas_frame f = qas_parse(e->data, e->length);
		if (f.kind == QAS_DATA) { s->quiet_since_us = e->time_us; }
		if (qas_parameter_is(e->data, e->length, "MOD", "0")) { s->mod = 0; }
		else if (qas_parameter_is(e->data, e->length, "MOD", "1")) { s->mod = 1; }
		return;
	}
	if (e->type == QAS_ERROR) { (void)qas_disconnect(e->slot); return; }
	if (s->stage == CLOSING || s->stage == OFF) { return; }
	if (e->type == QAS_CONNECTED) {
		err = start_discovery(s, SERVICE16, &service16.uuid, 1, 0xffff);
	} else if (e->type == QAS_DISCOVERED) {
		if (e->result != 0) {
			if (s->stage == SERVICE16) {
				s->variant128 = true;
				err = start_discovery(s, SERVICE128, &service128.uuid, 1, 0xffff);
			} else { err = e->result; }
		} else if (s->stage == SERVICE16 || s->stage == SERVICE128) {
			s->service_end = e->end_handle; s->char_end = e->end_handle;
			err = start_discovery(s, CHARACTERISTIC, NULL, e->handle + 1, e->end_handle);
		} else if (s->stage == CHARACTERISTIC) {
			if ((e->properties & BT_GATT_CHRC_NOTIFY) == 0 ||
			    (e->properties & (BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP)) == 0 ||
			    e->handle >= e->end_handle) { err = -ENOTSUP; }
			else { err = start_discovery(s, CCC, BT_UUID_GATT_CCC, e->handle + 1, e->end_handle); }
		} else if (s->stage == CCC) {
			s->stage = SUBSCRIBING;
			s->subscription = (struct bt_gatt_subscribe_params) {
				.notify = notified, .subscribe = subscribed,
				.value_handle = s->value_handle, .ccc_handle = e->handle,
				.value = BT_GATT_CCC_NOTIFY,
			};
			atomic_set_bit(s->subscription.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);
			err = bt_gatt_subscribe(s->conn, &s->subscription);
		}
	} else if (e->type == QAS_SUBSCRIBED) {
		if (e->result != 0) { err = -EIO; }
		else {
			s->stage = READY; s->quiet_since_us = e->time_us;
			struct bt_conn_info info;
			if (bt_conn_get_info(s->conn, &info) == 0) {
				parameters(s->conn, (uint16_t)(info.le.interval_us / 1250U), info.le.latency, info.le.timeout);
			}
		}
	}
	if (err != 0) { fail(s, err); }
}

void qas_tick(void)
{
	uint64_t now = bridge_now_us();
	if (scanning && (now >= scan_end_us || terminal)) {
		int err = bt_le_scan_stop(); scanning = false;
		struct qas_event e = event_for(NULL, QAS_SCAN_STOPPED); e.result = err; emit(&e);
	}
	if (!terminal && atomic_get(&critical_loss)) {
		terminal = true;
		for (size_t i = 0; i < QAS_SLOTS; i++) { (void)qas_disconnect(i); }
	}
	for (size_t i = 0; i < QAS_SLOTS; i++) {
		struct client_slot *s = &slots[i];
		if (s->conn != NULL && s->stage != READY && s->stage != CLOSING && now > s->deadline_us) {
			fail(s, -ETIMEDOUT);
		}
	}
}

bool qas_ready(unsigned int slot) { return slot < QAS_SLOTS && slots[slot].stage == READY; }
int qas_mod(unsigned int slot) { return slot < QAS_SLOTS ? slots[slot].mod : -1; }
uint32_t qas_epoch(unsigned int slot) { return slot < QAS_SLOTS ? slots[slot].epoch : 0; }
const char *qas_state(unsigned int slot)
{
	static const char *const names[] = {"off", "connecting", "service16", "service128",
		"characteristic", "ccc", "subscribing", "ready", "closing"};
	return slot < QAS_SLOTS ? names[slots[slot].stage] : "invalid";
}
uint32_t qas_event_drops(void) { return (uint32_t)atomic_get(&drops); }
bool qas_terminal(void) { return terminal; }

bool qas_idle(unsigned int slot) { return qas_ready(slot) && !slots[slot].busy; }
bool qas_write_supported(unsigned int slot, bool request)
{
	return qas_ready(slot) && (slots[slot].properties &
		(request ? BT_GATT_CHRC_WRITE : BT_GATT_CHRC_WRITE_WITHOUT_RESP)) != 0;
}
uint64_t qas_quiet_since(unsigned int slot) { return slot < QAS_SLOTS ? slots[slot].quiet_since_us : 0; }
bool qas_events_pending(void) { return k_msgq_num_used_get(&events) != 0; }
