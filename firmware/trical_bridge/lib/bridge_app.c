#include "bridge_app.h"
#include "qas_client.h"
#include "rate_core.h"
#include "usb_transport.h"

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trical_bridge, LOG_LEVEL_INF);

static struct {
	bool enabled, active, available, request;
	unsigned int slot;
	uint32_t run, epoch, last_command_id;
	uint32_t drop_events, drop_tx, drop_rx, end_events, end_tx, end_rx;
	const char *reason;
	struct rate_core core;
} bench;
static uint32_t generated_id = 0x80000000U;

static void hex_encode(const uint8_t *data, size_t length, char *hex)
{
	static const char digits[] = "0123456789abcdef";
	for (size_t i = 0; i < length; i++) {
		hex[2 * i] = digits[data[i] >> 4]; hex[2 * i + 1] = digits[data[i] & 15];
	}
	hex[2 * length] = '\0';
}

static void status(uint32_t id)
{
	bridge_usb_json("{\"schema\":1,\"type\":\"status\",\"id\":%u,\"app\":\"%s\","
		"\"clock\":\"bridge_uptime_us\",\"ticks_per_second\":%u,\"now_us\":%" PRIu64 ","
		"\"states\":[\"%s\",\"%s\",\"%s\"],\"epochs\":[%u,%u,%u],"
		"\"event_drops\":%u,\"usb_tx_drops\":%u,\"usb_rx_drops\":%u,\"terminal\":%s}",
		id, bench.enabled ? "qas_rate" : "trical_bridge", CONFIG_SYS_CLOCK_TICKS_PER_SEC,
		bridge_now_us(), qas_state(0), qas_state(1), qas_state(2),
		qas_epoch(0), qas_epoch(1), qas_epoch(2), qas_event_drops(),
		bridge_usb_tx_drops(), bridge_usb_rx_drops(), qas_terminal() ? "true" : "false");
}

static void report(void)
{
	if (!bench.available) { return; }
	struct rate_core *r = &bench.core;
	bool complete = strcmp(bench.reason, "finished") == 0;
	uint32_t events = bench.active ? qas_event_drops() : bench.end_events;
	uint32_t tx = bench.active ? bridge_usb_tx_drops() : bench.end_tx;
	uint32_t rx = bench.active ? bridge_usb_rx_drops() : bench.end_rx;
	bool raw_complete = events == bench.drop_events && tx == bench.drop_tx && rx == bench.drop_rx;
	int emitted = bridge_usb_json("{\"schema\":1,\"type\":\"rate_report\",\"run\":%u,\"slot\":%u,\"epoch\":%u,"
		"\"reason\":\"%s\",\"window_complete\":%s,\"raw_complete\":%s,"
		"\"start_us\":%" PRIu64 ",\"stop_us\":%" PRIu64 ",\"end_us\":%" PRIu64 ","
		"\"interval_us\":%" PRIu64 ",\"write_mode\":\"%s\",\"scheduled\":%u,\"skipped\":%u,"
		"\"submitted\":%u,\"busy\":%u,\"errors\":%u,\"done\":%u,\"done_errors\":%u,\"pending\":%u,"
		"\"max_late_us\":%" PRIu64 ",\"dat0\":%u,\"dat1\":%u,\"tail_data\":%u,"
		"\"meas_ack\":%u,\"malformed\":%u,\"other\":%u,"
		"\"event_drops\":%u,\"usb_tx_drops\":%u,\"usb_rx_drops\":%u}",
		bench.run, bench.slot, bench.epoch, bench.reason, complete ? "true" : "false",
		raw_complete ? "true" : "false", r->start_us, r->stop_us, r->end_us, r->interval_us,
		bench.request ? "wreq" : "wcmd", r->slots, r->skipped, r->submitted, r->busy,
		r->errors, r->done, r->done_errors, r->pending, r->max_late_us,
		r->data[0].count, r->data[1].count, r->tail_data, r->ack, r->malformed, r->other,
		events - bench.drop_events, tx - bench.drop_tx, rx - bench.drop_rx);
	if (emitted != 0 && !bench.active) { bench.end_tx = bridge_usb_tx_drops(); }
}

static void stop_bench(const char *reason)
{
	if (bench.active) {
		bench.end_events = qas_event_drops(); bench.end_tx = bridge_usb_tx_drops();
		bench.end_rx = bridge_usb_rx_drops();
		bench.active = false; bench.reason = reason; report();
	}
}

static void event_output(const struct qas_event *e)
{
	static const char *const types[] = {"connected", "disconnected", "discovered", "subscribed",
		"notify", "write", "write_done", "advertisement", "link_params", "error", "scan_stopped"};
	char hex[QAS_PAYLOAD_MAX * 2 + 1];
	hex_encode(e->data, MIN(e->length, sizeof(e->data)), hex);
	struct qas_frame f = e->type == QAS_NOTIFY ? qas_parse(e->data, e->length) : (struct qas_frame){0};
	bridge_usb_json("{\"schema\":1,\"type\":\"%s\",\"seq\":%u,\"slot\":%u,\"epoch\":%u,"
		"\"time_us\":%" PRIu64 ",\"id\":%u,\"result\":%d,\"write_mode\":\"%s\","
		"\"length\":%u,\"hex\":\"%s\",\"truncated\":%s,\"kind\":%d,\"key\":\"%s\","
		"\"numeric\":%s,\"mantissa\":%" PRId64 ",\"decimals\":%u,\"unit_tag\":%u,"
		"\"state\":\"%s\",\"properties\":%u,\"address\":\"%s\",\"rssi\":%d,"
		"\"interval_us\":%u,\"latency\":%u,\"supervision_timeout_ms\":%u,\"handle\":%u,\"end_handle\":%u}",
		types[e->type], e->sequence, e->slot, e->epoch, e->time_us, e->command_id,
		e->result, e->request ? "wreq" : "wcmd", e->length, hex,
		e->length > sizeof(e->data) ? "true" : "false", f.kind, f.key,
		f.numeric ? "true" : "false", f.mantissa, f.decimals, (unsigned char)f.unit_tag,
		qas_state(e->slot), e->properties, e->address, e->rssi,
		e->interval * 1250U, e->latency, e->timeout * 10U, e->handle, e->end_handle);
}

static void bench_event(const struct qas_event *e)
{
	if (!bench.active || e->slot != bench.slot || e->epoch != bench.epoch) { return; }
	if (e->type == QAS_DISCONNECTED || e->type == QAS_ERROR ||
	    (e->type == QAS_SUBSCRIBED && e->result != 0)) { stop_bench("link_fault"); return; }
	if (e->type == QAS_NOTIFY) {
		struct qas_frame f = qas_parse(e->data, e->length);
		int stream = f.kind == QAS_DATA ? (strcmp(f.key, "DAT0") == 0 ? 0 : 1) : -1;
		/* Only an exact MEAS:OK is an application acknowledgment. Not a DAT. */
		bool ack = qas_parameter_is(e->data, e->length, "MEAS", "OK");
		rate_notify(&bench.core, e->time_us, stream, f.numeric, ack);
		if (bench.core.interval_us != 0 && qas_mod(bench.slot) != 0) { stop_bench("mod_changed"); }
	} else if (e->type == QAS_WRITE_DONE && e->command_id >= bench.run &&
		   e->command_id <= bench.last_command_id && e->time_us < bench.core.end_us) { rate_done(&bench.core, e->result); }
}

static int start_bench(unsigned int slot, uint32_t interval, uint32_t duration,
		       uint32_t drain, bool request)
{
	if (!bench.enabled) { return -ENOTSUP; }
	if (bench.active || !qas_idle(slot)) { return -EBUSY; }
	if (duration < 1000 || duration > 300000 || drain < 500 || drain > 30000 ||
	    (interval != 0 && (interval < 20 || interval > 10000))) { return -EINVAL; }
	if (interval != 0) {
		if (!qas_write_supported(slot, request)) { return -ENOTSUP; }
		if (qas_mod(slot) != 0) { return -EPERM; }
		if (bridge_now_us() - qas_quiet_since(slot) < 1000000) { return -EAGAIN; }
	}
	bench.slot = slot; bench.epoch = qas_epoch(slot); bench.request = request;
	bench.run = ++generated_id; bench.last_command_id = bench.run;
	bench.active = true; bench.available = true; bench.reason = "running";
	bench.drop_events = qas_event_drops(); bench.drop_tx = bridge_usb_tx_drops();
	bench.drop_rx = bridge_usb_rx_drops();
	rate_begin(&bench.core, bridge_now_us(), interval * 1000ULL, duration * 1000ULL, drain * 1000ULL);
	bridge_usb_json("{\"schema\":1,\"type\":\"rate_started\",\"run\":%u,\"slot\":%u,\"epoch\":%u,"
		"\"start_us\":%" PRIu64 ",\"stop_us\":%" PRIu64 ",\"end_us\":%" PRIu64 "}",
		bench.run, slot, bench.epoch, bench.core.start_us, bench.core.stop_us, bench.core.end_us);
	return 0;
}

static bool number(const char *text, uint32_t *value)
{
	if (text == NULL || *text < '0' || *text > '9') { return false; }
	char *end; errno = 0;
	unsigned long n = strtoul(text, &end, 10);
	if (errno || *end || n > UINT32_MAX) { return false; }
	*value = (uint32_t)n; return true;
}

static void command(char *line)
{
	char *tokens[9], *save;
	size_t count = 0;
	for (char *t = strtok_r(line, " ", &save); t != NULL; t = strtok_r(NULL, " ", &save)) {
		if (count == ARRAY_SIZE(tokens)) { count++; break; }
		tokens[count++] = t;
	}
	uint32_t id = 0, slot = 0, a, b, c;
	int err = -EINVAL;
	if (count < 2 || count > ARRAY_SIZE(tokens) || !number(tokens[0], &id) || id == 0 || id >= 0x80000000U) {
		id = 0; goto done;
	}
	const char *op = tokens[1];
	if (!strcmp(op, "status") && count == 2) { status(id); err = 0; }
	else if (!strcmp(op, "report") && count == 2) { report(); err = bench.available ? 0 : -ENOENT; }
	else if (!strcmp(op, "stop") && count == 2) { stop_bench("cancelled"); err = 0; }
	else if (!strcmp(op, "scan") && count == 3 && number(tokens[2], &a)) {
		err = bench.active ? -EBUSY : qas_scan(a);
	} else if (count >= 3 && number(tokens[2], &slot) && slot < QAS_SLOTS) {
		if (!strcmp(op, "connect") && count == 5) {
			err = bench.active ? -EBUSY : qas_connect(slot, tokens[3], tokens[4]);
		} else if (!strcmp(op, "disconnect") && count == 3) {
			if (bench.active && slot == bench.slot) { stop_bench("cancelled"); }
			err = qas_disconnect(slot);
		} else if (!strcmp(op, "cmd") && count == 5 &&
			   (!strcmp(tokens[3], "wcmd") || !strcmp(tokens[3], "wreq"))) {
			err = bench.active ? -EBUSY : qas_send(slot, id, tokens[4], !strcmp(tokens[3], "wreq"));
		} else if (!strcmp(op, "rate") && count == 7 && number(tokens[3], &a) &&
			   number(tokens[4], &b) && number(tokens[5], &c) &&
			   (!strcmp(tokens[6], "wcmd") || !strcmp(tokens[6], "wreq"))) {
			err = start_bench(slot, a, b, c, !strcmp(tokens[6], "wreq"));
		}
	}
 done:
	bridge_usb_json("{\"schema\":1,\"type\":\"result\",\"id\":%u,\"result\":%d}", id, err);
}

int bridge_run(bool rate_test)
{
	bench.enabled = rate_test;
	int err = bridge_usb_init();
	if (err == 0) { err = qas_client_init(); }
	if (err != 0) { LOG_ERR("Bridge initialization failed (%d)", err); return err; }
	bool host = false;
	uint32_t rx_drops = bridge_usb_rx_drops();
	LOG_INF("QAS bridge ready; connections require an explicit USB command");
	for (;;) {
		bool connected = bridge_usb_connected();
		if (host && !connected) {
			stop_bench("usb_disconnected");
			for (unsigned int i = 0; i < QAS_SLOTS; i++) { (void)qas_disconnect(i); }
			bridge_usb_reset();
		} else if (!host && connected) { status(0); }
		host = connected;
		if (bridge_usb_rx_drops() != rx_drops) {
			rx_drops = bridge_usb_rx_drops(); stop_bench("usb_input_loss");
			for (unsigned int i = 0; i < QAS_SLOTS; i++) { (void)qas_disconnect(i); }
			bridge_usb_reset();
		}
		struct qas_event event;
		for (int i = 0; i < 96 && qas_next_event(&event); i++) {
			qas_process(&event);
			if (host) { event_output(&event); }
			bench_event(&event);
		}
		qas_tick();
		if (host) {
			char line[USB_COMMAND_MAX];
			if (bridge_usb_line(line)) { command(line); }
		}
		if (bench.active) {
			uint64_t now = bridge_now_us();
			if (qas_terminal()) { stop_bench("event_loss"); }
			else if (rate_due(&bench.core, now)) {
				bench.last_command_id = ++generated_id;
				err = qas_send(bench.slot, generated_id, "MEAS", bench.request);
				rate_submit(&bench.core, err);
			} else if (now >= bench.core.end_us && !qas_events_pending()) { stop_bench("finished"); }
		}
		k_msleep(1);
	}
	return 0;
}
