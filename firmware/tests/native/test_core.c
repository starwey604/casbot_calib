#include "qas_protocol.h"
#include "rate_core.h"

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

static struct qas_frame parse(const char *text)
{
	return qas_parse((const uint8_t *)text, strlen(text));
}

static void protocol(void)
{
	struct qas_frame f = parse("DAT1: m-0.001\r\n");
	assert(f.kind == QAS_DATA && f.numeric && f.mantissa == -1 && f.decimals == 3 && f.unit_tag == 'm');
	f = parse("DAT0:+12.345");
	assert(f.numeric && f.mantissa == 12345 && f.decimals == 3);
	f = parse("DAT1:i+.0123");
	assert(f.numeric && f.mantissa == 123 && f.decimals == 4 && f.unit_tag == 'i');
	const char *bad[] = {"DAT0:", "DAT0:m", "DAT1:NaN", "DAT1:1.2mm", "DAT1:1.2 DAT0:3",
		"DAT1:0x1", "DAT1:1e3", "DAT1:--1", "DAT1:1.2.3", "DAT1:1.",
		"DAT1:9223372036854775808", "DAT1:0.1234567890"};
	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		f = parse(bad[i]); assert(f.kind == QAS_DATA && !f.numeric);
	}
	assert(parse("MEAS:OK").kind == QAS_PARAM);
	assert(parse("MOD:0").kind == QAS_PARAM);
	assert(parse("garbage").kind == QAS_OTHER);
	assert(parse("\"bad\":0").kind == QAS_OTHER);
	assert(qas_parse(NULL, 20).kind == QAS_OTHER);
	assert(qas_parse((const uint8_t *)"x", QAS_PAYLOAD_MAX + 1).kind == QAS_OTHER);
	uint8_t binary[] = {'D','A','T','0',':','1',0,'2'};
	assert(!qas_parse(binary, sizeof(binary)).numeric);
	assert(qas_command_valid("MEAS") && qas_command_valid("MOD") && qas_command_valid("MOD:0"));
	assert(qas_command_valid("NAM:GL-BLE#01"));
	const char *blocked[] = {"CLR", "RESTORE", "OTA", "DAT0:OK", "ffd1:1", "MOD:\n1", "MOD:1:2",
		"mod:1", "MOD:", "MOD:1 2", "MOD:\"", "MEAS:1", ""};
	for (size_t i = 0; i < sizeof(blocked) / sizeof(blocked[0]); i++) {
		assert(!qas_command_valid(blocked[i]));
	}
	/* Length-bounded binary fuzz exercises the same parser linked into firmware. */
	uint32_t random = 1234;
	uint8_t bytes[QAS_PAYLOAD_MAX];
	for (int i = 0; i < 20000; i++) {
		for (size_t j = 0; j < sizeof(bytes); j++) {
			random = random * 1664525U + 1013904223U; bytes[j] = random >> 24;
		}
		(void)qas_parse(bytes, (random % sizeof(bytes)) + 1);
	}
}

static void rates(void)
{
	struct rate_core r;
	rate_begin(&r, 1000000, 100000, 1000000, 500000);
	assert(!rate_due(&r, 999999));
	assert(rate_due(&r, 1000000)); rate_submit(&r, 0);
	assert(!rate_due(&r, 1000000));
	assert(rate_due(&r, 1350000)); rate_submit(&r, -EBUSY);
	assert(r.slots == 4 && r.skipped == 2 && r.max_late_us == 50000);
	/* A delayed tick at the end accounts remaining slots without catch-up sends. */
	assert(!rate_due(&r, 2100000));
	assert(r.slots == 10 && r.skipped == 8);
	assert(r.slots == r.skipped + r.submitted + r.busy + r.errors);
	rate_done(&r, 0); assert(r.done == 1 && r.pending == 0);
	rate_notify(&r, 999999, 0, true, false);
	rate_notify(&r, 1000000, 0, true, false);
	rate_notify(&r, 1250000, 0, true, false); /* Same reading is still a new notification. */
	rate_notify(&r, 1300000, 1, true, false);
	rate_notify(&r, 1350000, -1, false, true);
	rate_notify(&r, 1400000, 1, false, false);
	rate_notify(&r, 2000000, 0, true, false); /* end-exclusive window, drain only */
	rate_notify(&r, 2500000, 0, true, false); /* outside drain */
	assert(r.data[0].count == 2 && r.data[1].count == 1 && r.tail_data == 1);
	assert(r.data[0].min_us == 250000 && r.data[0].max_us == 250000);
	assert(r.ack == 1 && r.malformed == 1);
	rate_begin(&r, 0, 0, 1000000, 500000);
	assert(!rate_due(&r, 100));
	rate_submit(&r, -ENOMEM); rate_done(&r, 3);
	assert(r.errors == 1 && r.done_errors == 1 && r.pending == 0);
}

int main(void)
{
	protocol(); rates();
	puts("PASS: QAS parsing/command allowlist, fixed-grid scheduling, independent streams and drain boundaries");
	return 0;
}
