#ifndef TRICAL_QAS_CLIENT_H
#define TRICAL_QAS_CLIENT_H

#include "qas_protocol.h"
#include <zephyr/kernel.h>

#define QAS_SLOTS 3
#define QAS_ADDRESS_MAX 40

enum qas_event_type {
	QAS_CONNECTED, QAS_DISCONNECTED, QAS_DISCOVERED, QAS_SUBSCRIBED,
	QAS_NOTIFY, QAS_WRITE, QAS_WRITE_DONE, QAS_ADVERTISEMENT, QAS_LINK_PARAMS,
	QAS_ERROR, QAS_SCAN_STOPPED
};

struct qas_event {
	enum qas_event_type type;
	uint64_t time_us;
	uint32_t sequence, epoch, command_id;
	int result;
	uint16_t length, handle, end_handle, interval, timeout, latency;
	uint8_t slot, properties;
	int8_t rssi;
	bool request;
	char address[QAS_ADDRESS_MAX];
	uint8_t data[QAS_PAYLOAD_MAX];
};

uint64_t bridge_now_us(void);
int qas_client_init(void);
int qas_connect(unsigned int slot, const char *address, const char *type);
int qas_disconnect(unsigned int slot);
int qas_scan(uint32_t duration_ms);
int qas_send(unsigned int slot, uint32_t id, const char *command, bool request);
bool qas_next_event(struct qas_event *event);
/* Call from the single application owner for each event, then emit it to USB. */
void qas_process(const struct qas_event *event);
void qas_tick(void);
bool qas_ready(unsigned int slot);
bool qas_idle(unsigned int slot);
bool qas_write_supported(unsigned int slot, bool request);
uint64_t qas_quiet_since(unsigned int slot);
bool qas_events_pending(void);
int qas_mod(unsigned int slot);
uint32_t qas_epoch(unsigned int slot);
const char *qas_state(unsigned int slot);
uint32_t qas_event_drops(void);
bool qas_terminal(void);

#endif
