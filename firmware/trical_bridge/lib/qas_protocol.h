#ifndef TRICAL_QAS_PROTOCOL_H
#define TRICAL_QAS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define QAS_PAYLOAD_MAX 244
#define QAS_COMMAND_MAX 48
#define QAS_KEY_MAX 16

enum qas_kind { QAS_OTHER, QAS_DATA, QAS_PARAM };

struct qas_frame {
	enum qas_kind kind;
	char key[QAS_KEY_MAX];
	bool numeric;
	int64_t mantissa;
	uint8_t decimals;
	char unit_tag; /* Lexical m/i prefix; not a calibrated SI conversion. */
};

/* One GATT notification is one packet. Never join or deduplicate equal values. */
struct qas_frame qas_parse(const uint8_t *data, size_t length);
bool qas_parameter_is(const uint8_t *data, size_t length, const char *key, const char *value);
/* Allowlisted ASCII commands only. No arbitrary GATT writes, OTA or reset. */
bool qas_command_valid(const char *command);

#endif
