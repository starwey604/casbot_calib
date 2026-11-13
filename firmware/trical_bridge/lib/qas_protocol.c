#include "qas_protocol.h"

#include <limits.h>
#include <string.h>

static bool padding(uint8_t c)
{
	return c == 0 || c == '\r' || c == '\n' || c == ' ' || c == '\t';
}

struct qas_frame qas_parse(const uint8_t *data, size_t length)
{
	struct qas_frame out = {0};
	size_t begin = 0, end = length, colon;

	if (data == NULL || length == 0 || length > QAS_PAYLOAD_MAX) {
		return out;
	}
	while (begin < end && padding(data[begin])) { begin++; }
	while (end > begin && padding(data[end - 1])) { end--; }
	colon = begin;
	while (colon < end && data[colon] != ':') { colon++; }
	if (colon == begin || colon == end || colon - begin >= sizeof(out.key)) {
		return out;
	}
	for (size_t i = begin; i < colon; i++) {
		if (!((data[i] >= 'A' && data[i] <= 'Z') ||
		      (data[i] >= '0' && data[i] <= '9'))) { return out; }
	}
	memcpy(out.key, data + begin, colon - begin);
	out.kind = strcmp(out.key, "DAT0") == 0 || strcmp(out.key, "DAT1") == 0 ?
		QAS_DATA : QAS_PARAM;
	if (out.kind != QAS_DATA) { return out; }

	size_t i = colon + 1;
	while (i < end && (data[i] == ' ' || data[i] == '\t')) { i++; }
	if (i < end && (data[i] == 'm' || data[i] == 'i')) { out.unit_tag = data[i++]; }
	bool negative = i < end && data[i] == '-';
	if (i < end && (data[i] == '+' || data[i] == '-')) { i++; }
	bool dot = false, digit = false;
	int64_t value = 0;
	for (; i < end; i++) {
		uint8_t c = data[i];
		if (c == '.' && !dot) { dot = true; continue; }
		if (c < '0' || c > '9') { return out; }
		if (value > (INT64_MAX - (c - '0')) / 10) { return out; }
		value = value * 10 + c - '0';
		digit = true;
		if (dot && ++out.decimals > 9) { return out; }
	}
	if (!digit || (end > 0 && data[end - 1] == '.')) { return out; }
	out.mantissa = negative ? -value : value;
	out.numeric = true;
	return out;
}

bool qas_command_valid(const char *command)
{
	static const char *const reads[] = {
		"MEAS", "IDEN", "NAM", "SN", "INT", "MOD", "IDLEN", "IDLTM", "TYPE",
		"CAT", "ACC", "AGFMT", "DELEN", "RPTED", "TONM", "BASE", "TXPWR",
		"RPTEDDL", "RPTCHARINT", "REGION", "HIDMOD", "HIDAPPEND", "CTLLV"
	};
	static const char *const writes[] = {
		"NAM", "INT", "MOD", "IDLEN", "IDLTM", "TYPE", "CAT", "ACC", "AGFMT",
		"DELEN", "RPTED", "TONM", "BASE", "TXPWR", "RPTEDDL", "RPTCHARINT",
		"REGION", "HIDMOD", "HIDAPPEND", "CTLLV"
	};
	if (command == NULL) { return false; }
	size_t length = strlen(command);
	if (length == 0 || length >= QAS_COMMAND_MAX) { return false; }
	const char *colon = strchr(command, ':');
	size_t key_length = colon == NULL ? length : (size_t)(colon - command);
	if (colon != NULL) {
		if (colon[1] == '\0') { return false; }
		for (const char *c = colon + 1; *c != '\0'; c++) {
			if (!( (*c >= 'A' && *c <= 'Z') || (*c >= 'a' && *c <= 'z') ||
			       (*c >= '0' && *c <= '9') || strchr("_.#+-", *c) != NULL)) {
				return false;
			}
		}
	}
	const char *const *keys = colon == NULL ? reads : writes;
	size_t count = colon == NULL ? sizeof(reads) / sizeof(reads[0]) :
		sizeof(writes) / sizeof(writes[0]);
	for (size_t i = 0; i < count; i++) {
		if (strlen(keys[i]) == key_length && strncmp(command, keys[i], key_length) == 0) {
			return true;
		}
	}
	return false;
}

bool qas_parameter_is(const uint8_t *data, size_t length, const char *key, const char *value)
{
	if (data == NULL || length > QAS_PAYLOAD_MAX) { return false; }
	size_t start = 0, end = length;
	while (start < end && padding(data[start])) { start++; }
	while (end > start && padding(data[end - 1])) { end--; }
	size_t k = strlen(key), v = strlen(value);
	return end - start == k + 1 + v && memcmp(data + start, key, k) == 0 &&
		data[start + k] == ':' && memcmp(data + start + k + 1, value, v) == 0;
}
