#ifndef TRICAL_RATE_CORE_H
#define TRICAL_RATE_CORE_H

#include <stdbool.h>
#include <stdint.h>

struct rate_gaps {
	uint32_t count;
	uint64_t first_us, last_us, min_us, max_us, sum_us;
};

struct rate_core {
	uint64_t start_us, stop_us, end_us, next_us, interval_us, max_late_us;
	uint32_t slots, skipped, submitted, busy, errors, done, done_errors;
	uint32_t ack, malformed, other, tail_data, pending;
	struct rate_gaps data[2]; /* DAT0 and DAT1 remain separate streams. */
};

void rate_begin(struct rate_core *r, uint64_t now, uint64_t interval,
		uint64_t duration, uint64_t drain);
/* Fixed grid. Missed slots are counted/skipped, never burst-replayed. */
bool rate_due(struct rate_core *r, uint64_t now);
void rate_notify(struct rate_core *r, uint64_t now, int stream, bool numeric, bool ack);
void rate_submit(struct rate_core *r, int result);
void rate_done(struct rate_core *r, int result);

#endif
