#include "rate_core.h"

#include <string.h>
#include <errno.h>

void rate_begin(struct rate_core *r, uint64_t now, uint64_t interval,
		uint64_t duration, uint64_t drain)
{
	memset(r, 0, sizeof(*r));
	r->start_us = now; r->stop_us = now + duration; r->end_us = r->stop_us + drain;
	r->next_us = now; r->interval_us = interval;
}

bool rate_due(struct rate_core *r, uint64_t now)
{
	if (r->interval_us == 0 || now < r->next_us || r->next_us >= r->stop_us) {
		return false;
	}
	/* Account for every scheduled slot, including a delayed final tick. */
	uint64_t last = now < r->stop_us ? now : r->stop_us - 1;
	uint64_t due = (last - r->next_us) / r->interval_us + 1;
	r->slots += (uint32_t)due;
	if (now >= r->stop_us) {
		r->skipped += (uint32_t)due;
		r->next_us += due * r->interval_us;
		return false;
	}
	r->skipped += (uint32_t)(due - 1);
	uint64_t scheduled = r->next_us + (due - 1) * r->interval_us;
	uint64_t late = now - scheduled;
	if (late > r->max_late_us) { r->max_late_us = late; }
	r->next_us += due * r->interval_us;
	return true;
}

void rate_notify(struct rate_core *r, uint64_t now, int stream, bool numeric, bool ack)
{
	if (now < r->start_us || now >= r->end_us) { return; }
	if (stream < 0 || stream > 1) {
		if (now < r->stop_us) {
			if (ack) { r->ack++; } else { r->other++; }
		}
		return;
	}
	if (!numeric) { r->malformed++; return; }
	if (now >= r->stop_us) { r->tail_data++; return; }
	struct rate_gaps *g = &r->data[stream];
	if (g->count != 0) {
		uint64_t gap = now - g->last_us;
		if (g->count == 1 || gap < g->min_us) { g->min_us = gap; }
		if (gap > g->max_us) { g->max_us = gap; }
		g->sum_us += gap;
	} else { g->first_us = now; }
	g->last_us = now;
	g->count++;
}

void rate_submit(struct rate_core *r, int result)
{
	if (result == 0) { r->submitted++; r->pending++; }
	else if (result == -EBUSY) { r->busy++; }
	else { r->errors++; }
}

void rate_done(struct rate_core *r, int result)
{
	if (r->pending != 0) { r->pending--; }
	if (result == 0) { r->done++; } else { r->done_errors++; }
}
