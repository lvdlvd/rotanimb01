#pragma once

// SPSC CAN message queue: thread enqueues, the tx-complete irq pumps into
// the 3 hardware tx buffers.

#include <stddef.h>
#include <stdint.h>

// Internal to a firmware image: this never goes on a wire, so adding to it
// cannot affect any log format or decoder.
struct CanMsg {
	uint64_t ts;     // arrival or send time in core cycles, 0 if not stamped
	uint32_t header;
	size_t	 len;
	uint8_t	 tag;    // caller's label, e.g. which bus the message came from
	uint8_t	 payload[8];
};

struct CanMsgQueue {
	volatile uint32_t head;
	volatile uint32_t tail;
	volatile uint32_t dropped;
	struct CanMsg	  elem[64];
};

static inline int canmsgq_empty(struct CanMsgQueue *q) { return q->head == q->tail; }

#define NELEM(x) (sizeof(x) / sizeof(x[0]))
// return null if empty/full
static inline struct CanMsg *canmsgq_tail(struct CanMsgQueue *q) {
	return (q->head == q->tail) ? NULL : &q->elem[q->tail % NELEM(q->elem)];
}
static inline struct CanMsg *canmsgq_head(struct CanMsgQueue *q) {
	if (q->head == q->tail + NELEM(q->elem)) {
		q->dropped++;
		return NULL;
	}
	return &q->elem[q->head % NELEM(q->elem)];
}
#undef NELEM

// call these exactly once for each non null returned value of msgq_head and _tail
static inline void canmsgq_pop_tail(struct CanMsgQueue *q) { ++q->tail; }
static inline void canmsgq_push_head(struct CanMsgQueue *q) { ++q->head; }

static inline void canmsgq_enq_tag(struct CanMsgQueue *q, uint8_t tag, uint64_t ts, uint32_t header, size_t len,
								   const uint8_t *payload) {
	struct CanMsg *m = canmsgq_head(q);
	if (m == NULL) {
		return;
	}
	m->ts     = ts;
	m->header = header;
	m->len	  = len;
	m->tag	  = tag;
	for (size_t i = 0; i < len; ++i) {
		m->payload[i] = payload[i];
	}
	canmsgq_push_head(q);
}

static inline void canmsgq_enq(struct CanMsgQueue *q, uint64_t ts, uint32_t header, size_t len, const uint8_t* payload) {
	canmsgq_enq_tag(q, 0, ts, header, len, payload);
}