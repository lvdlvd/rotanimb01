// fmtcan.c — pseudocan lines to and from fifos, see fmtcan.h.

#include "fmtcan.h"

#include "can.h"

static char hex(uint32_t v) { return "0123456789abcdef"[v & 0xf]; }

static int unhex(int c) {
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}
	return -1;
}

// crc16 poly 0xc599, msb first, init 0; feeding the check bytes back yields 0
static uint16_t can_crc16_next(uint16_t crc, uint8_t data) {
	crc ^= (uint16_t)data << 8;
	for (int i = 0; i < 8; i++) {
		if ((crc & 0x8000) != 0) {
			crc ^= 0xc599;
		}
		crc <<= 1;
	}
	return crc;
}

static uint16_t crc_msg(uint32_t header, size_t len, const uint8_t *payload) {
	uint16_t crc = 0;
	crc = can_crc16_next(crc, (uint8_t)(header >> 24));
	crc = can_crc16_next(crc, (uint8_t)(header >> 16));
	crc = can_crc16_next(crc, (uint8_t)(header >> 8));
	crc = can_crc16_next(crc, (uint8_t)header);
	for (size_t i = 0; i < len; i++) {
		crc = can_crc16_next(crc, payload[i]);
	}
	return crc;
}

static size_t fifo_put(struct Fifo *f, unsigned port, unsigned fmi, uint32_t header, size_t len, const uint8_t *payload, int with_crc) {
	if (len > 8 || (len > 0 && payload == NULL)) {
		return 0;
	}
	int ext = can_header_isext(header) != 0;
	int rtr = can_header_isrtr(header) != 0;

	// 3 ida + ['.'+5 idb] + ['R'] + ':' 2len + [':'crc4] + ' 'port + ' 'fmi + '\n'
	size_t total = 3 + (ext ? 6 : 0) + (rtr ? 1 : 0) + 1 + 2 * len + (with_crc ? 5 : 0) + 2 + 2 + (fmi >= 10 ? 1 : 0) + 1;
	if (fifo_free(f) < total) {
		return 0;
	}

	uint32_t ida = can_header_id_a(header);
	fifo_put_head(f, (uint8_t)hex(ida >> 8));
	fifo_put_head(f, (uint8_t)hex(ida >> 4));
	fifo_put_head(f, (uint8_t)hex(ida));
	if (ext) {
		uint32_t idb = can_header_id_b(header);
		fifo_put_head(f, '.');
		for (int s = 16; s >= 0; s -= 4) {
			fifo_put_head(f, (uint8_t)hex(idb >> s));
		}
	}
	if (rtr) {
		fifo_put_head(f, 'R');
	}
	fifo_put_head(f, ':');
	for (size_t i = 0; i < len; i++) {
		fifo_put_head(f, (uint8_t)hex(payload[i] >> 4));
		fifo_put_head(f, (uint8_t)hex(payload[i]));
	}
	if (with_crc) {
		uint16_t crc = crc_msg(header, len, payload);
		fifo_put_head(f, ':');
		for (int s = 12; s >= 0; s -= 4) {
			fifo_put_head(f, (uint8_t)hex(crc >> s));
		}
	}
	fifo_put_head(f, ' ');
	fifo_put_head(f, (uint8_t)('0' + (port & 7)));
	fifo_put_head(f, ' ');
	if (fmi >= 10) {
		fifo_put_head(f, (uint8_t)('0' + fmi / 10 % 10));
	}
	fifo_put_head(f, (uint8_t)('0' + fmi % 10));
	fifo_put_head(f, '\n');
	return total;
}

size_t can_fifo_put(struct Fifo *f, unsigned port, unsigned fmi, uint32_t header, size_t len, const uint8_t *payload) {
	return fifo_put(f, port, fmi, header, len, payload, 0);
}

size_t can_fifo_put_crc(struct Fifo *f, unsigned port, unsigned fmi, uint32_t header, size_t len, const uint8_t *payload) {
	return fifo_put(f, port, fmi, header, len, payload, 1);
}

// scan up to 8 hex digits from fifo positions [*i, end); count in *ndig
static uint32_t scan_hex(struct Fifo *f, size_t *i, size_t end, int *ndig) {
	uint32_t v = 0;
	*ndig = 0;
	while (*i < end && *ndig < 8) {
		int d = unhex(fifo_at(f, *i));
		if (d < 0) {
			break;
		}
		v = v << 4 | (uint32_t)d;
		++*i;
		++*ndig;
	}
	return v;
}

int can_fifo_get(struct Fifo *f, unsigned *port, uint32_t *header, size_t *len, uint8_t *payload) {
	// skip line terminators left over from \r\n and blank lines
	while (fifo_avail(f) > 0) {
		uint8_t c = fifo_at(f, 0);
		if (c != '\n' && c != '\r') {
			break;
		}
		fifo_pop_tail(f, 1);
	}

	// a complete line?
	size_t n = fifo_avail(f);
	size_t end = 0;
	while (end < n && fifo_at(f, end) != '\n' && fifo_at(f, end) != '\r') {
		end++;
	}
	if (end == n) {
		if (n > 96) { // unterminated flood: not our protocol, drop it
			fifo_pop_tail(f, n);
			return -1;
		}
		return 0;
	}
	size_t consume = end + 1;

	size_t i = 0;
	int ndig;
	int ok = 0;
	*len = 0;
	*port = 3;

	do { // parse [0, end); any failure falls out with ok = 0
		uint32_t ida = scan_hex(f, &i, end, &ndig);
		if (ndig == 0 || ida == 0 || ida > 0x7ff) {
			break;
		}
		*header = can_header(ida, 0);
		if (i < end && fifo_at(f, i) == '.') {
			i++;
			uint32_t idb = scan_hex(f, &i, end, &ndig);
			if (ndig == 0 || idb > 0x3ffff) {
				break;
			}
			*header = can_header_ext(ida, idb, 0);
		}
		if (i < end && fifo_at(f, i) == 'R') {
			i++;
			*header = can_header_setrtr(*header, 1);
		}

		if (i < end && fifo_at(f, i) == ':') { // payload
			i++;
			int bad = 0;
			while (*len < 8 && i < end) {
				int d1 = unhex(fifo_at(f, i));
				if (d1 < 0) {
					break; // non-hex: end of the payload field
				}
				int d2 = (i + 1 < end) ? unhex(fifo_at(f, i + 1)) : -1;
				if (d2 < 0) {
					bad = 1; // a lone trailing nibble
					break;
				}
				payload[(*len)++] = (uint8_t)(d1 << 4 | d2);
				i += 2;
			}
			if (bad || (i < end && unhex(fifo_at(f, i)) >= 0)) {
				break; // lone nibble, or more than 8 payload bytes
			}
		}

		if (i < end && fifo_at(f, i) == ':') { // checksum, must verify
			i++;
			uint32_t chk = scan_hex(f, &i, end, &ndig);
			if (ndig != 4) {
				break;
			}
			uint16_t crc = crc_msg(*header, *len, payload);
			crc = can_crc16_next(crc, (uint8_t)(chk >> 8));
			crc = can_crc16_next(crc, (uint8_t)chk);
			if (crc != 0) {
				break;
			}
		}

		if (i < end && fifo_at(f, i) == ' ') { // port; the rest (fmi) is ignored
			i++;
			uint8_t c = (i < end) ? fifo_at(f, i) : 0;
			if (c >= '0' && c <= '7') {
				*port = (unsigned)(c - '0');
			}
		}
		ok = 1;
	} while (0);

	fifo_pop_tail(f, consume);
	return ok ? (int)consume : -1;
}
