#pragma once

// fmtcan — CAN messages as quasi-readable "pseudocan" lines for tunneling
// CAN over a serial byte pipe (VCP), reading and writing lib/fifo.h fifos
// directly:
//
//     ID-A['.' ID-B]['R'] ':' hexpayload ':' crc16 ' ' port ' ' fmi '\n'
//
// ID-A is ID-A[10:0] in hex (000..7ff); the '.' and ID-B[17:0] are present
// for extended 29-bit headers; 'R' if the RTR bit is set. The crc16 runs
// over the four header bytes then the payload, polynomial 0xc599 msb-first
// (NOT the CAN bus crc). port is one digit, a bitmask naming which of up
// to 3 CAN ports the message goes to / came from; fmi is the receive
// filter match index. The header representation is nlib/can.h's priority-
// preserving uint32.
//
// On input the ':'crc16 and the trailing " port fmi" are optional; a bare
// "ID-A:hexpayload" line is accepted (port defaults to all).

#include <stddef.h>
#include <stdint.h>

#include "fifo.h"

// can_fifo_put appends one complete pseudocan line to the fifo, all or
// nothing: if the encoding does not fit the free space (or len > 8),
// nothing is written and the return is 0, else the bytes appended.
// The _crc form includes the ':'crc16 field, the plain form omits it.
// Safe against a concurrent tail-side reader (head-side only).
size_t can_fifo_put(struct Fifo *f, unsigned port, unsigned fmi, uint32_t header, size_t len, const uint8_t *payload);
size_t can_fifo_put_crc(struct Fifo *f, unsigned port, unsigned fmi, uint32_t header, size_t len, const uint8_t *payload);

// can_fifo_get parses and consumes the next newline-terminated line.
// Returns n > 0: a valid message, n bytes consumed; 0: no complete line
// buffered yet, nothing consumed (call again when more bytes arrive);
// -1: a malformed line (or an unterminated flood) was consumed and
// dropped. A missing checksum is accepted, a present one must verify.
// *port is the port digit, 3 (all) when absent; payload must have room
// for 8 bytes. Safe against a concurrent head-side writer (an irq
// feeding the fifo): only the tail side is touched.
int can_fifo_get(struct Fifo *f, unsigned *port, uint32_t *header, size_t *len, uint8_t *payload);
