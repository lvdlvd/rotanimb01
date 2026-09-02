#pragma once

// can.h — portable uint32 representation of CAN message headers (ID-A, ID-B,
// IDE, RTR) for in-memory manipulation, interoperable with an existing
// family of in-house CAN devices. The same representation fronts hardware
// with different header register layouts (the G4 FDCAN here; the L4-family
// bxCAN elsewhere), and the field order is chosen so that comparing two
// headers as unsigned integers reproduces their arbitration priority on the
// wire (lower value wins the bus, dominant bits first).

#include "stdint.h"

/**  uint32 representation of id_a, id_b, the IDE and the RTR bits for in-memory manipulation.

let IDE be the identifier extension flag, when set to 1 meaning the extended id field is present
let RTR be the remote transmission request flag, when set to 1 meaning no payload, but a request

	bit 31 (1 bit): unused
	bit 30-20 (11 bits): ID-A[10:0]               0x7ff_ ____
	bit 19    (1 bit): RTR                        0x___8 ____
	bit 18    (1 bit): IDE                        0x___4 ____
	bit 17-16 (2 bits):  ID-B[17:16]              0x___3 ____
	bit 15-0  (16 bits): ID-B[15:0]               0x____ ffff

*/
static inline uint32_t can_header(uint32_t id_a, int32_t rtr) { return ((id_a & 0x7ff) << 20) | ((rtr != 0) ? (1 << 19) : 0); }

static inline uint32_t can_header_ext(uint32_t id_a, uint32_t id_b, int32_t rtr) {
	return ((id_a & 0x7ff) << 20) | ((rtr != 0) ? (1 << 19) : 0) | (1 << 18) | (id_b & 0x3ffff);
}

static inline int32_t  can_header_isext(uint32_t h) { return ((h & (1 << 18)) != 0) ? -1 : 0; }
static inline uint32_t can_header_id_a(uint32_t h) { return (h >> 20) & 0x7ff; }
static inline uint32_t can_header_id_b(uint32_t h) { return h & 0x3ffff; }

static inline uint32_t can_header_setrtr(uint32_t h, int32_t rtr) { return (rtr != 0) ? (h | (1 << 19)) : (h & ~(1U << 19)); }
static inline int32_t  can_header_isrtr(uint32_t h) { return ((h & (1 << 19)) != 0) ? -1 : 0; }

// from29 creates an extended header in our 32 bit representation from a 29 (id_a+id_b). RTR is clear.
static inline uint32_t can_header_from29(uint32_t id29) { return ((id29 & (0x7ff << 18)) << 2) | (1 << 18) | (id29 & 0x3ffff); }

// to29 extracts the 29 bit id_a+id_b from a header, (assuming it is extended).
static inline uint32_t can_header_to29(uint32_t header) { return ((header >> 2) & (0x7ff << 18)) | (header & 0x3ffff); }

