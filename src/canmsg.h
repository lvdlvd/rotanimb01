#pragma once

// canmsg — the harness's CAN message headers and dictionary.
//
// All traffic uses 29-bit headers, interpreted as bit-fields per the shared
// in-house dictionary convention (ARINC825-derived), and big-endian payloads
// (lib/binary.h). Header layout, MSB-justified in the 29-bit id:
//
//   [28:26] LCC     logical communication channel: 0b001 measurements,
//                   0b110 (TMC) configuration/commands
//   [25:19] MSGID   message label (7 bits)
//   [18]    FSB     functional status bit (with DLC: NO/NCD/FT/FW)
//   [17]    LCL=0   messages may be routed
//   [16]    PRV=1   not defined cf arinc825
//   [15:8]  SRCID   8 bits hashed from the device UID (0 reserved: time sync)
//   [7:4]   0b1001  reserved
//   [3:0]   ts_seq  sequence number or low 4 bits of the µs timestamp
//
// MSGIDs 0x01..0x3f (measurements) and 0x00..0x03, 0x70..0x7f (TMC) are
// allocated by the existing device dictionary; the harness owns the 0x40
// block in both channels:
//
//   TMC  -> harness   0x40 CMD_STATE  i16 V cm/s, i16 hdot cm/s, i16 psidot mrad/s, u16 flags
//                     0x41 CMD_ENV    u16 QNH Pa/10, u16 T0 0.1K, u16 B 0.01uT, i16 incl 0.01deg
//                     0x42 CMD_NOISE  per-sensor enable mask + levels
//                     0x43 FDM_MODE   u8 mode (0 kinematic / 1 six-dof), u8 flags
//                     0x44 FDM_INIT   u16 alt m, u16 IAS 0.1 m/s, u16 heading 0.01 deg
//                                     -> fdm trim & reset (air-start), fdm-DESIGN.md
//                     0x45 PWM_CAL    u8 ch (0 ail 1 ele 2 thr 3 rud), u8 part;
//                                     part 0: u16 min/trim/max us; part 1: i16
//                                     full deflection 0.01 deg (thr: 1e-4)
//   All command frames are 8 bytes, zero-padded: the decoder treats a
//   shorter frame as truncated and refuses it (cmd_snapshot len guard).
//   MEAS harness ->   0x40 PWM14      4 x u16 us, ch 1-4, 50 Hz + on change > 2 us
//                     0x41 PWM58      idem ch 5-8
//                     0x42 STATUS     u32 time us, u16 psi 0.01deg, u16 flags (bit0: cmd stale), 10 Hz
//                     0x43 DIAG       u16 can tx, u16 can rx, u16 rx ovfl, u16 pwm errs, 1 Hz

#include "device.h"

#include <stdint.h>

enum {
	CANMSG_LCC_MEAS = 1, // measurements
	CANMSG_LCC_TMC = 6,  // configuration / commands

	// harness dictionary (the 0x40 block)
	CANMSG_CMD_STATE = 0x40, // TMC
	CANMSG_CMD_ENV = 0x41,
	CANMSG_CMD_NOISE = 0x42,
	CANMSG_FDM_MODE = 0x43,
	CANMSG_FDM_INIT = 0x44,
	CANMSG_PWM_CAL = 0x45,
	CANMSG_PWM14 = 0x40, // MEAS
	CANMSG_PWM58 = 0x41,
	CANMSG_STATUS = 0x42,
	CANMSG_DIAG = 0x43,
};

// compose a 29-bit id (LCL=0, PRV=1, reserved=0b1001 fixed)
static inline uint32_t canmsg_id29(uint32_t lcc, uint32_t msgid, uint32_t fsb, uint32_t srcid, uint32_t seq) {
	return ((lcc & 7) << 26) | ((msgid & 0x7f) << 19) | ((fsb & 1) << 18) | (1u << 16) |
	       ((srcid & 0xff) << 8) | (0x9u << 4) | (seq & 0xf);
}

static inline uint32_t canmsg_lcc(uint32_t id29) { return (id29 >> 26) & 7; }
static inline uint32_t canmsg_msgid(uint32_t id29) { return (id29 >> 19) & 0x7f; }
static inline uint32_t canmsg_fsb(uint32_t id29) { return (id29 >> 18) & 1; }
static inline uint32_t canmsg_srcid(uint32_t id29) { return (id29 >> 8) & 0xff; }
static inline uint32_t canmsg_seq(uint32_t id29) { return id29 & 0xf; }

// SRCID: the 96-bit device UID folded to 8 bits, avoiding the reserved 0
static inline uint8_t canmsg_srcid_self(void) {
	uint32_t h = DEVSIG.UID0 ^ DEVSIG.UID1 ^ DEVSIG.UID2;
	h ^= h >> 16;
	h ^= h >> 8;
	return (uint8_t)h ? (uint8_t)h : 1;
}
