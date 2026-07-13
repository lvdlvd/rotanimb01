#pragma once

// dronecan — a minimal DroneCAN (UAVCAN v0) broadcast encoder for the
// harness's on-board GPS/airspeed feeder (fdm-DESIGN.md F4): GNSS Fix2,
// RawAirData and NodeStatus, bit-exact per libcanard (scalar codec, float16,
// transfer CRC and framing are faithful ports; layouts follow ArduPilot's
// dsdlc output). Freestanding — no device headers — so the host check
// (make dccheck) pins every byte against reference frames generated with
// ArduPilot's own libcanard.

#include <stddef.h>
#include <stdint.h>

enum {
	DRONECAN_FIX2_ID = 1063,
	DRONECAN_RAWAIR_ID = 1027,
	DRONECAN_NODESTATUS_ID = 341,
	DRONECAN_PRIO_MEDIUM = 16,
	DRONECAN_PRIO_LOW = 24,
};
#define DRONECAN_FIX2_SIGNATURE 0xCA41E7000F37435FULL
#define DRONECAN_RAWAIR_SIGNATURE 0xC77DF38BA122F5DAULL
#define DRONECAN_NODESTATUS_SIGNATURE 0x0F0868D0C1A7C6F1ULL

struct DroneCanFrame {
	uint32_t id29;
	uint8_t len;
	uint8_t data[8];
};

// split one transfer into classic CAN frames (single, or CRC + 7-byte
// chunks with alternating toggle); returns the frame count, 0 if out is
// too small. *tid advances by one transfer.
int dronecan_broadcast(uint16_t dtid, uint64_t signature, uint8_t prio, uint8_t node,
                       uint8_t *tid, const uint8_t *payload, size_t plen,
                       struct DroneCanFrame *out, int max);

struct DroneCanFix2 {
	uint64_t usec;
	int64_t lat_deg_1e8, lon_deg_1e8;
	int32_t h_mm; // used for both ellipsoid and MSL height
	float vned[3];
	uint8_t sats, status; // status 3 = 3D fix
	float cov[6];         // position and velocity variances
	float pdop;
};

// encoders return the payload byte length (tail array optimization applied)
size_t dronecan_fix2(const struct DroneCanFix2 *m, uint8_t buf[64]);
size_t dronecan_rawair(float diff_pa, float static_pa, float temp_k, uint8_t buf[24]);
size_t dronecan_nodestatus(uint32_t uptime_sec, uint8_t buf[8]);
