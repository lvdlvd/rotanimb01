// dccheck — host validation of the on-target DroneCAN encoder: every byte
// pinned against reference frames generated with ArduPilot's own libcanard
// (scratchpad dronecan/refvec.c). Built and run by `make dccheck`.

#include "dronecan.h"

#include <stdio.h>
#include <string.h>

static int failures;

static void check_frames(const char *name, const struct DroneCanFrame *got, int n,
                         const uint32_t *ids, const char (*hexes)[24], int want) {
	if (n != want) {
		printf("FAIL %s: %d frames, want %d\n", name, n, want);
		failures++;
		return;
	}
	for (int i = 0; i < n; i++) {
		uint8_t wd[8];
		int wl = 0;
		for (const char *p = hexes[i]; *p; p += 2) {
			unsigned v;
			sscanf(p, "%2x", &v);
			wd[wl++] = (uint8_t)v;
		}
		if (got[i].id29 != ids[i] || got[i].len != wl || memcmp(got[i].data, wd, (size_t)wl) != 0) {
			printf("FAIL %s frame %d: id %08x len %d\n", name, i, (unsigned)got[i].id29, got[i].len);
			failures++;
			return;
		}
	}
	printf("OK   %s: %d frames byte-exact\n", name, n);
}

int main(void) {
	// Fix2: usec 1234567, lon 5.12345678, lat 52.12345678, h 100 m,
	// vned {25, -1.5, 0.25}, 12 sats, 3D fix, cov {1,1,1,.25,.25,.25}, pdop 1.5
	struct DroneCanFix2 fx = {
		.usec = 1234567,
		.lon_deg_1e8 = 512345678LL,
		.lat_deg_1e8 = 5212345678LL,
		.h_mm = 100000,
		.vned = {25.0f, -1.5f, 0.25f},
		.sats = 12,
		.status = 3,
		.cov = {1, 1, 1, 0.25f, 0.25f, 0.25f},
		.pdop = 1.5f,
	};
	uint8_t buf[64];
	size_t len = dronecan_fix2(&fx, buf);
	struct DroneCanFrame fr[16];
	uint8_t tid = 0;
	int n = dronecan_broadcast(DRONECAN_FIX2_ID, DRONECAN_FIX2_SIGNATURE, DRONECAN_PRIO_MEDIUM,
	                           42, &tid, buf, len, fr, 16);
	static const uint32_t fix_ids[10] = {0x1004272a, 0x1004272a, 0x1004272a, 0x1004272a, 0x1004272a,
	                                     0x1004272a, 0x1004272a, 0x1004272a, 0x1004272a, 0x1004272a};
	static const char fix_hex[10][24] = {
		"5aef87d612000080", "0000000000000020", "00000000004ec600", "891e0270ad71b020",
		"6821804504300800", "0000c8410000c020", "bf0000803e330000", "06003c003c003c20",
		"0034003400340000", "3e60",
	};
	check_frames("fix2", fr, n, fix_ids, fix_hex, 10);

	len = dronecan_rawair(382.8f, 0.0f, 288.15f, buf);
	tid = 0;
	n = dronecan_broadcast(DRONECAN_RAWAIR_ID, DRONECAN_RAWAIR_SIGNATURE, DRONECAN_PRIO_MEDIUM,
	                       42, &tid, buf, len, fr, 16);
	static const uint32_t air_ids[3] = {0x1004032a, 0x1004032a, 0x1004032a};
	static const char air_hex[3][24] = {"2237000000000080", "6666bf4300000020", "00815c000040"};
	check_frames("rawair", fr, n, air_ids, air_hex, 3);

	len = dronecan_nodestatus(3600, buf);
	tid = 0;
	n = dronecan_broadcast(DRONECAN_NODESTATUS_ID, DRONECAN_NODESTATUS_SIGNATURE, DRONECAN_PRIO_LOW,
	                       42, &tid, buf, len, fr, 16);
	static const uint32_t ns_ids[1] = {0x1801552a};
	static const char ns_hex[1][24] = {"100e0000000000c0"};
	check_frames("nodestatus", fr, n, ns_ids, ns_hex, 1);

	printf(failures ? "dccheck FAILED\n" : "dccheck OK\n");
	return failures ? 1 : 0;
}
