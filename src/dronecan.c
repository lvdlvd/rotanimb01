#include "dronecan.h"

// ---- libcanard scalar codec (faithful ports) -----------------------------------

// the bit stream fills each destination byte MSB-first
static void copy_bits(const uint8_t *src, uint32_t src_ofs, uint32_t src_len,
                      uint8_t *dst, uint32_t dst_ofs) {
	src += src_ofs / 8;
	dst += dst_ofs / 8;
	src_ofs %= 8;
	dst_ofs %= 8;
	uint32_t last_bit = src_ofs + src_len;
	while (last_bit > src_ofs) {
		uint32_t src_bit = src_ofs % 8;
		uint32_t dst_bit = dst_ofs % 8;
		uint32_t max_ofs = src_bit > dst_bit ? src_bit : dst_bit;
		uint32_t n = last_bit - src_ofs;
		if (8 - max_ofs < n) {
			n = 8 - max_ofs;
		}
		uint8_t write_mask = (uint8_t)((uint8_t)(0xFF00u >> n) >> dst_bit);
		uint8_t src_data = (uint8_t)(((uint32_t)src[src_ofs / 8] << src_bit) >> dst_bit);
		dst[dst_ofs / 8] = (uint8_t)((dst[dst_ofs / 8] & ~write_mask) | (src_data & write_mask));
		src_ofs += n;
		dst_ofs += n;
	}
}

// value in two's complement for signed fields; returns the advanced offset
static uint32_t enc(uint8_t *dst, uint32_t bit_ofs, uint8_t bit_len, uint64_t value) {
	uint8_t storage[8] = {0};
	uint8_t n = bit_len <= 8 ? 1 : bit_len <= 16 ? 2 : bit_len <= 32 ? 4 : 8;
	for (uint8_t i = 0; i < n; i++) { // little-endian scalar image
		storage[i] = (uint8_t)(value >> (8 * i));
	}
	if (bit_len % 8 != 0) {
		storage[bit_len / 8] = (uint8_t)(storage[bit_len / 8] << (8 - bit_len % 8));
	}
	copy_bits(storage, 0, bit_len, dst, bit_ofs);
	return bit_ofs + bit_len;
}

static uint32_t f32bits(float f) {
	union { float f; uint32_t u; } v = {.f = f};
	return v.u;
}

// float16 conversion, the libcanard magic-constant algorithm
static uint16_t f16bits(float value) {
	const uint32_t f32inf = 255ul << 23;
	const uint32_t f16inf = 31ul << 23;
	const uint32_t sign_mask = 0x80000000ul;
	const uint32_t round_mask = 0xFFFFF000ul;
	union { float f; uint32_t u; } in = {.f = value}, magic = {.u = 15ul << 23};
	uint32_t sign = in.u & sign_mask;
	in.u ^= sign;
	uint16_t out;
	if (in.u >= f32inf) {
		out = in.u > f32inf ? 0x7FFFu : 0x7C00u;
	} else {
		in.u &= round_mask;
		in.f *= magic.f;
		in.u -= round_mask;
		if (in.u > f16inf) {
			in.u = f16inf;
		}
		out = (uint16_t)(in.u >> 13);
	}
	return (uint16_t)(out | (sign >> 16));
}

// ---- transfer CRC and framing ---------------------------------------------------

static uint16_t crc_add(uint16_t crc, uint8_t b) {
	crc ^= (uint16_t)((uint16_t)b << 8);
	for (int i = 0; i < 8; i++) {
		crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
	}
	return crc;
}

int dronecan_broadcast(uint16_t dtid, uint64_t signature, uint8_t prio, uint8_t node,
                       uint8_t *tid, const uint8_t *payload, size_t plen,
                       struct DroneCanFrame *out, int max) {
	uint32_t id = (uint32_t)prio << 24 | (uint32_t)dtid << 8 | node;
	int nf = 0;
	if (plen <= 7) {
		if (max < 1) {
			return 0;
		}
		out[0].id29 = id;
		out[0].len = (uint8_t)(plen + 1);
		for (size_t i = 0; i < plen; i++) {
			out[0].data[i] = payload[i];
		}
		out[0].data[plen] = (uint8_t)(0xC0u | (*tid & 0x1F)); // start|end|toggle0
		nf = 1;
	} else {
		uint16_t crc = 0xFFFF;
		for (int i = 0; i < 8; i++) { // signature LSB first
			crc = crc_add(crc, (uint8_t)(signature >> (8 * i)));
		}
		for (size_t i = 0; i < plen; i++) {
			crc = crc_add(crc, payload[i]);
		}
		// virtual buffer: crc lo, crc hi, payload — split 7 per frame
		size_t total = plen + 2;
		uint8_t toggle = 0;
		for (size_t off = 0; off < total; off += 7, toggle ^= 1) {
			if (nf >= max) {
				return 0;
			}
			struct DroneCanFrame *f = &out[nf++];
			f->id29 = id;
			size_t n = total - off < 7 ? total - off : 7;
			for (size_t i = 0; i < n; i++) {
				size_t k = off + i;
				f->data[i] = k == 0 ? (uint8_t)crc : k == 1 ? (uint8_t)(crc >> 8) : payload[k - 2];
			}
			uint8_t tail = (uint8_t)(*tid & 0x1F);
			if (off == 0) {
				tail |= 0x80;
			}
			if (off + 7 >= total) {
				tail |= 0x40;
			}
			tail |= (uint8_t)(toggle << 5);
			f->data[n] = tail;
			f->len = (uint8_t)(n + 1);
		}
	}
	*tid = (uint8_t)((*tid + 1) & 0x1F);
	return nf;
}

// ---- the messages (layouts per ArduPilot's dsdlc output) -------------------------

size_t dronecan_fix2(const struct DroneCanFix2 *m, uint8_t buf[64]) {
	for (int i = 0; i < 64; i++) {
		buf[i] = 0;
	}
	uint32_t o = 0;
	o = enc(buf, o, 56, m->usec); // timestamp
	o = enc(buf, o, 56, 0);       // gnss_timestamp: unknown
	o = enc(buf, o, 3, 0);        // gnss_time_standard NONE
	o += 13;                      // void13
	o = enc(buf, o, 8, 0);        // num_leap_seconds
	o = enc(buf, o, 37, (uint64_t)m->lon_deg_1e8);
	o = enc(buf, o, 37, (uint64_t)m->lat_deg_1e8);
	o = enc(buf, o, 27, (uint64_t)(uint32_t)m->h_mm); // ellipsoid
	o = enc(buf, o, 27, (uint64_t)(uint32_t)m->h_mm); // msl
	for (int i = 0; i < 3; i++) {
		o = enc(buf, o, 32, f32bits(m->vned[i]));
	}
	o = enc(buf, o, 6, m->sats);
	o = enc(buf, o, 2, m->status);
	o = enc(buf, o, 4, 0); // mode SINGLE
	o = enc(buf, o, 6, 0); // sub_mode
	o = enc(buf, o, 6, 6); // covariance length
	for (int i = 0; i < 6; i++) {
		o = enc(buf, o, 16, f16bits(m->cov[i]));
	}
	o = enc(buf, o, 16, f16bits(m->pdop));
	// ecef_position_velocity: length bit omitted (tail array optimization)
	return (o + 7) / 8;
}

size_t dronecan_rawair(float diff_pa, float static_pa, float temp_k, uint8_t buf[24]) {
	for (int i = 0; i < 24; i++) {
		buf[i] = 0;
	}
	uint32_t o = 0;
	o = enc(buf, o, 8, 0); // flags
	o = enc(buf, o, 32, f32bits(static_pa));
	o = enc(buf, o, 32, f32bits(diff_pa));
	o = enc(buf, o, 16, f16bits(0.0f)); // static pressure sensor temp
	o = enc(buf, o, 16, f16bits(0.0f)); // differential pressure sensor temp
	o = enc(buf, o, 16, f16bits(temp_k));
	o = enc(buf, o, 16, f16bits(0.0f)); // pitot temp
	// covariance: length prefix omitted (tail array optimization), empty
	return (o + 7) / 8;
}

size_t dronecan_nodestatus(uint32_t uptime_sec, uint8_t buf[8]) {
	for (int i = 0; i < 8; i++) {
		buf[i] = 0;
	}
	uint32_t o = 0;
	o = enc(buf, o, 32, uptime_sec);
	o = enc(buf, o, 2, 0);  // health OK
	o = enc(buf, o, 3, 0);  // mode OPERATIONAL
	o = enc(buf, o, 3, 0);  // sub_mode
	o = enc(buf, o, 16, 0); // vendor specific
	return (o + 7) / 8;
}
