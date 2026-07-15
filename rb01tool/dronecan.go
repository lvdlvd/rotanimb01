package main

// dronecan — a minimal DroneCAN (UAVCAN v0) broadcast encoder for the GPS /
// airspeed feeder (fdm-DESIGN.md F4): uavcan.equipment.gnss.Fix2 and
// uavcan.equipment.air_data.RawAirData, encoded bit-exactly per libcanard
// (the scalar codec, float16, transfer CRC and framing are faithful ports;
// the layouts follow ArduPilot's dsdlc output; the tests pin every byte to
// reference frames generated with ArduPilot's own libcanard).

import "math"

const (
	fix2ID        = 1063
	fix2Signature = 0xCA41E7000F37435F
	rawAirID      = 1027
	rawAirSig     = 0xC77DF38BA122F5DA
	nodeStatusID  = 341
	nodeStatusSig = 0x0F0868D0C1A7C6F1

	prioMedium = 16
	prioLow    = 24
	feederNode = 42
)

// ---- libcanard scalar codec ----------------------------------------------------

// copyBitArray is a faithful port of libcanard's bit copier: the bit stream
// fills each destination byte MSB-first.
func copyBitArray(src []byte, srcOfs, srcLen uint32, dst []byte, dstOfs uint32) {
	src = src[srcOfs/8:]
	dst = dst[dstOfs/8:]
	srcOfs %= 8
	dstOfs %= 8
	lastBit := srcOfs + srcLen
	for lastBit > srcOfs {
		srcBit := srcOfs % 8
		dstBit := dstOfs % 8
		maxOfs := srcBit
		if dstBit > maxOfs {
			maxOfs = dstBit
		}
		copyBits := lastBit - srcOfs
		if 8-maxOfs < copyBits {
			copyBits = 8 - maxOfs
		}
		writeMask := byte(uint32(0xFF00)>>copyBits) >> dstBit
		srcData := byte(uint32(src[srcOfs/8]) << srcBit >> dstBit)
		dst[dstOfs/8] = dst[dstOfs/8]&^writeMask | srcData&writeMask
		srcOfs += copyBits
		dstOfs += copyBits
	}
}

// encodeScalar appends value (two's complement for signed) at bitOfs.
func encodeScalar(dst []byte, bitOfs uint32, bitLen uint8, value uint64) uint32 {
	var storage [8]byte
	var n uint8
	switch {
	case bitLen <= 8:
		n = 1
	case bitLen <= 16:
		n = 2
	case bitLen <= 32:
		n = 4
	default:
		n = 8
	}
	for i := uint8(0); i < n; i++ { // little-endian, as on every target we run
		storage[i] = byte(value >> (8 * i))
	}
	if bitLen%8 != 0 {
		storage[bitLen/8] <<= 8 - bitLen%8
	}
	copyBitArray(storage[:], 0, uint32(bitLen), dst, bitOfs)
	return bitOfs + uint32(bitLen)
}

// float16 conversion, the libcanard magic-constant algorithm
func float16bits(value float32) uint16 {
	const (
		f32inf    = uint32(255) << 23
		f16inf    = uint32(31) << 23
		signMask  = uint32(0x80000000)
		roundMask = uint32(0xFFFFF000)
	)
	magic := math.Float32frombits(uint32(15) << 23)
	u := math.Float32bits(value)
	sign := u & signMask
	u ^= sign
	var out uint16
	if u >= f32inf {
		if u > f32inf {
			out = 0x7FFF
		} else {
			out = 0x7C00
		}
	} else {
		u &= roundMask
		f := math.Float32frombits(u) * magic
		u = math.Float32bits(f) - roundMask
		if u > f16inf {
			u = f16inf
		}
		out = uint16(u >> 13)
	}
	return out | uint16(sign>>16)
}

// ---- transfer CRC and framing --------------------------------------------------

func crc16Add(crc uint16, b byte) uint16 {
	crc ^= uint16(b) << 8
	for i := 0; i < 8; i++ {
		if crc&0x8000 != 0 {
			crc = crc<<1 ^ 0x1021
		} else {
			crc <<= 1
		}
	}
	return crc
}

func transferCRC(signature uint64, payload []byte) uint16 {
	crc := uint16(0xFFFF)
	for i := 0; i < 8; i++ { // signature LSB first
		crc = crc16Add(crc, byte(signature>>(8*i)))
	}
	for _, b := range payload {
		crc = crc16Add(crc, b)
	}
	return crc
}

type canFrame struct {
	id   uint32 // 29-bit
	data []byte
}

// broadcast splits one transfer into classic CAN frames per libcanard.
func broadcast(dtid uint16, signature uint64, prio uint8, node uint8, tid *uint8, payload []byte) []canFrame {
	id := uint32(prio)<<24 | uint32(dtid)<<8 | uint32(node)
	tail := func(start, end bool, toggle uint8) byte {
		var b byte = *tid & 0x1F
		if start {
			b |= 0x80
		}
		if end {
			b |= 0x40
		}
		b |= toggle << 5
		return b
	}
	var frames []canFrame
	if len(payload) <= 7 {
		frames = append(frames, canFrame{id, append(append([]byte{}, payload...), tail(true, true, 0))})
	} else {
		crc := transferCRC(signature, payload)
		buf := append([]byte{byte(crc), byte(crc >> 8)}, payload...)
		toggle := uint8(0)
		for off := 0; off < len(buf); off += 7 {
			end := off+7 >= len(buf)
			chunk := buf[off:min(off+7, len(buf))]
			frames = append(frames, canFrame{id, append(append([]byte{}, chunk...), tail(off == 0, end, toggle))})
			toggle ^= 1
		}
	}
	*tid = (*tid + 1) & 0x1F
	return frames
}

// ---- the two messages -----------------------------------------------------------

type fix2 struct {
	usec        uint64
	lonDeg1e8   int64
	latDeg1e8   int64
	heightEllMM int32
	heightMSLMM int32
	nedVel      [3]float32
	satsUsed    uint8
	status      uint8 // 3 = 3D fix
	covariance  []float32
	pdop        float32
}

// encodeFix2 with tail array optimization (last field ecef omitted, len 0)
func encodeFix2(m *fix2) []byte {
	buf := make([]byte, 222)
	o := uint32(0)
	o = encodeScalar(buf, o, 56, m.usec) // timestamp
	o = encodeScalar(buf, o, 56, 0)      // gnss_timestamp: unknown
	o = encodeScalar(buf, o, 3, 0)       // gnss_time_standard NONE
	o += 13                              // void13
	o = encodeScalar(buf, o, 8, 0)       // num_leap_seconds
	o = encodeScalar(buf, o, 37, uint64(m.lonDeg1e8))
	o = encodeScalar(buf, o, 37, uint64(m.latDeg1e8))
	o = encodeScalar(buf, o, 27, uint64(m.heightEllMM))
	o = encodeScalar(buf, o, 27, uint64(m.heightMSLMM))
	for i := 0; i < 3; i++ {
		o = encodeScalar(buf, o, 32, uint64(math.Float32bits(m.nedVel[i])))
	}
	o = encodeScalar(buf, o, 6, uint64(m.satsUsed))
	o = encodeScalar(buf, o, 2, uint64(m.status))
	o = encodeScalar(buf, o, 4, 0) // mode SINGLE
	o = encodeScalar(buf, o, 6, 0) // sub_mode
	o = encodeScalar(buf, o, 6, uint64(len(m.covariance)))
	for _, c := range m.covariance {
		o = encodeScalar(buf, o, 16, uint64(float16bits(c)))
	}
	o = encodeScalar(buf, o, 16, uint64(float16bits(m.pdop)))
	// ecef_position_velocity: len bit omitted (TAO), zero elements
	return buf[:(o+7)/8]
}

type rawAir struct {
	diffPa   float32
	staticPa float32
	airTempK float32
}

func encodeRawAir(m *rawAir) []byte {
	buf := make([]byte, 50)
	o := uint32(0)
	o = encodeScalar(buf, o, 8, 0) // flags
	o = encodeScalar(buf, o, 32, uint64(math.Float32bits(m.staticPa)))
	o = encodeScalar(buf, o, 32, uint64(math.Float32bits(m.diffPa)))
	o = encodeScalar(buf, o, 16, uint64(float16bits(0))) // static press sensor temp
	o = encodeScalar(buf, o, 16, uint64(float16bits(0))) // diff press sensor temp
	o = encodeScalar(buf, o, 16, uint64(float16bits(m.airTempK)))
	o = encodeScalar(buf, o, 16, uint64(float16bits(0))) // pitot temp
	// covariance: len prefix omitted (TAO), zero elements
	return buf[:(o+7)/8]
}

func encodeNodeStatus(uptimeSec uint32) []byte {
	buf := make([]byte, 8)
	o := uint32(0)
	o = encodeScalar(buf, o, 32, uint64(uptimeSec))
	o = encodeScalar(buf, o, 2, 0)  // health OK
	o = encodeScalar(buf, o, 3, 0)  // mode OPERATIONAL
	o = encodeScalar(buf, o, 3, 0)  // sub_mode
	o = encodeScalar(buf, o, 16, 0) // vendor specific
	return buf[:(o+7)/8]
}
