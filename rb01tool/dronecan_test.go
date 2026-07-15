package main

// Reference vectors generated with ArduPilot's own libcanard + dsdlc output
// (scratchpad/dronecan/refvec.c): every byte below came off canardBroadcast.

import (
	"encoding/hex"
	"strings"
	"testing"
)

func refPayloadFix2() *fix2 {
	return &fix2{
		usec:        1234567,
		lonDeg1e8:   512345678,
		latDeg1e8:   5212345678,
		heightEllMM: 100000,
		heightMSLMM: 100000,
		nedVel:      [3]float32{25.0, -1.5, 0.25},
		satsUsed:    12,
		status:      3,
		covariance:  []float32{1, 1, 1, 0.25, 0.25, 0.25},
		pdop:        1.5,
	}
}

const refFix2Payload = "87 d6 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 4e c6 89 1e 02 70 ad 71 b0 68 21 80 45 04 30 08 00 00 c8 41 00 00 c0 bf 00 00 80 3e 33 00 06 00 3c 00 3c 00 3c 00 34 00 34 00 34 00 3e"

var refFix2Frames = []string{
	"1004272a 5a ef 87 d6 12 00 00 80",
	"1004272a 00 00 00 00 00 00 00 20",
	"1004272a 00 00 00 00 00 4e c6 00",
	"1004272a 89 1e 02 70 ad 71 b0 20",
	"1004272a 68 21 80 45 04 30 08 00",
	"1004272a 00 00 c8 41 00 00 c0 20",
	"1004272a bf 00 00 80 3e 33 00 00",
	"1004272a 06 00 3c 00 3c 00 3c 20",
	"1004272a 00 34 00 34 00 34 00 00",
	"1004272a 3e 60",
}

const refRawAirPayload = "00 00 00 00 00 66 66 bf 43 00 00 00 00 81 5c 00 00"

var refRawAirFrames = []string{
	"1004032a 22 37 00 00 00 00 00 80",
	"1004032a 66 66 bf 43 00 00 00 20",
	"1004032a 00 81 5c 00 00 40",
}

func unhex(t *testing.T, s string) []byte {
	b, err := hex.DecodeString(strings.ReplaceAll(s, " ", ""))
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func checkFrames(t *testing.T, got []canFrame, want []string) {
	if len(got) != len(want) {
		t.Fatalf("%d frames, want %d", len(got), len(want))
	}
	for i, w := range want {
		parts := strings.SplitN(w, " ", 2)
		var id uint32
		if _, err := hexScan(parts[0], &id); err != nil {
			t.Fatal(err)
		}
		if got[i].id != id {
			t.Fatalf("frame %d id %08x want %08x", i, got[i].id, id)
		}
		wd := unhex(t, parts[1])
		if hex.EncodeToString(got[i].data) != hex.EncodeToString(wd) {
			t.Fatalf("frame %d data %x want %x", i, got[i].data, wd)
		}
	}
}

func hexScan(s string, v *uint32) (int, error) {
	var x uint64
	for _, c := range s {
		x <<= 4
		switch {
		case c >= '0' && c <= '9':
			x |= uint64(c - '0')
		case c >= 'a' && c <= 'f':
			x |= uint64(c-'a') + 10
		default:
			return 0, nil
		}
	}
	*v = uint32(x)
	return 1, nil
}

func TestFix2Payload(t *testing.T) {
	got := encodeFix2(refPayloadFix2())
	want := unhex(t, refFix2Payload)
	if hex.EncodeToString(got) != hex.EncodeToString(want) {
		t.Fatalf("payload\n got %x\nwant %x", got, want)
	}
}

func TestFix2Frames(t *testing.T) {
	tid := uint8(0)
	frames := broadcast(fix2ID, fix2Signature, prioMedium, feederNode, &tid, encodeFix2(refPayloadFix2()))
	checkFrames(t, frames, refFix2Frames)
	if tid != 1 {
		t.Fatalf("tid %d", tid)
	}
}

func TestRawAirPayloadAndFrames(t *testing.T) {
	m := &rawAir{diffPa: 382.8, staticPa: 0, airTempK: 288.15}
	got := encodeRawAir(m)
	want := unhex(t, refRawAirPayload)
	if hex.EncodeToString(got) != hex.EncodeToString(want) {
		t.Fatalf("payload\n got %x\nwant %x", got, want)
	}
	tid := uint8(0)
	frames := broadcast(rawAirID, rawAirSig, prioMedium, feederNode, &tid, got)
	checkFrames(t, frames, refRawAirFrames)
}

// reference: ArduPilot libcanard, uptime 3600 s, health OK, mode OPERATIONAL
func TestNodeStatusFrames(t *testing.T) {
	tid := uint8(0)
	frames := broadcast(nodeStatusID, nodeStatusSig, prioLow, feederNode, &tid, encodeNodeStatus(3600))
	checkFrames(t, frames, []string{"1801552a 100e0000000000c0"})
}
