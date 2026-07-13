package main

import (
	"bytes"
	"encoding/hex"
	"testing"
)

// harness-emitted line (crc-less, trailing port+fmi), per lib/fmtcan:
// id29(MEAS, STATUS, srcid 0xee, seq 0) = 0x0611ee90 -> ida 184, idb 1ee90
func TestParseHarnessLine(t *testing.T) {
	h, p, err := parseLine("184.1ee90:0000303907d20000 1 0\r\n")
	if err != nil {
		t.Fatal(err)
	}
	id := h.id29()
	if !h.isExt() || lccOf(id) != lccMEAS || msgidOf(id) != measSTATUS || srcidOf(id) != 0xee {
		t.Fatalf("header %v id29 %08x", h, id)
	}
	if id != id29(lccMEAS, measSTATUS, 0xee, 0) {
		t.Fatalf("id29 %08x vs builder %08x", id, id29(lccMEAS, measSTATUS, 0xee, 0))
	}
	if len(p) != 8 || p[4] != 0x07 || p[5] != 0xd2 {
		t.Fatalf("payload %x", p)
	}
}

// our own emitted form (with crc) must parse and verify
func TestSendRoundtrip(t *testing.T) {
	payload, _ := hex.DecodeString("07d0000000a00000")
	h := mkHeader29(id29(lccTMC, cmdSTATE, 0xb0, 5))
	var buf bytes.Buffer
	if _, err := buf.WriteString(""); err != nil {
		t.Fatal(err)
	}
	line := h.String() + ":" + hex.EncodeToString(payload) + ":" +
		hexCrc(checksum(h, payload)) + " 1 0\n"
	h2, p2, err := parseLine(line)
	if err != nil {
		t.Fatal(err)
	}
	if h2 != h || !bytes.Equal(p2, payload) {
		t.Fatalf("roundtrip %v %x vs %v %x", h2, p2, h, payload)
	}
	// corrupt one payload nibble: the crc must catch it
	bad := []byte(line)
	bad[12] ^= 1
	if _, _, err := parseLine(string(bad)); err == nil {
		t.Fatal("corruption not caught")
	}
}

// the mon example from the bench: TMC CMD_STATE as bm10tool printed it
func TestParseKnownWire(t *testing.T) {
	h, p, err := parseLine("680.10090:07d0000000a00000")
	if err != nil {
		t.Fatal(err)
	}
	if lccOf(h.id29()) != lccTMC || msgidOf(h.id29()) != cmdSTATE {
		t.Fatalf("id29 %08x", h.id29())
	}
	if len(p) != 8 {
		t.Fatalf("payload %x", p)
	}
}

func hexCrc(c uint16) string {
	const hexd = "0123456789abcdef"
	return string([]byte{hexd[c>>12&15], hexd[c>>8&15], hexd[c>>4&15], hexd[c&15]})
}
