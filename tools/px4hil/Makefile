# cgo rejects -fno-builtin in #cgo CFLAGS; the emulation build wants it
# (see fdm/Makefile), so route it through the allowance.
export CGO_CFLAGS_ALLOW = -fno-builtin

test:
	go test ./...

px4hil:
	go build -o px4hil .

.PHONY: test px4hil
