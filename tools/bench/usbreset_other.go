//go:build !linux

package main

import "fmt"

func usbReset(dev string) error {
	return fmt.Errorf("usbreset: only implemented on Linux (USBDEVFS_RESET)")
}
