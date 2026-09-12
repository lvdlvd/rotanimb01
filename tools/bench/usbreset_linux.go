//go:build linux

package main

import (
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"golang.org/x/sys/unix"
)

// usbReset issues USBDEVFS_RESET to the USB device behind a tty node (a
// /dev/serial/by-id/... link or /dev/ttyACMn): the device re-enumerates,
// the firmware keeps running. This is the cure for a wedged CDC endpoint
// on the harness that does not cost the flight (doc/BENCH-OPERATIONS.md).
// Needs write access to /dev/bus/usb (root, or a udev rule).
func usbReset(dev string) error {
	tty, err := filepath.EvalSymlinks(dev)
	if err != nil {
		return err
	}
	p, err := filepath.EvalSymlinks("/sys/class/tty/" + filepath.Base(tty) + "/device")
	if err != nil {
		return fmt.Errorf("%s: not a tty with a sysfs device: %v", dev, err)
	}
	// walk up from the interface to the USB device (the first ancestor
	// carrying busnum + devnum)
	for {
		if _, e1 := os.Stat(p + "/busnum"); e1 == nil {
			if _, e2 := os.Stat(p + "/devnum"); e2 == nil {
				break
			}
		}
		parent := filepath.Dir(p)
		if parent == p || parent == "/" {
			return fmt.Errorf("%s: no USB device ancestor in sysfs", dev)
		}
		p = parent
	}
	bus, err := readInt(p + "/busnum")
	if err != nil {
		return err
	}
	num, err := readInt(p + "/devnum")
	if err != nil {
		return err
	}
	node := fmt.Sprintf("/dev/bus/usb/%03d/%03d", bus, num)
	f, err := os.OpenFile(node, os.O_RDWR, 0)
	if err != nil {
		return err
	}
	defer f.Close()
	const usbdevfsReset = 0x5514
	if _, _, errno := unix.Syscall(unix.SYS_IOCTL, f.Fd(), usbdevfsReset, 0); errno != 0 {
		return fmt.Errorf("USBDEVFS_RESET %s: %v", node, errno)
	}
	fmt.Printf("usbreset: %s (%s) reset\n", dev, node)
	return nil
}

func readInt(path string) (int, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return 0, err
	}
	return strconv.Atoi(strings.TrimSpace(string(b)))
}
