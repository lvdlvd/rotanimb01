// serial — raw termios serial open, no cgo. All bench devices are USB CDC
// (baud largely nominal), but the ST-Link VCP forwards the requested baud to
// its bridged UART, so we set it for real.
package main

import (
	"fmt"
	"os"

	"golang.org/x/sys/unix"
)

// openSerial opens nonblocking (a plain tty open can hang on modem-control
// lines, and Go's poller needs a nonblocking fd for read deadlines to work).
func openSerial(path string, baud int) (*os.File, error) {
	f, err := os.OpenFile(path, os.O_RDWR|unix.O_NOCTTY|unix.O_NONBLOCK, 0)
	if err != nil {
		return nil, err
	}
	if err := makeRaw(int(f.Fd()), baud); err != nil {
		f.Close()
		return nil, fmt.Errorf("%s: termios: %w", path, err)
	}
	return f, nil
}

func makeRaw(fd, baud int) error {
	t, err := unix.IoctlGetTermios(fd, ioctlGet)
	if err != nil {
		return err
	}
	t.Iflag &^= unix.IGNBRK | unix.BRKINT | unix.PARMRK | unix.ISTRIP |
		unix.INLCR | unix.IGNCR | unix.ICRNL | unix.IXON
	t.Oflag &^= unix.OPOST
	t.Lflag &^= unix.ECHO | unix.ECHONL | unix.ICANON | unix.ISIG | unix.IEXTEN
	t.Cflag &^= unix.CSIZE | unix.PARENB
	t.Cflag |= unix.CS8 | unix.CREAD | unix.CLOCAL
	t.Cc[unix.VMIN] = 0 // nonblocking fd: the Go poller does the waiting
	t.Cc[unix.VTIME] = 0
	setSpeed(t, baud)
	return unix.IoctlSetTermios(fd, ioctlSet, t)
}
