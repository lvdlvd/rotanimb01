package main

import "golang.org/x/sys/unix"

const (
	ioctlGet = unix.TCGETS
	ioctlSet = unix.TCSETS
)

var baudBits = map[int]uint32{
	9600: unix.B9600, 57600: unix.B57600, 115200: unix.B115200,
	230400: unix.B230400, 460800: unix.B460800, 921600: unix.B921600,
	1000000: unix.B1000000,
}

func setSpeed(t *unix.Termios, baud int) {
	b, ok := baudBits[baud]
	if !ok {
		b = unix.B115200
	}
	t.Cflag &^= unix.CBAUD
	t.Cflag |= b
	t.Ispeed = b
	t.Ospeed = b
}
