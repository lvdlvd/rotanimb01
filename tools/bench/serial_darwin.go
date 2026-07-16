package main

import "golang.org/x/sys/unix"

const (
	ioctlGet = unix.TIOCGETA
	ioctlSet = unix.TIOCSETA
)

func setSpeed(t *unix.Termios, baud int) {
	t.Ispeed = uint64(baud)
	t.Ospeed = uint64(baud)
}
