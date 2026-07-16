// serbridge — TCP <-> serial bridge, runs ON the pi next to the DUT's
// MAVLink CDC port. One client at a time; the serial device is reopened
// automatically when it vanishes (bench power cycles unplug it mid-session).
package main

import (
	"fmt"
	"net"
	"os"
	"sync"
	"time"
)

func cmdSerbridge(dev string, port int) error {
	ln, err := net.Listen("tcp", fmt.Sprintf(":%d", port))
	if err != nil {
		return err
	}
	fmt.Printf("serbridge: %s <-> :%d\n", dev, port)
	for {
		client, err := ln.Accept()
		if err != nil {
			return err
		}
		fmt.Printf("client %s\n", client.RemoteAddr())
		bridge(dev, client)
		client.Close()
		fmt.Println("client gone")
	}
}

// bridge pumps both directions until the client goes away; a dead serial
// device is reopened in place (the client just sees a data gap).
func bridge(dev string, client net.Conn) {
	var mu sync.Mutex
	var ser *os.File
	open := func() *os.File {
		mu.Lock()
		defer mu.Unlock()
		if ser != nil {
			ser.Close()
			ser = nil
		}
		for i := 0; i < 30; i++ {
			f, err := openSerial(dev, 115200)
			if err == nil {
				ser = f
				return f
			}
			time.Sleep(2 * time.Second)
		}
		return nil
	}
	if open() == nil {
		return
	}
	done := make(chan struct{})
	// client -> serial
	go func() {
		defer close(done)
		buf := make([]byte, 4096)
		for {
			n, err := client.Read(buf)
			if err != nil {
				return
			}
			mu.Lock()
			s := ser
			mu.Unlock()
			if s != nil {
				s.Write(buf[:n])
			}
		}
	}()
	// serial -> client
	buf := make([]byte, 4096)
	for {
		select {
		case <-done:
			return
		default:
		}
		mu.Lock()
		s := ser
		mu.Unlock()
		if s == nil {
			return
		}
		s.SetReadDeadline(time.Now().Add(300 * time.Millisecond))
		n, err := s.Read(buf)
		if n > 0 {
			if _, werr := client.Write(buf[:n]); werr != nil {
				return
			}
		}
		if err != nil && !os.IsTimeout(err) {
			fmt.Printf("serial lost (%v), reopening\n", err)
			if open() == nil {
				return
			}
		}
	}
}
