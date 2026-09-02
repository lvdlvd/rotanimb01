// usb.c — minimal CDC-ACM device (virtual serial port), see usb.h. Register
// definitions from the generated device.h; packet memory and endpoint-
// register write discipline in usbfs.h.
//
//  TODO:
//    proper suspend/wakeup detection

#include "usb.h"

#include "usbfs.h"

static enum usb_state_t _usb_state		 = USB_UNATTACHED;

struct UsbDebug usb_debug; // event counters + last setup, see usb.h

static uint16_t _ctrl_line_state; // bit0 = DTR, bit1 = RTS, as last set by the host
static uint8_t _ctrl_out_pending; // a control DATA-OUT stage is in flight

// manufacturer/product as set by usb_init; the serial comes from the die id
static const char *_manufacturer = "";
static const char *_product = "";

enum usb_state_t usb_state(void) { return _usb_state; }

static const char *const _sstr[] = {"UNATTACHED", "DEFAULT", "ADDRESS", "CONFIGURED", "SUSPENDED", "UNDEFINED"};

const char *usb_state_str(enum usb_state_t s) {
	if (s > 5) s = 5;
	return _sstr[s];
}

void usb_init(const char *manufacturer, const char *product) {

	_manufacturer = manufacturer ? manufacturer : "";
	_product = product ? product : "";
	_usb_state = USB_UNATTACHED;

	// a fresh RCC reset pulse first: the 48 MHz clock domain has been seen
	// coming up wedged out of power-on (EPR file and SOF receiver dead while
	// the APB side responds); the block reset forces a clean resync
	RCC.APB1RSTR1 |= RCC_APB1RSTR1_USBRST;
	RCC.APB1RSTR1 &= ~RCC_APB1RSTR1_USBRST;

	USB.CNTR = USB_CNTR_FRES; // hold in reset, clear power down
	for (volatile int i = 0; i < 400; i++) {
		// tSTARTUP >= 1 us after PDWN clear before releasing FRES (RM0440 45.5.2)
	}
	USB.ISTR   = 0;				 // clear pending interrupts
	USB.DADDR  = 0;				 // Device address is zero
	USB.BTABLE = 0;				 // btable at start of packet mem
	USB.LPMCSR = 0;				 // no low-power support
	USB.BCDR   = 0;				 // no battery charging detector support

	for (int i = 0; i < 8; ++i) {
		usb_ep_reset(i);
	}

	// 1024 bytes in USB_PMA
	// btable: 8 * 4 * 2 = 64 bytes
	USB_RAM.btable[0].ADDR_TX = 1 * 64;	 // control endpoint 0x00/0x80
	USB_RAM.btable[0].ADDR_RX = 2 * 64;
	usb_ep_set_rx_size(0, 64);

	USB_RAM.btable[1].ADDR_TX = 3 * 64;	 // bulk endpoint 0x01/0x81
	USB_RAM.btable[1].ADDR_RX = 4 * 64;
	usb_ep_set_rx_size(1, 64);

	USB_RAM.btable[2].ADDR_TX = 5 * 64;	 // notification endpoint 0x82 (CDC; silent)
	USB_RAM.btable[2].COUNT_TX = 0;

	// bring out of reset and enable interrupts
	USB.CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM; // | USB_CNTR_WKUPM | USB_CNTR_SUSPM;
	USB.BCDR |= USB_BCDR_DPPU;	// connect DP pullup
}

void usb_shutdown(void) {
	USB.CNTR = USB_CNTR_FRES;
	USB.ISTR = 0;
	USB.BCDR = 0;  // disconnect
	USB.CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;

	_usb_state = USB_UNATTACHED;
}


static size_t buf2fifo(uint8_t ep, struct Fifo* fifo) {
	uint16_t len = usb_ep_get_rx_count(ep);
	if (len > fifo_free(fifo)) {
		return 0;
	}
	const volatile uint16_t *src = usb_ep_rx_buf(ep);
	for (size_t i = 0; i < len/2; ++i) {
		uint16_t v = src[i];
		fifo_put_head(fifo, v);
		fifo_put_head(fifo, v>>8);
	}
	 if (len % 2 == 1) {
        uint16_t v = ((const volatile uint8_t *)src)[len - 1];
		fifo_put_head(fifo, v);
    }
	return len;
}

static size_t fifo2buf(uint8_t ep, struct Fifo* fifo) {
	size_t len = fifo_avail(fifo);
	if (len > 64) {
		len = 64;
	}

	volatile uint16_t *dst = usb_ep_tx_buf(ep);
	for (size_t i = 0; i < len/2; ++i) {
		uint16_t v = fifo_get_tail(fifo);
		uint16_t w = fifo_get_tail(fifo);
		dst[i] = v | (w<<8);
	}
	 if (len % 2 == 1) {
        ((volatile uint8_t *)dst)[len - 1] = fifo_get_tail(fifo);
    }

	usb_ep_set_tx_count(ep, len);
	return len;
}

static void handle_ep0(void);  // below

size_t usb_recv(struct Fifo* fifo) {

	uint16_t istr = USB.ISTR;

	USB.ISTR &= ~(USB_ISTR_SOF | USB_ISTR_ESOF | USB_ISTR_ERR | USB_ISTR_PMAOVR | USB_ISTR_L1REQ);

	// the other ISTR bits are r or rc_w0, meaning to clear them write ~bit to the register, avoid read-mod-write.

	if (istr & USB_ISTR_RESET) {
		usb_debug.resets++;

		USB.ISTR &= ~(USB_ISTR_RESET | USB_ISTR_WKUP | USB_ISTR_SUSP);
		USB.CNTR &= ~(USB_CNTR_RESUME | USB_CNTR_FSUSP | USB_CNTR_LP_MODE | USB_CNTR_PDWN | USB_CNTR_FRES);

		usb_ep_config(0, USB_EP_TYPE_CONTROL, 0);
		usb_ep_set_stat_tx(0, USB_EP_STAT_STALL);  // only setup will succeed
		usb_ep_set_stat_rx(0, USB_EP_STAT_STALL);

		for (int i = 1; i < 8; ++i) {
			usb_ep_reset(i);
		}

		usb_daddr_add_set(0);
		USB.DADDR |= USB_DADDR_EF;

		_ctrl_line_state = 0; // bus reset: the port is no longer open
		_ctrl_out_pending = 0;
		_usb_state = USB_DEFAULT;
		return 0;
	}

	if ((istr & USB_ISTR_CTR) != 0) {
		uint8_t ep = usb_istr_get_ep_id();

		if (ep == 0) {
			handle_ep0();
			return 0;
		}

		// assert (ep == 1)
		if (USB.EPR[ep] & USB_EPR_CTR_TX) {
			// IN: last transmit succeeded.
			// assert(usb_ep_get_stat_tx() == USB_EP_STAT_NAK)
			// hardware will have toggled DTOG
			usb_ep_clr_ctr_tx(ep);
		}

		if (USB.EPR[ep] & USB_EPR_CTR_RX) {
			// OUT transaction
			// assert(usb_ep_get_stat_rx() == USB_EP_STAT_NAK)
			usb_ep_clr_ctr_rx(ep);

			size_t len = buf2fifo(ep, fifo);
			usb_ep_set_stat_rx(ep, USB_EP_STAT_VALID);
			return len;
		}
	}

	return 0;
}

size_t usb_send(struct Fifo* fifo) {
	// still not done with the previous one, or perhaps unconfigured
	if (usb_ep_get_stat_tx(1) != USB_EP_STAT_NAK) {
		return 0;
	}

	if (fifo_empty(fifo)) { 
		return 0;
	}

	size_t len = fifo2buf(1, fifo);
	usb_ep_set_stat_tx(1, USB_EP_STAT_VALID);

	return len;
}

// Setup and standard request handling


// CDC-ACM flavor: enumerates as a virtual serial port (/dev/cu.usbmodem* on
// mac, /dev/ttyACM* on linux) — no special host client needed. The config
// descriptor set is exactly the minimal ACM: header + ACM + union functional
// descriptors (the optional call-management one is dropped), one interrupt
// notification endpoint that never speaks (held at NAK), and the same EP1
// bulk pair carrying the serial payload. That comes to 62 bytes — it DOES
// fit the single 64-byte control packet, so the one-packet control flow
// below carries over unchanged.

static const uint8_t _deviceDescriptor[] = {
		18,				// length of this descriptor
		0x01,			// DEVICE Descriptor Type
		0x00, 0x02,		// USB version 2.00
		0x02,			// Device Class: Communications (CDC)
		0,	  0,		// subclass, protocol 0,0
		64,				// Max Packet Size ep0
		0x83, 0x04,		// VendorID  = 0x0483 (STMicroelectronics)
		0x40, 0x57,		// ProductID = 0x5740 (Virtual COM Port)
		0x00, 0x02,		// Device Version 2.0
		1,	  2,	3,	// iManufacturer/iProduct/iSerialNumber string indices
		1,				// NumConfigurations
};

static const uint8_t _langidDescriptor[] = {4, 0x03, 0x09, 0x04}; // string 0: en-US

static const uint8_t _configDescriptor[] = {
		// Config 0 header
		9,		//  Length
		0x02,	//  CONFIGURATION Descriptor Type
		62, 0,	//  TotalLength: 9+9+5+4+5+7+9+7+7
		2,		//  NumInterfaces
		1,		//  ConfigurationValue
		0,		//  Configuration string not set
		0x80,	//  Attributes 0x80 for historical reasons
		50,		//  MaxPower 100mA

		// interface 0: communications, ACM
		9,	   // Length
		0x04,  // INTERFACE Descriptor Type
		0, 0,  // Interface Number, Alternate Setting
		1,	   // Num Endpoints (the notification pipe)
		0x02,  // InterfaceClass: Communications
		0x02,  // InterfaceSubClass: Abstract Control Model
		0,	   // InterfaceProtocol
		0,	   // Interface string not set

		// CDC functional descriptors (class-specific, type 0x24)
		5, 0x24, 0x00, 0x10, 0x01,	// header: CDC 1.10
		4, 0x24, 0x02, 0x02,		// ACM: caps = line coding + control line state
		5, 0x24, 0x06, 0x00, 0x01,	// union: control if 0, data if 1

		// endpoint 0x82: notification (never used, held at NAK)
		7,		//  Length
		0x05,	//  ENDPOINT Descriptor Type
		0x82,	//  Endpoint Address: 2-IN
		0x03,	//  Attributes: INTERRUPT
		8, 0,	//  MaxPacketSize
		16,		//  Interval, ms

		// interface 1: data
		9,	   // Length
		0x04,  // INTERFACE Descriptor Type
		1, 0,  // Interface Number, Alternate Setting
		2,	   // Num Endpoints
		0x0A,  // InterfaceClass: USB_CLASS_DATA
		0,	   // InterfaceSubClass
		0,	   // InterfaceProtocol
		0,	   // Interface string not set

		// endpoint 0x1
		7,		//  Length
		0x05,	//  ENDPOINT Descriptor Type
		0x01,	//  Endpoint Address: 1-OUT
		0x02,	//  Attributes: BULK
		64, 0,	//  MaxPacketSize
		0,		//  Interval, ignored for BULK

		// endpoint 0x81
		7,		//  Length
		0x05,	//  ENDPOINT Descriptor Type
		0x81,	//  Endpoint Address 1-IN
		0x02,	//  Attributes: BULK
		64, 0,	//  MaxPacketSize
		0,		//  Interval, ignored for BULK
};

// the line coding (wire layout, little-endian baud) as last set by the
// host; 115200 8N1 until told otherwise
static uint8_t _line_coding[7] = {0x00, 0xC2, 0x01, 0, 0, 0, 8};

int usb_dtr(void) { return (_ctrl_line_state & 1) != 0; }



enum {
	REQ_TYPE_TX = 1 << 7,  // bit 7 direction: 1: device->host

   	//	REQ_TYPE_VENDOR 		= 1<<6, // bits 6..5 : type
   	//	REQ_TYPE_CLASS 			= 1<<5, //  00 = standard, 11 is reserved

	//	REQ_TYPE_DEVICE 		=  0x00,
	REQ_TYPE_INTERFACE = 0x01,
	REQ_TYPE_ENDPOINT  = 0x02,
	REQ_TYPE_OTHER	   = 0x03,

	// USB Standard Request Codes - Table 9-4
	REQ_GET_STATUS	= (0 << 8) | REQ_TYPE_TX,  // return 1: self-powered, 2: remote wakeup
	REQ_CLR_FEATURE = (1 << 8),				   // 1 remote wakeup-enable, 2: test mode (high-speed only)
	REQ_SET_FEATURE = (3 << 8),
	REQ_SET_ADDRESS = (5 << 8),	 // device only
	REQ_GET_DESCRIPTOR = (6 << 8) | REQ_TYPE_TX,	 // device only

	 //	REQ_SET_DESCRIPTOR 		= (7<<8),                // device only       // we don't support
	REQ_GET_CONFIGURATION = (8 << 8) | REQ_TYPE_TX,	 // device only       // return state == USB_CONFIGURED ? 1 : 0
	REQ_SET_CONFIGURATION = (9 << 8),  // device only       // 0-> state to ADDRESS,  1 -> state to CONFIGURED (and configure)
	REQ_GET_INTERFACE	  = (10 << 8) | REQ_TYPE_TX | REQ_TYPE_INTERFACE,  // interface only
	REQ_SET_INTERFACE	  = (11 << 8) | REQ_TYPE_INTERFACE,				   // interface only
	//	REQ_SYNC_FRAME 			= (12<<8)               | REQ_TYPE_ENDPOINT,  // endpoint, synch mode only, not supported

	REQ_GET_STATUS_INTERFACE  = REQ_GET_STATUS | REQ_TYPE_INTERFACE,   // return 0x0000
	REQ_CLR_FEATURE_INTERFACE = REQ_CLR_FEATURE | REQ_TYPE_INTERFACE,  // noop
	REQ_SET_FEATURE_INTERFACE = REQ_SET_FEATURE | REQ_TYPE_INTERFACE,  // noop

	REQ_GET_STATUS_ENDPOINT	 = REQ_GET_STATUS | REQ_TYPE_ENDPOINT,	 // return 0x1 if feature 'HALT' is set
	REQ_CLR_FEATURE_ENDPOINT = REQ_CLR_FEATURE | REQ_TYPE_ENDPOINT,	 // clear HALT (wvalue = 0), windex = 0x008f (dir/epnr)
	REQ_SET_FEATURE_ENDPOINT = REQ_SET_FEATURE | REQ_TYPE_ENDPOINT,	 // set HALT

	// CDC PSTN class requests (bmRequestType 0x21/0xA1: class, to interface)
	REQ_TYPE_CLASS_IF = (1 << 5) | REQ_TYPE_INTERFACE,
	REQ_SET_LINE_CODING		   = (0x20 << 8) | REQ_TYPE_CLASS_IF,				// 7-byte DATA-OUT stage
	REQ_GET_LINE_CODING		   = (0x21 << 8) | REQ_TYPE_CLASS_IF | REQ_TYPE_TX, // 7-byte reply
	REQ_SET_CONTROL_LINE_STATE = (0x22 << 8) | REQ_TYPE_CLASS_IF,				// wValue: DTR|RTS
};

// Cf. USB2.0 sections 5.5.5 and 8.5.3 there are 3 possible control flows:
// req.len > 0 &&  (req.req & REQ_TYPE_TX)  rx:SETUP (-> tx:DATA-IN-PART)*  -> tx:DATA-IN  -> rx:STATUS-OUT
// req.len > 0 && !(req.req & REQ_TYPE_TX)  rx:SETUP (-> rx:DATA-OUT-PART)* -> rx:DATA-OUT -> tx:STATUS-IN (zero lenght packet)
// (req.len == 0)                           rx:SETUP  -> tx:STATUS-IN  (zero lenght packet)
// Cf. USB2.0 section 9.4,  the second one (DATA-OUT/STATUS-IN) we don't need to support because it is only
// required by SET_DESCRIPTOR(device), which we don't support.
// Another simplification follows from the fact that the responses for the first flow are 1, 2 or
// len(descriptor) bytes, which in our case is always shorter than 64 (the pipe packet size),
// so we always can send the response in 1 go.
// Therefore, we have 3 events: rx:SETUP, tx:DATA-IN, rx:STATUS-OUT

// The most recently received SETUP request
struct {
	uint16_t req;  // lower byte: Type, upper byte request code
	uint16_t val;
	uint16_t idx;
	uint16_t len;
} _ctrl_req = {0, 0, 0, 0};

// false on failure, true on success
static int handle_set_request(void) {
	switch (_ctrl_req.req) {
	case REQ_SET_ADDRESS:
		// actually setting the address is handled after the ACK has been sent
		// here we just check for validity
		return (_usb_state != USB_CONFIGURED) && (_ctrl_req.val < 128) && (_ctrl_req.idx == 0);

	case REQ_SET_CONFIGURATION:
		switch (_usb_state) {
		case USB_ADDRESS:
			switch (_ctrl_req.val) {
			case 1:
				// configure our endpoint 0x01/0x81
				usb_ep_config(1, USB_EP_TYPE_BULK, 0x01);
				usb_ep_set_stat_rx(1, USB_EP_STAT_VALID);
				usb_ep_set_stat_tx(1, USB_EP_STAT_NAK);
				// the notification pipe: configured but forever silent (NAK)
				usb_ep_config(2, USB_EP_TYPE_INTERRUPT, 0x02);
				usb_ep_set_stat_tx(2, USB_EP_STAT_NAK);
				_usb_state = USB_CONFIGURED;
				// fallthrough
			case 0:
				return 1;
			}
			return 0;

		case USB_CONFIGURED:
			switch (_ctrl_req.val) {
			case 0:
				// unconfigure our endpoints 0x01/0x81
				usb_ep_reset(1);
				_usb_state = USB_ADDRESS;
				// fallthrough
			case 1:
				return 1;
			}
		default:
			break;
		}
		return 0;

	case REQ_SET_INTERFACE:
		return (_usb_state == USB_CONFIGURED) && (_ctrl_req.idx == 0);

	case REQ_SET_CONTROL_LINE_STATE:
		_ctrl_line_state = _ctrl_req.val;
		return 1;
	// case REQ_CLR_FEATURE:
	// case REQ_SET_FEATURE:
	// case REQ_CLR_FEATURE_INTERFACE:
	// case REQ_SET_FEATURE_INTERFACE:
	//    return 0; // no features implemented at device or interface level

	case REQ_CLR_FEATURE_ENDPOINT:
		if ((_usb_state != USB_CONFIGURED) || ((_ctrl_req.idx & 0xf) != 1)) {
			return 0;
		}
		if (_ctrl_req.idx & 0x80) {
			usb_ep_set_stat_tx(1, USB_EP_STAT_NAK);
			usb_ep_clr_dtog_tx(1);
		} else {
			usb_ep_set_stat_rx(1, USB_EP_STAT_NAK);
			usb_ep_clr_dtog_rx(1);
		}
		return 1;

	case REQ_SET_FEATURE_ENDPOINT:
		if ((_usb_state != USB_CONFIGURED) || ((_ctrl_req.idx & 0xf) != 1)) {
			return 0;
		}
		if (_ctrl_req.idx & 0x80) {
			usb_ep_set_stat_tx(1, USB_EP_STAT_STALL);
		} else {
			usb_ep_set_stat_rx(1, USB_EP_STAT_STALL);
		}
		return 1;
	}

	return 0;
}

// String descriptors are built directly in the EP0 packet memory: the PMA
// is halfword-addressed, so writing one ASCII char per halfword IS the
// UTF-16LE encoding — nothing is stored beyond the two const char pointers.
static size_t write_string_desc(const char *s) {
	volatile uint16_t *dst = usb_ep_tx_buf(0);
	size_t n = 0;
	while (s[n] != 0 && n < 31) { // bLength = 2 + 2n must fit the 64-byte buffer
		n++;
	}
	dst[0] = (uint16_t)((2 + 2 * n) | (0x03 << 8)); // bLength | STRING
	for (size_t i = 0; i < n; i++) {
		dst[1 + i] = (uint8_t)s[i];
	}
	usb_ep_set_tx_count(0, (uint16_t)(2 + 2 * n));
	return 2 + 2 * n;
}

// the serial number: the 96-bit unique device id (RM0440 48.1), 24 hex digits
static size_t write_serial_desc(void) {
	volatile uint16_t *dst = usb_ep_tx_buf(0);
	const uint32_t uid[3] = {DEVSIG.UID2, DEVSIG.UID1, DEVSIG.UID0};
	dst[0] = (uint16_t)((2 + 2 * 24) | (0x03 << 8));
	for (int i = 0; i < 24; i++) {
		dst[1 + i] = (uint8_t)"0123456789ABCDEF"[(uid[i / 8] >> (28 - 4 * (i % 8))) & 0xF];
	}
	usb_ep_set_tx_count(0, 2 + 2 * 24);
	return 2 + 2 * 24;
}

// copy buf[:sz] to the tx packet buffer of endpoint ep
static inline size_t write_buffer(uint8_t ep, const uint8_t *buf, size_t len) {
	struct Fifo f = {(uint8_t*)buf, 0x3f, len, 0}; // hack
	return fifo2buf(ep, &f);
}

// false on failure, true on success
static int handle_get_request(void) {
	uint8_t data[2] = {0, 0};
	size_t	len		= 0;

	switch (_ctrl_req.req) {
	case REQ_GET_DESCRIPTOR:
		switch (_ctrl_req.val) {
		case 0x0100:
		case 0x0200:
			if (_ctrl_req.idx != 0) {
				return 0;
			}
			len = (_ctrl_req.val == 0x0100) ? write_buffer(0, _deviceDescriptor, sizeof _deviceDescriptor)
			                                : write_buffer(0, _configDescriptor, sizeof _configDescriptor);
			break;
		case 0x0300: // string 0: supported language ids
			len = write_buffer(0, _langidDescriptor, sizeof _langidDescriptor);
			break;
		case 0x0301: // strings 1..3: wIndex carries the language id, any accepted
			len = write_string_desc(_manufacturer);
			break;
		case 0x0302:
			len = write_string_desc(_product);
			break;
		case 0x0303:
			len = write_serial_desc();
			break;
		default:
			return 0;
		}
		if (len > _ctrl_req.len) {
			usb_ep_set_tx_count(0, _ctrl_req.len);
		}
		return 1;

	case REQ_GET_STATUS:
		if ((_usb_state == USB_DEFAULT) || (_ctrl_req.val != 0) || (_ctrl_req.idx != 0) || (_ctrl_req.len != 2)) {
			return 0;
		}
		break;

	case REQ_GET_STATUS_INTERFACE:
		if ((_usb_state == USB_DEFAULT) || (_ctrl_req.val != 0) || (_ctrl_req.idx != 0) || (_ctrl_req.len != 2)) {
			return 0;
		}
		break;

	case REQ_GET_STATUS_ENDPOINT:
		if ((_usb_state == USB_DEFAULT) || (_ctrl_req.val != 0) || (_ctrl_req.len != 2)) {
			return 0;
		}
		switch (_ctrl_req.idx) {
		case 0x00:
			data[0] = (usb_ep_get_stat_rx(0) == USB_EP_STAT_STALL) ? 1 : 0;
			break;
		case 0x80:
			data[0] = (usb_ep_get_stat_tx(0) == USB_EP_STAT_STALL) ? 1 : 0;
			break;
		case 0x01:
			data[0] = (usb_ep_get_stat_rx(1) == USB_EP_STAT_STALL) ? 1 : 0;
			break;
		case 0x81:
			data[0] = (usb_ep_get_stat_tx(1) == USB_EP_STAT_STALL) ? 1 : 0;
			break;
		default:
			return 0;
		}
		break;

	case REQ_GET_CONFIGURATION:
		if ((_usb_state == USB_DEFAULT) || (_ctrl_req.len != 1)) {
			return 0;
		}
		data[0] = (_usb_state == USB_CONFIGURED) ? 1 : 0;
		break;

	case REQ_GET_INTERFACE:
		if ((_usb_state != USB_CONFIGURED) || (_ctrl_req.len != 1)) {
			return 0;
		}
		break;

	case REQ_GET_LINE_CODING:
		return write_buffer(0, _line_coding, sizeof _line_coding) == _ctrl_req.len;
	}

	return write_buffer(0, data, _ctrl_req.len) == _ctrl_req.len;
}

static void handle_ep0(void) {
	switch (USB.EPR[0] & (USB_EPR_CTR_RX | USB_EPR_SETUP | USB_EPR_CTR_TX)) {
	case USB_EPR_CTR_RX | USB_EPR_SETUP:

		// assert(usb_ep_get_stat_tx() == USB_EP_STAT_NAK)
		// assert(usb_ep_get_stat_rx() == USB_EP_STAT_NAK)
		// assert dir == 1

		if (usb_ep_get_rx_count(0) != 8) {
			break;
		}

		// this copying relies on the platform being little-endian, 
		// like the native transfer in usb
		const volatile uint16_t *src = usb_ep_rx_buf(0);
		_ctrl_req.req = src[0];
		_ctrl_req.val = src[1];
		_ctrl_req.idx = src[2];
		_ctrl_req.len = src[3];
		usb_debug.setups++;
		usb_debug.last_req = _ctrl_req.req;
		usb_debug.last_val = _ctrl_req.val;
		usb_debug.last_len = _ctrl_req.len;

		usb_ep_clr_ctr_rx(0);
		_ctrl_out_pending = 0; // a new SETUP aborts any expected DATA-OUT

		// non-zero length request with direction OUT: only CDC's
		// SET_LINE_CODING (7 bytes); accept its DATA-OUT stage, the
		// payload lands in the CTR_RX case below
		if ((_ctrl_req.len > 0) && !(_ctrl_req.req & REQ_TYPE_TX)) {
			if (_ctrl_req.req != REQ_SET_LINE_CODING || _ctrl_req.len != sizeof _line_coding) {
				break;
			}
			_ctrl_out_pending = 1;
			usb_ep_set_stat_rx(0, USB_EP_STAT_VALID);
			return;
		}

		if (_ctrl_req.len == 0) {
			if (!handle_set_request()) {
				break;
			}
			usb_ep_set_tx_count(0, 0);	// ZLP status-in reply
		} else {
			if (!handle_get_request()) {  // sets up reply buffer
				break;
			}
		}

		usb_ep_set_stat_tx(0, USB_EP_STAT_VALID);
		return;

	case USB_EPR_CTR_TX:  // USB IN: sent the reply for the most recent GET or the ACK for the most recent SET

		usb_debug.ep0_tx++;
		usb_ep_clr_ctr_tx(0);

		if (_ctrl_out_pending) {
			_ctrl_out_pending = 0; // sent the STATUS-IN ACK for the DATA-OUT flow
			return;
		}
		if (_ctrl_req.len == 0) {
			// last request was a SET, so we are here because we sent the ACK
			// if the request was set_address, we should execute it here
			if (_ctrl_req.req == REQ_SET_ADDRESS) {
				usb_daddr_add_set(_ctrl_req.val);
				USB.DADDR |= USB_DADDR_EF;
				_usb_state = (_ctrl_req.val == 0) ? USB_DEFAULT : USB_ADDRESS;
			}
		} else {
			// last request was a GET, so we are here because we sent the reply
			// next thing should be the RX of the host's STATUS_OUT
			// the hardware has a special mechanism to deal with this when we set the 'kind' bit
			usb_ep_set_stat_rx(0, USB_EP_STAT_VALID);
		}

		return;

	case USB_EPR_CTR_RX:  // RX, USB OUT the final status zero lenght reply from the host to the GET

		if (_ctrl_out_pending) { // the SET_LINE_CODING payload
			if (usb_ep_get_rx_count(0) != sizeof _line_coding) {
				break;
			}
			const volatile uint16_t *lsrc = usb_ep_rx_buf(0);
			for (size_t i = 0; i < sizeof _line_coding; i++) {
				_line_coding[i] = (uint8_t)(lsrc[i / 2] >> (8 * (i % 2)));
			}
			usb_ep_clr_ctr_rx(0);
			usb_ep_set_tx_count(0, 0); // ZLP STATUS-IN
			usb_ep_set_stat_tx(0, USB_EP_STAT_VALID);
			return;
		}
		if (usb_ep_get_rx_count(0) != 0) {
			break;
		}

		usb_ep_clr_ctr_rx(0);  // should be done automatically by STATUS_OUT mechanism, but apparently not

		return;
	}

	// if we got here we didn't handle the transfer, abort it
	usb_debug.stalls++;
	usb_ep_set_stat_rx(0, USB_EP_STAT_STALL);
	usb_ep_set_stat_tx(0, USB_EP_STAT_STALL);

	return;
}
