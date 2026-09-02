#pragma once

// U[S]ART init for the v3 (FIFO-capable) IP: STM32G4, H7. The L4/F7
// families carry the v2 IP without the FIFO — include lib/usart_v2.h there
// instead; everything else in lib/serial.h (struct Serial, the DMA/IRQ
// handlers, usart_putc) is IP-version-neutral because the v3 header keeps
// the legacy TXE/RXNE flag names.
//
// Split out of serial.h so that serial.h compiles against a generated
// device.h that (correctly) lacks USART_CR1_FIFOEN.

#include "serial.h"

// Full-duplex DMA RX+TX, 8N1, with receive-timeout flush of partial DMA bursts.
static inline void usart_init(struct USART_Type *u, uint32_t kernel_hz, uint32_t baud) {
	u->CR1 = USART_CR1_FIFOEN | USART_CR1_RTOIE;
	u->CR2 = USART_CR2_RTOEN;
	usart_rtor_rto_set(u, 160); // ~16 byte times at 8N1
	u->BRR = usart_brr(kernel_hz, baud);
	u->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
	u->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
}

static inline void usart_init_tx(struct USART_Type *u, uint32_t kernel_hz, uint32_t baud) {
	u->CR1 = USART_CR1_FIFOEN;
	u->CR2 = 0;
	u->BRR = usart_brr(kernel_hz, baud);
	u->CR3 = USART_CR3_DMAT;
	u->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

// LPUART shares the USART base layout, so the Serial struct and the DMA/TX
// handlers work on it via SERIAL_INITIALIZER_LP's cast — only the BRR differs.
// LPUART has no receive-timeout (RTOR), so RX is DMA-only (no flush handler);
// use usart_init for full-duplex USART/UART instead when timeout flush matters.
static inline void lpuart_init_tx(struct USART_Type *u, uint32_t kernel_hz, uint32_t baud) {
	u->CR1 = USART_CR1_FIFOEN;
	u->CR2 = 0;
	u->BRR = lpuart_brr(kernel_hz, baud);
	u->CR3 = USART_CR3_DMAT;
	u->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

static inline void usart_init_rx(struct USART_Type *u, uint32_t kernel_hz, uint32_t baud) {
	u->CR1 = USART_CR1_FIFOEN | USART_CR1_RTOIE;
	u->CR2 = USART_CR2_RTOEN;
	usart_rtor_rto_set(u, 160);
	u->BRR = usart_brr(kernel_hz, baud);
	u->CR3 = USART_CR3_DMAR;
	u->CR1 |= USART_CR1_UE | USART_CR1_RE;
}

// RS-485 half-duplex with the Driver-Enable pin (active high). Configure the TX
// pin as AF/open-drain; the RX pin is unused. RM0440 37.5.15.
static inline void usart_set_rs485(struct USART_Type *u) {
	u->CR1 &= ~USART_CR1_UE;
	usart_cr1_deat_set(u, 31); // 16 = one bit time, 31 = max
	usart_cr1_dedt_set(u, 31);
	u->CR3 |= USART_CR3_DEM | USART_CR3_HDSEL;
	u->CR1 |= USART_CR1_UE;
}
