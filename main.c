// MIT License - Copyright (c) Matej Kogovsek
// This file is subject to the terms and conditions defined in
// LICENSE, which is part of this source code package

#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include <util/delay.h>

#include "mat/spi.h"
#include "mat/serque.h"
#include "mat/gpio.h"

#include "cc1101.h"
#include "cc1101_def.h"

#define LED_PIN NPORTD+4
#define SS_PIN NPORTB+2

// SmartRF studio, Base f. 433.92, Xtal 26, Mod GFSK, Chan 0, Rate 9.6, Dev 20, Spac 100, RX BW 70
#define cc_RFSET_LEN 22
const uint8_t cc_RFSETTINGS[cc_RFSET_LEN][2] = {
{CC_SYNC1,122},
{CC_SYNC0,14},
{CC_FSCTRL1,6},
{CC_FREQ2,16},
{CC_FREQ1,176},
{CC_FREQ0,113},
{CC_MDMCFG4,232},
{CC_MDMCFG3,131},
{CC_MDMCFG2,22},
{CC_MDMCFG1,33},
{CC_DEVIATN,52},
{CC_FOCCFG,22},
{CC_AGCCTRL2,67},
{CC_AGCCTRL1,73},
{CC_WORCTRL,251},
{CC_FSCAL3,233},
{CC_FSCAL2,42},
{CC_FSCAL1,0},
{CC_FSCAL0,31},
{CC_TEST2,129},
{CC_TEST1,53},
{CC_TEST0,9}
};

#define cc_SET_LEN 6
const uint8_t cc_SETTINGS[cc_SET_LEN][2] = {
{CC_IOCFG0,0x06},
{CC_FIFOTHR,47},
{CC_PKTLEN,CC_PKT_MAXLEN},
{CC_PKTCTRL0,0x45},
{CC_MCSM0,0x18},
{CC_FREND0,0x7}
};

// CC1101 datasheet, page 60, 433 MHz
#define cc_PAT_LEN 8
const uint8_t cc_PATABLE[cc_PAT_LEN] = {0x12,0x0e,0x1d,0x34,0x60,0x84,0xc8,0xc0};

uint8_t rxbuf[80];
uint8_t txbuf[80];

volatile uint8_t rxto_ms = 0;
volatile uint8_t ledto_ms = 0;
volatile uint8_t calto_s = 0;

void spi_cs(uint8_t a)
{
	gpio_set(SS_PIN, a);
}

uint8_t cc_wregs(const uint8_t ra[][2], uint8_t len)
{
	for(uint8_t i = 0; i < len; ++i ) {
		uint8_t reg = ra[i][0];
		uint8_t val = ra[i][1];
		cc_wreg(reg, val);
		if( val != cc_rreg(reg) ) {
			ser_puts_P(0, PSTR("reg vrf\r\n"));
			return i+1;
		}
	}
	return 0;
}

void hprintbuf(const uint8_t* buf, uint8_t len)
{
	for( uint8_t i = 0; i < len; ++i ) {
		ser_puti_lc(0, buf[i], 16, 2, '0');
		ser_putc(0, ' ');
	}
	ser_puts_P(0, PSTR("\r\n"));
}

void printbuf(const uint8_t* buf, uint8_t len)
{
	for( uint8_t i = 0; i < len; ++i ) {
		ser_putc(0, buf[i]);
	}
}

void led_on(uint8_t ms)
{
	ledto_ms = ms;
	gpio_set(LED_PIN, 1);
}

int main(void)
{
	gpio_dir(LED_PIN, 1);
	gpio_dir(SS_PIN, 1);
	cc_init(SPI_FDIV_32);
	ser_init(0, BAUD_9600, txbuf, sizeof(txbuf), rxbuf, sizeof(rxbuf));

	// Timer0
	TCCR0A = 0;
	TCCR0B = _BV(CS01) | _BV(CS00); // prescaler 64
	TIMSK0 |= _BV(TOIE0);	// enable overflow interrupt

	wdt_enable(WDTO_2S);

	sei();
	//ser_puts_P(0, PSTR("reset\r\n"));

	_delay_ms(10);
	cc_cmds(CC_SRES);
	_delay_ms(10);

	cc_wregs(cc_RFSETTINGS, cc_RFSET_LEN);
	cc_wpat(cc_PATABLE, cc_PAT_LEN);
	cc_wregs(cc_SETTINGS, cc_SET_LEN);

	struct cc_pkt_t rxpkt;
	struct cc_pkt_t txpkt;
	txpkt.len = 0;

	while( 1 ) {
		wdt_reset();
		// check RX FIFO
		uint8_t ec = cc_rxf(&rxpkt);
		if( ec == 0 ) { // packet received
			if( rxpkt.crc_ok ) {
				printbuf(rxpkt.data, rxpkt.len);
			}
		}
		if( ec != 1 ) { // state is IDLE - restart RX
			cc_cmds(CC_SRX);
			calto_s = 60; // transition from IDLE to RX starts FS cal, reset timeout
		}
		// transfer serial RX queue into txpkt
		if( txpkt.len < CC_PKT_MAXLEN ) {
			uint8_t d;
			while( ser_getc(0, &d) ) {
				txpkt.data[txpkt.len++] = d;
				rxto_ms = 10;
				if( txpkt.len == CC_PKT_MAXLEN ) break;
			}
		}
		// if txpkt full or serial RX timeout fill TX FIFO
		if( (txpkt.len == CC_PKT_MAXLEN) || (txpkt.len && !rxto_ms) ) {
			if( cc_txf(&txpkt) == 0 ) {
				txpkt.len = 0;
			}
		}
		// attempt TX if state = RX and bytes in TX FIFO
		// CC does not necessarily enter TX mode (depends on CCA), so do this repeatedly
		uint8_t state = cc_cmds(CC_SNOP) & CC_STATE_MASK;
		if( state == CC_STATE_RX ) {
			if( cc_sreg(CC_TXBYTES, 0) & 0x7f ) {
				cc_cmds(CC_STX);
			} else
			if( calto_s == 0 ) {
				cc_cmds(CC_SIDLE); // go to idle, so that FS get recalibrated
			}
		} else
		if( state == CC_STATE_TX ) {
			led_on(100);
		}
	}
}

// Timer0 overflow
// interrupts every 1 ms
//  8000000/64/125 = 1000
// 16000000/64/250 = 1000
ISR(TIMER0_OVF_vect)
{
#if F_CPU == 8000000
	TCNT0 = 0xff - 125;
#elif F_CPU == 16000000
	TCNT0 = 0xff - 250;
#else
	#error Calculate TCNT0 for your F_CPU
#endif
	static uint16_t ms = 0;
	// serial rx timeout
	if( rxto_ms ) {
		--rxto_ms;
	}
	// LED on timeout
	if( ledto_ms ) {
		--ledto_ms;
	} else {
		gpio_set(LED_PIN, 0);
	}
	// second timers
	if( ms-- == 0 ) {
		ms = 1000;
		// calibration timeout
		if( calto_s ) {
			--calto_s;
		}
	}
}
