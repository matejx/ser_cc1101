// MIT License - Copyright (c) Matej Kogovsek
// This file is subject to the terms and conditions defined in
// LICENSE, which is part of this source code package

#include <inttypes.h>
#include <util/delay.h>
#include "cc1101_def.h"
#include "cc1101.h"
#include "mat/spi.h"

extern void spi_cs(uint8_t);

void cc_init(uint8_t spi_fdiv)
{
	spi_init(spi_fdiv);
	spi_mode(0);
	spi_cs(1);
}

uint8_t cc_xreg(uint8_t pre, uint8_t reg, uint8_t data)
{
	spi_cs(0);
	spi_rw(pre | reg);
	uint8_t r = spi_rw(data);
	spi_cs(1);
	return r;
}

uint8_t cc_wreg(uint8_t reg, uint8_t data)
{
	return cc_xreg(CC_WRITE, reg, data);
}

uint8_t cc_rreg(uint8_t reg)
{
	return cc_xreg(CC_READ, reg, 0);
}

uint8_t cc_cmds(uint8_t cmd)
{
	spi_cs(0);
	uint8_t r = spi_rw(CC_WRITE | cmd);
	spi_cs(1);
	return r;
}

uint8_t cc_sreg(uint8_t reg, uint8_t* stat)
{
	spi_cs(0);
	uint8_t s = spi_rw(CC_READ_BURST | reg);
	uint8_t r = spi_rw(0);
	spi_cs(1);
	if( stat ) *stat = s;
	return r;
}

void cc_wpat(const uint8_t* pat, uint8_t len)
{
	spi_cs(0);
	spi_rw(CC_WRITE_BURST | CC_PATABLE);
	while( len-- ) {
		spi_rw(*pat++);
	}
	spi_cs(1);
}

uint8_t cc_txf(const struct cc_pkt_t* pkt)
{
	if( pkt->len == 0 ) {
		return 1;
	}

	uint8_t txb = cc_sreg(CC_TXBYTES, 0) & 0x7f;
	if( txb > CC_FIFO_SIZE ) { // should not happen, but mask allows it
		return 2;
	}
	if( (pkt->len + CC_PKT_OVERHEAD) > (CC_FIFO_SIZE - txb) ) {
		return 3;
	}

	spi_cs(0);
	spi_rw(CC_WRITE_BURST | CC_TXFIFO);
	spi_rw(pkt->len);
	for( uint8_t i = 0; i < pkt->len; ++i ) {
		spi_rw(pkt->data[i]);
	}
	spi_cs(1);

	return 0;
}

uint8_t cc_rxf(struct cc_pkt_t *pkt)
{
	spi_cs(0);
	uint8_t status = spi_rw(CC_READ_BURST | CC_RXFIFO);
	if( (status & CC_STATE_MASK) != CC_STATE_IDLE ) {
		spi_cs(1);
		return 1; // not IDLE (packet reception might not be complete yet)
	}
	if( (status & 0x0f) == 0 ) {
		spi_cs(1);
		return 2; // no bytes available in RXFIFO
	}
	pkt->len = spi_rw(0); // 1st byte is packet length
	if( pkt->len > CC_PKT_MAXLEN ) { // something went seriously wrong, clear RXFIFO and return
		spi_cs(1);
		cc_cmds(CC_SFRX);
		return 3;
	}
	for( uint8_t i = 0; i < pkt->len; ++i ) {
		pkt->data[i] = spi_rw(0);
	}
	pkt->rssi = spi_rw(0);
	pkt->lqi = spi_rw(0);
	spi_cs(1);
	pkt->crc_ok = pkt->lqi & 0x80; // extract crc_ok bit from lqi
	pkt->lqi &= 0x7f; // remove crc_ok bit from lqi

	return 0;
}
