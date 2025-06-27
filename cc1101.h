#ifndef _CC1101_H
#define _CC1101_H

#define CC_FIFO_SIZE 64
#define CC_PKT_OVERHEAD 4
#define CC_PKT_MAXLEN (CC_FIFO_SIZE - CC_PKT_OVERHEAD)

struct cc_pkt_t
{
	uint8_t len;
	uint8_t data[CC_PKT_MAXLEN];
	uint8_t crc_ok;
	uint8_t rssi;
	uint8_t lqi;
};

void cc_init(uint8_t spi_fdiv);
uint8_t cc_wreg(uint8_t reg, uint8_t data);
uint8_t cc_rreg(uint8_t reg);
uint8_t cc_cmds(uint8_t cmd);
uint8_t cc_sreg(uint8_t reg, uint8_t* stat);
void cc_wpat(const uint8_t* pat, uint8_t len);
uint8_t cc_txf(const struct cc_pkt_t* pkt);
uint8_t cc_rxf(struct cc_pkt_t* pkt);

#endif
