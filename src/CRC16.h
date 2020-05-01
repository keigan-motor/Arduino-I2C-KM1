#ifndef CRC16_H
#define CRC16_H

#include <inttypes.h>

void crc16_init(void);

uint16_t crc16(uint8_t *buf, uint32_t len);
void crc16_encode(uint8_t *buf, uint32_t len, uint8_t *crc);
uint8_t append_crc16(uint8_t *buf, uint32_t len);

    
#endif // CRC16_H

