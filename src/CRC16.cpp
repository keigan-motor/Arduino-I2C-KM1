#include "CRC16.h"
#include "TypeUtility.h"

uint16_t crc16_table[256];

 
//#define CRC16POLY 0xa001 // 右送り CRC-16-IBM (x16+x15+x2+1) ビット反転（初期値=0）
#define CRC16POLY 0x8408 //右送り CRC-16-CCITT (x16 + x12 + x5 + 1) ビット反転（初期値=0）


void crc16_init(void) {
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t c = i;
        for (int j = 0; j < 8; j++) {
            c = (c & 1) ? (CRC16POLY ^ (c >> 1)) : (c >> 1);
        }
        crc16_table[i] = c;
    }
}

/**@brief CRC16を計算する
 *  [送信側] CRC16 を データ列に付加して送信する 
 *  [受信側] [データ列+CRC16] のバイト列をこの関数にかけると、0が返却される = 誤りなしの証拠
 */
uint16_t crc16(uint8_t *buf, uint32_t len) {
    uint16_t c = 0;
    for (uint32_t i = 0; i < len; i++) {
        c = crc16_table[(c ^ buf[i]) & 0xFF] ^ (c >> 8);
    }
    return c;
}

/**@brief CRC16を計算してバイト列として返す
 */

void crc16_encode(uint8_t *buf, uint32_t len, uint8_t *crc_buffer) {
    uint16_t crc_value = crc16(buf, len);
    uint16_encode(crc_value, crc_buffer); // uint16_big_encode ではないので注意！！！
}

/**@brief CRC16を付加する（ [len + 1]までの メモリ破壊注意。）
 */
uint8_t append_crc16(uint8_t *buf, uint32_t len) {
    uint16_t crc_value = crc16(buf, len);
    uint16_encode(crc_value, &buf[len]); // uint16_big_encode ではないので注意！！！
    return sizeof(uint16_t);
}
