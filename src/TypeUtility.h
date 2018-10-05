#ifndef typeutility_h
#define typeutility_h

#include <inttypes.h>

uint8_t float_big_encode(float f, uint8_t * p_encoded_data);
float float_big_decode(const uint8_t * p_encoded_data);
bool float_validation(float f);
uint8_t uint32_big_encode(uint32_t value, uint8_t * p_encoded_data);
uint32_t uint32_big_decode(const uint8_t * p_encoded_data);
uint8_t uint16_big_encode(uint16_t value, uint8_t * p_encoded_data);
uint16_t uint16_big_decode(const uint8_t * p_encoded_data);
uint8_t int32_big_encode(int32_t value, uint8_t * p_encoded_data);
int32_t int32_big_decode(const uint8_t * p_encoded_data);
uint8_t int16_big_encode(int16_t value, uint8_t * p_encoded_data);
int16_t int16_big_decode(const uint8_t * p_encoded_data);


#endif // typeutility_h

