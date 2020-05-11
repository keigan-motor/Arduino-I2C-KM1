#ifndef typeutility_h
#define typeutility_h

#include <inttypes.h>

/**@brief Unit conversion */
#define PI (3.14159265358979f) // π: def. of PI
#define RPM_TO_RADPERSEC(rpm) ((float)(rpm)*2 * (PI) / 60)
#define RPM_TO_RADPERMILLIS(rpm) ((float)(rpm)*2 * (PI) / 60 / 1000)
#define RADPERSEC_TO_RPM(rps) ((float)(rps)*60 / (2 * (PI)))
#define RADPERMILLIS_TO_RPM(rpms) ((float)(rpms)*1000 * 60 / (2 * (PI)))
#define DEGREES_TO_RADIANS(deg) ((float)(deg) * (PI) / 180)
#define RADIANS_TO_DEGREES(rad) ((float)(rad)*180 / (PI))

void print_hexdump(uint8_t *data, uint8_t len);

uint8_t float_big_encode(float f, uint8_t * p_encoded_data);
float float_big_decode(const uint8_t * p_encoded_data);
bool float_validation(float f);
uint8_t uint32_big_encode(uint32_t value, uint8_t * p_encoded_data);
uint32_t uint32_big_decode(const uint8_t * p_encoded_data);
uint8_t uint16_big_encode(uint16_t value, uint8_t * p_encoded_data);
uint16_t uint16_big_decode(const uint8_t * p_encoded_data);
uint8_t uint16_encode(uint16_t value, uint8_t * p_encoded_data);
uint16_t uint16_decode(const uint8_t * p_encoded_data);
uint8_t int32_big_encode(int32_t value, uint8_t * p_encoded_data);
int32_t int32_big_decode(const uint8_t * p_encoded_data);
uint8_t int16_big_encode(int16_t value, uint8_t * p_encoded_data);
int16_t int16_big_decode(const uint8_t * p_encoded_data);


#endif // typeutility_h
