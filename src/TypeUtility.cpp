#include "TypeUtility.h"
#include "Arduino.h"

void print_hexdump(uint8_t *data, uint8_t len)
{

  for (int i = 0; i < len; i++)
  {
    Serial.print(data[i], HEX);
    if (i == len - 1)
      Serial.println("");
  }
}


// Convert between float and bytes array
union float_byte_ui32 {
    float fl;
    uint8_t bt[sizeof(float)];
    uint32_t ui32;
};

/**
 * @brief Function for encoding a float value in big-endian format.
 *
 * @param[in]   f                Float value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data will be written.
 *
 * @return      Number of bytes written.
 */
uint8_t float_big_encode(float f, uint8_t * p_encoded_data)
{
    union float_byte_ui32 u;
    u.fl = f;
    uint32_t value = u.ui32;
    p_encoded_data[0] = (uint8_t) ((value & 0xFF000000) >> 24);
    p_encoded_data[1] = (uint8_t) ((value & 0x00FF0000) >> 16);
    p_encoded_data[2] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[3] = (uint8_t) ((value & 0x000000FF) >> 0);
    return sizeof(uint32_t);
}


/**@brief Function for decoding a float value in big-endian format.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
float float_big_decode(const uint8_t * p_encoded_data)
{
    union float_byte_ui32 u;
    u.ui32 = ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 24) |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 0) );
    return u.fl;
}

/**@brief Function for validation of float value.
 *
 * @param[in]   f            Value to be valiadated.
 * @return      return true if validated.
 *     0x7F800000: positive infinite
 *     0xFF800000: negative infinite
 *     0x7F800001: Signal NaN
 *     0x7FC00000: Quiet NaN
 */
bool float_validation(float f)
{
    if(f == 0x7F800000 || f == 0xFF800000 || f == 0x7F800001 || f == 0x7FC00000	){return false;}
    else { return true;}    
}


/**
 * @brief Function for encoding an uint16 value in big-endian format.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data will be written.
 *
 * @return      Number of bytes written.
 */
uint8_t uint32_big_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) (value >> 24);
    p_encoded_data[1] = (uint8_t) (value >> 16);
    p_encoded_data[2] = (uint8_t) (value >> 8);
    p_encoded_data[3] = (uint8_t) (value & 0xFF);

    return sizeof(uint32_t);
}


/**@brief Function for decoding a uint32 value in big-endian format.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
uint32_t uint32_big_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 24) |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 0) );
}

/**
 * @brief Function for encoding an uint16 value in big-endian format.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data will be written.
 *
 * @return      Number of bytes written.
 */
uint8_t uint16_big_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) (value >> 8);
    p_encoded_data[1] = (uint8_t) (value & 0xFF);

    return sizeof(uint16_t);
}

/**@brief Function for decoding a uint16 value in big-endian format.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
uint16_t uint16_big_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0]) << 8 ) |
                 (((uint16_t)((uint8_t *)p_encoded_data)[1])) );
}


/**@brief Function for encoding a uint16 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
uint8_t uint16_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}

/**@brief Function for decoding a uint16 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
uint16_t uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0])) |
                 (((uint16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}

/**
 * @brief Function for encoding an int32 value in big-endian format.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data will be written.
 *
 * @return      Number of bytes written.
 */
uint8_t int32_big_encode(int32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) (value >> 24);
    p_encoded_data[1] = (uint8_t) (value >> 16);
    p_encoded_data[2] = (uint8_t) (value >> 8);
    p_encoded_data[3] = (uint8_t) (value & 0xFF);

    return sizeof(int32_t);
}


/**@brief Function for decoding an int32 value in big-endian format.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
int32_t int32_big_decode(const uint8_t * p_encoded_data)
{
    return ( (((int32_t)((uint8_t *)p_encoded_data)[0]) << 24) |
             (((int32_t)((uint8_t *)p_encoded_data)[1]) << 16) |
             (((int32_t)((uint8_t *)p_encoded_data)[2]) << 8)  |
             (((int32_t)((uint8_t *)p_encoded_data)[3]) << 0) );
}



/**
 * @brief Function for encoding an uint16 value in big-endian format.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data will be written.
 *
 * @return      Number of bytes written.
 */
uint8_t int16_big_encode(int16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) (value >> 8);
    p_encoded_data[1] = (uint8_t) (value & 0xFF);

    return sizeof(int16_t);
}



/**@brief Function for decoding a uint16 value in big-endian format.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
int16_t int16_big_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0]) << 8 ) |
                 (((uint16_t)((uint8_t *)p_encoded_data)[1])) );
}
