#ifndef BYTES_H
#define BYTES_H

#include <stdint.h>
#include <string.h>
#include <math.h>   // For POW function

int toInt(unsigned char mybyte);  // Single ASCII Hex char

int hex_to_int(const char *str);  // BYTE in ASCII HEX

int hex_to_uint32(const char *str, uint32_t *out);  // 32-bit value in ASCII HEX

#endif
