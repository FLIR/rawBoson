#ifndef BYTES_H
#define BYTES_H

#include <string.h>
#include <math.h>   // For POW function

int toInt( unsigned char mybyte);  // Single ASCII Hex caracter

int hex_to_int(char *str);  // BYTE in ASCII HEX

long hex_to_long(char *str);  // Long in ASCII HEX

#endif
