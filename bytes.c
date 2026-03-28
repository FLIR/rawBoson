#include "bytes.h"


// Convert single ascii_HEX to Int
// x1 to Int
int toInt(unsigned char mybyte) {
  if ( (mybyte>='0') && (mybyte<='9') ) {
    mybyte=mybyte-'0';
  } else if ( (mybyte>='A') && (mybyte<='F')) {
    mybyte=mybyte-'A'+10;
  } else if ( (mybyte >= 'a') && (mybyte<='f') ) {
    mybyte=mybyte-'a'+10;
  } else {  // Error , no HEX
    return -1;
  }
  return mybyte;
}


// Convert two ascii_HEX to Int
// x12 to Int
int hex_to_int(const char *str) {
  int i, j = 0;
  size_t len;
  int value = 0;
  unsigned char Nibble;
  int intNibble;

  // Read size of STR
  len = strlen(str);

  // Check SIZE of command
  if (len == 0 || len > 2) {
    return -1;
  }

  // Convert to int
  for (i = (int)len - 1; i >= 0; i--) {
    Nibble = (unsigned char)str[i];
    intNibble = toInt(Nibble);
    if (intNibble < 0) {
      return -2; // Error with the conversion
    } else {
      value |= (intNibble << (4 * j++));
    }
  }
  //printf("value=%i \n", value);
  return value;
}


// Convert 8 ascii_HEX to 32-bit unsigned
// x12345678 to value
int hex_to_uint32(const char *str, uint32_t *out) {
  int i, j = 0;
  size_t len;
  uint32_t value = 0;
  unsigned char Nibble;
  int intNibble;

  if (out == NULL) {
    return -1;
  }

  // Read size of STR
  len = strlen(str);

  // Check SIZE of command
  if (len == 0 || len > 8) {
    return -1;
  }

  // Convert to uint32
  for (i = (int)len - 1; i >= 0; i--) {
    Nibble = (unsigned char)str[i];
    intNibble = toInt(Nibble);
    if (intNibble < 0) {
      return -2; // Error with the conversion
    } else {
      value |= ((uint32_t)intNibble << (4 * j++));
    }
  }

  *out = value;
  return 0;
}
