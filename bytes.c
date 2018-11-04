#include "bytes.h"


// Convert single ascii_HEX to Int
// x1 to Int
int toInt( unsigned char mybyte) {
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
int hex_to_int(char *str) {
  int i,j=0;
  int len;
  int value=0;
  unsigned char Nibble;
  int intNibble;

  // Read size of STR
  len=strlen(str);

  // Check SIZE of command
  if  (len>2 ) {
    return -1;
  }

  // Convert to int
  for (i=len-1; i>=0; i=i-1) {
    Nibble = str[i];
    intNibble = toInt(Nibble);
    if ( intNibble < 0 ) {
      return -2; // Error with the convertion
    } else {
      value = value+(long)( intNibble * (pow(2,4*j++)) );
    }
  }
  //printf("value=%i \n", value);
  return value;
}


// Convert 8 ascii_HEX to long
// x12345678 to long
long hex_to_long(char *str) {
  int i,j=0;
  int len;
  long value=0;
  unsigned char Nibble;
  int intNibble;

  // Read size of STR
  len=strlen(str);

  // Check SIZE of command
  if  (len>8 ) {
    return -1;
  }

  // Convert to long
  for (i=len-1; i>=0; i=i-1) {
    Nibble = str[i];
    intNibble = toInt(Nibble);
    if ( intNibble < 0 ) {
      return -2; // Error with the convertion
    } else {
      value = value+(long)( intNibble * (pow(2,4*j++)) );
    }
  }
  //printf("value=%lu \n", value);   // DEBUG
  return value;
}
