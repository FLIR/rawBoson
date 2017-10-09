// version 1.1 : adding ascii mode. rawBoson a ....
// version 1.2 : fixing the send command. using buffer instead of byte by byte
// version 1.3 : Fixing the HELP menu to include working examples.
// version 1.4 : Translating to english

#ifndef RAWBOSON_H
#define RAWBOSON_H

#include "serial.h"
#include "bytes.h"
#include <unistd.h>
#include <time.h>

// Define version
#define version "v1.4"

// Define COLOR CODES
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

#define SetFontYellow()    printf(YEL);   /* Yellow */
#define SetFontWhite()     printf(WHT);   /* White */
#define SetFontRed()       printf(RED);   /* Red */
#define SetFontGreen()     printf(GRN);   /* Green */
#define SetFontBlue()      printf(BLU);   /* Blue */
#define SetFontCyan()      printf(CYN);   /* Cyan  */
#define SetFontMagenta()   printf(MAG);   /* Magenta  */
#define SetFontReset()     printf(RESET); /* Reset */

// Define error codes

#define NoError                           0x00

#define R_CAM_DSPCH_BAD_CMD_ID            0x0161
#define R_CAM_DSPCH_BAD_PAYLOAD_STATUS    0x0162
#define R_CAM_PKG_UNSPECIFIED_FAILURE     0x0170
#define R_CAM_PKG_INSUFFICIENT_BYTES      0x017D
#define R_CAM_PKG_EXCESS_BYTES            0x017E
#define R_CAM_PKG_BUFFER_OVERFLOW         0x017F
#define FLIR_RANGE_ERROR                  0x0203  

PortSettingsType puerto_serie_conf;
HANDLE serial;
void print_buffer(unsigned char *buffer, int bufferlen);

#endif
