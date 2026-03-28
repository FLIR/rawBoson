// --------------------------------------
// FLIR Systems 2017 (c)
// --------------------------------------

#include <stdint.h>
#include "rawBoson.h"

#define SERIAL_TIMEOUT 500
#define MAX_DATA_BYTES 751
#define MAX_COMMAND_HEX 8

typedef struct {
  // ..... Boson Package (Before bit stuffing) .....
  unsigned char aux_boson_package[768];
  unsigned short aux_boson_package_len;

  // ..... Boson Package (After bit stuffing) .....
  unsigned char boson_stuffed_package[1544];
  unsigned short boson_stuffed_package_len;

  // ..... Received Boson requested data .....
  unsigned char BosonData[768];
  unsigned short BosonData_count;
  unsigned char sequence;
} BosonContext;

/*
------------------------------------------------------------------------------
--- Aux functions: CRC, Serial port TX/RX , Gettickcount, Hex Operations   ---
------------------------------------------------------------------------------
*/

// Boson is using this method: CRC-16/AUG-CCITT
// https://github.com/meetanthony/crcphp/blob/master/crc16/crc_16_aug_ccitt.php
uint16_t CalcBlockCRC16(size_t bufferlen, const unsigned char *buffer)
{
  uint16_t ccitt_16Table[] =
  {
      0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5,
      0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B,
      0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210,
      0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
      0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C,
      0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401,
      0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B,
      0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
      0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6,
      0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738,
      0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5,
      0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
      0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969,
      0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96,
      0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
      0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
      0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03,
      0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD,
      0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6,
      0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
      0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A,
      0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB,
      0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1,
      0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
      0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C,
      0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2,
      0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB,
      0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
      0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447,
      0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
      0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2,
      0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
      0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9,
      0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827,
      0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C,
      0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
      0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0,
      0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D,
      0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07,
      0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
      0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA,
      0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
      0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
  };
  unsigned int i;
  unsigned short crc = 0x1D0F;   // CRC_16_AUG_CCITT Init Value

  if( buffer!=0) {
    for (i=0; i<bufferlen; i++) {
      crc = (unsigned short) ((crc << 8) ^ ccitt_16Table[(crc >> 8) ^ buffer[i]]);
    }
  }
  return crc;
}

int64_t GetTickCount() {
  unsigned long long   l_i64RetValue = 0;
  struct    timespec now;
  unsigned long long timestamp_secs_to_ms = 0;
  unsigned long long  timestamp_nsecs_to_ms = 0;
  unsigned long long   timestamp_ms = 0;

  //Clock Get Time ////////////////////
  clock_gettime(CLOCK_MONOTONIC, &now);
  timestamp_secs_to_ms = ( unsigned long long)now.tv_sec;
  timestamp_secs_to_ms = timestamp_secs_to_ms * 1000;
  timestamp_nsecs_to_ms = ( unsigned long long)now.tv_nsec;
  timestamp_nsecs_to_ms = timestamp_nsecs_to_ms / 1000000;
  timestamp_ms = timestamp_secs_to_ms + timestamp_nsecs_to_ms;
  l_i64RetValue = timestamp_ms;
  return l_i64RetValue;
}

// Low level function that sends a package through serial
// Returns  0 (NoError) if OK
// Returns -1 if ERROR
int Send_Serial_package(HANDLE serial_fd, const unsigned char *package, size_t package_len) {
  // Clear TX buffer
  if (flush_buffer_tx(serial_fd) != 0) {
    return -4; // Serial flush error
  }
  return send_buffer(serial_fd, package, package_len);
}

// Low level function that waits to receive a package from serial
// It is prepared for Boson, so will start receiving bytes when 0x8E is
// received, and will finish when 0xAE is received... or after a timeout
// Returns  0 if OK
// Returns -1 if TIMEOUT (no start byte, or no data)
// Returns -2 if MAX BUFFER REACHED
// Returns -3 if TIMEOUT ( without getting End_of_frame)
int Receive_Serial_package(HANDLE serial_fd, BosonContext *ctx) {
  size_t i = 0;
  int car; // byte read from serial
  int64_t endwait;

  // Clear the reception buffer
  if (flush_buffer_rx(serial_fd) != 0) {
    return -4; // Serial flush error
  }

  // Repeat until START BYTE is received or timeout happens
  int64_t now;
  now = GetTickCount();
  endwait = now + SERIAL_TIMEOUT;
  while ( now < endwait )  {
    int waiting = rxbyte_waiting(serial_fd);
    if (waiting < 0) {
      return -4; // Serial read error
    }
    if (waiting > 0) {
      car = read_byte(serial_fd);
      if (car < 0) {
        return -4; // Serial read error
      }
      if ( (unsigned char)car == 0x8E ) {
        ctx->boson_stuffed_package[i++] = 0x8E;
        break;
      }
    }
    now = GetTickCount();
  }

  if ( i == 0 ) {
    ctx->boson_stuffed_package_len = 0;
    if (now >= endwait) {
      return -3; //  // ERROR TIMEOUT
    } else { 
      return -1; // ERROR START FRAME not received
    }
  }

  // Receive data until END of FRAME is received.
  endwait = GetTickCount() + SERIAL_TIMEOUT;
  while (GetTickCount() < endwait) {
    int waiting = rxbyte_waiting(serial_fd);
    if (waiting < 0) {
      return -4; // Serial read error
    }
    if (waiting > 0) {
      car = read_byte(serial_fd);
      if (car < 0) {
        return -4; // Serial read error
      }
      if (i >= sizeof(ctx->boson_stuffed_package)) { // Check MAX BUFFER REACHED before storing
        return -2;  // ERROR : MAX BUFFER REACHED -> End of package not received
      }
      ctx->boson_stuffed_package[i++] = (unsigned char)car;
      ctx->boson_stuffed_package_len = i;
      if ((unsigned char)car == 0xAE) {  // Check end of frame
        return NoError;  // Package received correctly  
      }
    }
  }
  return -3; // ERROR TIMEOUT -> End of package not received
}






// Print Buffer in HEX
void print_buffer(const unsigned char *buffer, size_t bufferlen) {
  for (size_t i = 0; i < bufferlen; i++) {
      printf("%02X ", buffer[i]);
  }
  printf("\n");

}

// Print HELP MENU
void print_help() {
  printf(YEL ">>> \n" WHT);
  printf(YEL ">>> " WHT "rawBoson -p<serial port> -b<baudrate> c_command [x_data_0 x_data_1 .... x_data_n] [v][a]" WHT "\n");
  printf(YEL ">>> " WHT "  ( to read serial number : rawBoson -p/dev/ttyAMC0 -b921600 c00050002 v a )" WHT "\n");
  printf(YEL ">>> " WHT "  ( to change to black-hot : rawBoson -p/dev/ttyAMC0 -b921600 c000B003 x0 x0 x0 x1 v a )" WHT "\n");
  printf(YEL ">>> " WHT "  ( to change to white-hot : rawBoson -p/dev/ttyAMC0 -b921600 c000B0003 x0 x0 x0 x0 v a )" WHT "\n");
  printf(YEL ">>> \n" WHT);
  printf(YEL ">>> " WHT "  -v or v -> verbose, print advanced output on screen" WHT "\n");
  printf(YEL ">>> " WHT "  -a or a -> ascii: print answers in ASCII " WHT "\n");
  printf(YEL ">>> " WHT "  -b or b -> ascii: print answers in ASCII and HEX" WHT "\n");
  printf(YEL ">>> " WHT "  -h or --help -> print this help message" WHT "\n");
}

/*
----------------------------------------------------------------
---                    Boson FUNCTIONS                       ---
----------------------------------------------------------------
*/

// This is the function that builds the package for Boson, adding STATUS, SEQUENCE, CRC , etc
// Output of this function gets storage in these two global variables
//   aux_boson_package        // Max package without bit stuffing 768
//   aux_boson_package_len    // To storage real size of package
//   sequence                 // This number increases in every SENT command.
void Boson_Build_FSLP(BosonContext *ctx,
  uint32_t function,
  const unsigned char *Data,
  size_t DataCount )
{
  unsigned short aux_crc;

  //////////////////
  // Create Header
  ctx->aux_boson_package_len = 0;

  // ---- Boson Header
  // Start FLAG
  ctx->aux_boson_package[0] = 0x8E;  // Start byte. For Boson is always this. Need bit stuffing later
  // Channel Number
  ctx->aux_boson_package[1] = 0x0;   // This Software always uses Channel 0 (Reserved for FLIR Binary Protocol)

  // ---- Boson Payload ( 0 <= N <= 768)

  // Sequence Number: Check that the answer has same number
  // ( 4 - bytes)
  ctx->sequence = (ctx->sequence + 1) % 0x0A;  // Number from 0 to 9
  ctx->aux_boson_package[2] = 0;  // Fix to cero
  ctx->aux_boson_package[3] = 0;  // Fix to cero
  ctx->aux_boson_package[4] = 0;  // Fix to cero
  ctx->aux_boson_package[5] = ctx->sequence & 0xFF;

  // Command ID (4 bytes)
  ctx->aux_boson_package[6] =  (unsigned char)(( function >> 24 ) & 0xFF);
  ctx->aux_boson_package[7] =  (unsigned char)(( function >> 16 ) & 0xFF);
  ctx->aux_boson_package[8] =  (unsigned char)(( function >> 8  ) & 0xFF);
  ctx->aux_boson_package[9] =  (unsigned char)( function & 0xFF) ;

  // Command status
  // When sending the command is set to 0xFF
  // If receiver is OK should be all 0's
  ctx->aux_boson_package[10] = 0xFF;
  ctx->aux_boson_package[11] = 0xFF;
  ctx->aux_boson_package[12] = 0xFF;
  ctx->aux_boson_package[13] = 0xFF;

  // Move pointer in the TX buffer to start adding DATA
  ctx->aux_boson_package_len += 14;

  // Data (0 <= N <= MAX_DATA_BYTES)
  if (DataCount > MAX_DATA_BYTES) {
    DataCount = MAX_DATA_BYTES;  // Cap to maximum payload size to prevent overflow.
  }
  if (DataCount > 0) {
    memcpy(ctx->aux_boson_package + ctx->aux_boson_package_len, Data, DataCount);
    ctx->aux_boson_package_len += DataCount;
  }

  // ---- Boson Trailer
  // CRC16 (from Channel number to Last byte of data payload )
  // CRC should be done before BIT STUFFING
  aux_crc = CalcBlockCRC16(ctx->aux_boson_package_len-1, ctx->aux_boson_package+1);
  ctx->aux_boson_package[ctx->aux_boson_package_len] = (unsigned char)((aux_crc >> 8) & 0xFF);
  ctx->aux_boson_package[ctx->aux_boson_package_len + 1] = (unsigned char)(aux_crc & 0xFF);
  // End FLAG (0x0A)
  ctx->aux_boson_package[ctx->aux_boson_package_len + 2] = 0xAE;

  // Return number of bytes to be sent
  ctx->aux_boson_package_len += 3;
}

// Prints Boson Payload before or after bit stuffing has been done
// It prints the Buffer but with Colorization
void Boson_Print_FSLP(const unsigned char *data, size_t datalen ) {
  int pos = 0;

  printf(YEL ">>> " WHT);

  for (size_t i = 0; i < datalen; i++) {
    // Sequence number
    if ( ((2<=pos) && (pos<=5)) ) {
      printf(MAG "%02X " WHT, data[i]);
      pos++;
    }
    // Command ID number
    else if ( (6 <= pos) && (pos<=9)) {
      printf(BLU "%02X " WHT, data[i]);
      pos++;
    }
    // Command status
    else if ( (10 <= pos) && (pos<=13)) {
      printf(RED "%02X " WHT, data[i]);
      pos++;
    }
    // Data
    else if ((14 <= pos) && (pos < (int)datalen - 3)) {
      printf(CYN "%02X " WHT, data[i]);
      pos++;
    } else {
      printf(WHT "%02X " WHT, data[i]);
      pos++;
    }
  }
  printf("\n");
}

// Once FRAME and BIT_UN_STUFFING has been done, we will
// perform basic checks in the FRAME. Also grab the DATA.
int Boson_Check_Received_Frame(BosonContext *ctx, const unsigned char *pkg, size_t pkg_len) {
  uint16_t aux_crc;
  uint32_t aux_var = 0;

  if (pkg_len < 17) {
    return -5; // Frame too short to contain valid header, CRC and end flag
  }

  if (pkg_len > sizeof(ctx->aux_boson_package)) {
    return -5; // Frame too large for unstuffed Boson packet
  }

  if (pkg[0] != 0x8E || pkg[pkg_len-1] != 0xAE) {
    return -5; // Invalid frame delimiters
  }

  // Check Channel is ZERO
  if ( pkg[1] != 0 ) {
    return -1;  // Answer not received in channel 0
  }

  // Check STATUS is all ZERO
  aux_var = ((uint32_t)pkg[10] << 24) | ((uint32_t)pkg[11] << 16) | ((uint32_t)pkg[12] << 8) | (uint32_t)pkg[13];
  if (aux_var != 0) {
    return (int)aux_var; // STATUS has errors. Is not ZERO
  }

  // Check Sequence number is what we sent.
  aux_var = ((uint32_t)pkg[2] << 24) | ((uint32_t)pkg[3] << 16) | ((uint32_t)pkg[4] << 8) | (uint32_t)pkg[5];
  if (aux_var != (uint32_t)ctx->sequence ) {
    return -3;  // Sequence mismatch
  }

  // Check CRC
  aux_crc = CalcBlockCRC16(pkg_len-4, pkg+1);
  if (aux_crc != ((pkg[pkg_len-3] << 8) | pkg[pkg_len-2])) {
    return -4;  // Bad CRC
  }

  ctx->BosonData_count = 0;
  if (pkg_len > 17) { // We got extra DATA
    ctx->BosonData_count = pkg_len - 17;
    memcpy(ctx->BosonData, pkg+14, ctx->BosonData_count);
  }
  return NoError;
}


// Boson Bit Stuffing
// Input : Boson Package (Before Bit Stuffing) ................
// unsigned char   aux_boson_package[768] = "";   // Max package without bit stuffing 768
// unsigned short  aux_boson_package_len  = 0 ;   // To storage real size of package
//
// Output: Boson Package (After Bit Stuffing) .............--
// unsigned char   boson_stuffed_package[1544] = "";   // Max package with bit stuffing 1544
// unsigned short  boson_stuffed_package_len  = 0 ;            // To storage real size of package
void Boson_BitStuffing(BosonContext *ctx) {
  int i;
  int j=0;

  // Don't include START and END of frame in bit stuffing
  ctx->boson_stuffed_package[0] = ctx->aux_boson_package[j++];
  // Search for bytes to be stuffed
  for (i = 1; i < ctx->aux_boson_package_len - 1; i++) {
    switch (ctx->aux_boson_package[i]) {
      case 0x8E:
        ctx->boson_stuffed_package[j++] = 0x9E;
        ctx->boson_stuffed_package[j++] = 0x81;
        break;
      case 0x9E:
        ctx->boson_stuffed_package[j++] = 0x9E;
        ctx->boson_stuffed_package[j++] = 0x91;
        break;
      case 0xAE:
        ctx->boson_stuffed_package[j++] = 0x9E;
        ctx->boson_stuffed_package[j++] = 0xA1;
        break;
      default:
        ctx->boson_stuffed_package[j++] = ctx->aux_boson_package[i];
    }
  }
  // Add END of FRAME to stuffed package
  ctx->boson_stuffed_package[j++] = ctx->aux_boson_package[i];
  // Update new length of package to be sent
  ctx->boson_stuffed_package_len = j;
}

// Boson Bit_Un_stuffing
// Input: Boson Package (With  Bit Stuffing) .............--
// unsigned char   boson_stuffed_package[1544] = "";   // Max package with bit stuffing 1544
// unsigned short  boson_stuffed_package_len  = 0 ;            // To storage real size of package
//
// Output : Boson Package (No Bit Stuffing) ................
// unsigned char   aux_boson_package[768] = "";   // Max package without bit stuffing 768
// unsigned short  aux_boson_package_len  = 0 ;   // To storage real size of package
void Boson_BitUnstuffing(BosonContext *ctx) {
  int i;
  int j=0;

  if (ctx->boson_stuffed_package_len < 2) {
    ctx->aux_boson_package_len = 0;
    return;
  }

  // Don't include START and END of frame in bit stuffing
  ctx->aux_boson_package[j++] = ctx->boson_stuffed_package[0];
  // Search for bytes to be stuffed
  for (i = 1; i < ctx->boson_stuffed_package_len - 1; i++) {
    if (ctx->boson_stuffed_package[i] == 0x9E) {
      if (i + 1 >= ctx->boson_stuffed_package_len - 1) {
        // Malformed frame: stuffed marker without replacement value
        ctx->aux_boson_package_len = 0;
        return;
      }
      ctx->aux_boson_package[j++] = ctx->boson_stuffed_package[i + 1] + 0xD;
      i++;
    } else {
      ctx->aux_boson_package[j++] = ctx->boson_stuffed_package[i];
    }
  }
  // Add END of FRAME to stuffed package
  ctx->aux_boson_package[j++] = ctx->boson_stuffed_package[i];
  // Update new length of package to be sent
  ctx->aux_boson_package_len = j;
}

// Boson Status message returns codes
// Boson can send an error in the status field of the
// received message. These are most common ones.
void Boson_Status_Error_Codes(int code) {
  switch (code) {
    case R_CAM_DSPCH_BAD_CMD_ID:  // 0x0161
      printf(RED ">>> Error : Boson Status (" MAG "Bad CMD ID" RED ")" RESET "\n");
      break;
    case R_CAM_DSPCH_BAD_PAYLOAD_STATUS:  // 0x0162
      printf(RED ">>> Error : Boson Status (" MAG "Bad Payload" RED ")" RESET "\n");
      break;
    case R_CAM_PKG_UNSPECIFIED_FAILURE:  // 0x0170
      printf(RED ">>> Error : Boson Status (" MAG "Unspecified" RED ")" RESET "\n");
      break;
    case R_CAM_PKG_INSUFFICIENT_BYTES:  // 0x017D
      printf(RED ">>> Error : Boson Status (" MAG "Insufficient bytes" RED ")" RESET "\n");
      break;
    case R_CAM_PKG_EXCESS_BYTES:  // 0x017E
      printf(RED ">>> Error : Boson Status (" MAG "Excess bytes" RED ")" RESET "\n");
      break;
    case R_CAM_PKG_BUFFER_OVERFLOW:  // 0x017F
      printf(RED ">>> Error : Boson Status (" MAG "Buffer Overflow" RED ")" RESET "\n");
      break;
    case FLIR_RANGE_ERROR :  // 0x0203
      printf(RED ">>> Error : Boson Status (" MAG "Range Error" RED ")" RESET "\n");
      break;
    default:
      printf(RED ">>> Error : Boson Status (" MAG "Unknown" RED ")" RESET "\n");
      break;
  }
}

//------------------------------------------------------
// Entry point to the program. Read arguments and
// run the Function to make the Boson Package.
int main(int argc, char **argv) {
  // Serial Port variables
  char puerto_str[30];            // We give up to 30 chars to put the path
  char baudrate_str[10];           // Max size is 921600 ... so we give margin
  int  baudrate;

  // Boson Function variables
  uint32_t commandID = 0;          // Mandatory to receive as input
  int commandID_valid = 0;
  char commandID_str[10];          // CommandID is 4 bytes. We give margin as well
  size_t num_bytes = 0;            // Number of DATA to send
  unsigned char  tx_buffer[768];   // Max Boson Data before bit stuffing

  // Auxiliary variables
  int  i,ret;
  int  debug_on = 0;
  int  ascii_on = 0;
  char aux_data_str[3];
  char aux_cad[20];
  int  aux_data;
  BosonContext ctx = {0};
  PortSettingsType serial_port_config;
  HANDLE serial;

  // Default BOSON values
  snprintf(puerto_str, sizeof puerto_str, "/dev/ttyACM0");
  baudrate = 921600;
  snprintf(baudrate_str, sizeof baudrate_str, "%d", baudrate);

  // Check that are line arguments
  if (argc<=1) {
    print_help();
    printf(YEL ">>>" WHT "\n");
    return -1;
  }

  // Read command line arguments
  for (i = 1; i < argc; i++) {
    char *arg = argv[i];

    if (strcmp(arg, "v") == 0 || strcmp(arg, "-v") == 0) {
      debug_on = 1;
    } else if (strcmp(arg, "a") == 0 || strcmp(arg, "-a") == 0) {
      ascii_on = 1;
    } else if (strcmp(arg, "b") == 0 || strcmp(arg, "-b") == 0) {
      ascii_on = 2;
    } else if (strcmp(arg, "-p") == 0) {
      if (i + 1 >= argc) {
        print_help();
        printf(YEL ">>> " RED "Error: missing serial port after -p" WHT "\n");
        return -1;
      }
      i++;
      snprintf(puerto_str, sizeof puerto_str, "%s", argv[i]);
    } else if (strncmp(arg, "-p", 2) == 0) {
      if (arg[2] == '\0') {
        print_help();
        printf(YEL ">>> " RED "Error: missing serial port after -p" WHT "\n");
        return -1;
      }
      snprintf(puerto_str, sizeof puerto_str, "%s", arg + 2);
    } else if (strcmp(arg, "-b") == 0) {
      if (i + 1 >= argc) {
        print_help();
        printf(YEL ">>> " RED "Error: missing baudrate after -b" WHT "\n");
        return -1;
      }
      i++;
      snprintf(baudrate_str, sizeof baudrate_str, "%s", argv[i]);
      baudrate = atoi(baudrate_str);
    } else if (strncmp(arg, "-b", 2) == 0) {
      if (arg[2] == '\0') {
        print_help();
        printf(YEL ">>> " RED "Error: missing baudrate after -b" WHT "\n");
        return -1;
      }
      snprintf(baudrate_str, sizeof baudrate_str, "%s", arg + 2);
      baudrate = atoi(baudrate_str);
    } else if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0) {
      print_help();
      return 0;
    }
    // Check for the commandID
    else if ( arg[0] == 'c' && arg[1] != '\0') {
      if (strlen(arg + 1) > MAX_COMMAND_HEX) {
        commandID_valid = 0;
      } else {
        snprintf(commandID_str, sizeof commandID_str, "%s", arg + 1);
        if (hex_to_uint32(commandID_str, &commandID) == 0) {
          commandID_valid = 1;
        } else {
          commandID_valid = 0;
        }
      }
    }
    // Check for FUNCTION DATA
    else if ( arg[0] == 'x' && arg[1] != '\0') {
      if (num_bytes >= MAX_DATA_BYTES) {
        print_help();
        printf(YEL "\n>>> " RED "Error : data num_bytes out of range. Too many DATA IN" WHT "\n");
        return -1;
      }
      snprintf(aux_data_str, sizeof aux_data_str, "%s", arg + 1);
      aux_data = hex_to_int(aux_data_str);
      if ( aux_data < 0 ) {  // ERROR parsing data. Bad entry
        print_help();
        if ( aux_data == -1) {
          printf(YEL ">>> " RED "Error : HexData too LONG (max 1 byte in HEX)" WHT "\n");
        } else if ( aux_data == -2) {
          printf(YEL ">>> " RED "Error : Some CHARs are not HEX range " WHT "\n");
        }
        printf(YEL ">>> " RED "Error : data[%zu]=0x%s not valid " WHT "\n", num_bytes, aux_data_str);
        return -1;   // Exit program
      }
      tx_buffer[num_bytes++] = aux_data & 0xFF;
    } else {
      print_help();
      printf(YEL ">>> " RED "Error: Invalid argument '%s'" WHT "\n", arg);
      return -1;
    }
  }  // End of parsing arguments.

  // If commandID < 0 then EXIT with error.
  // Don't even try to Open Serial Port
  if (!commandID_valid) {  // ERROR getting the CommandID
    print_help();
    printf(YEL ">>> " RED "Error : Invalid command ID" WHT "\n");
    return -1;  // Exit main program
  }

  sprintf(aux_cad,"%i,8,n,1",baudrate);
  // open serial port
  serial_port_config = str2ps(puerto_str, aux_cad);
  ret = open_port(serial_port_config, &serial);
  if ( ret != 0) {
    print_help();
    printf(RED ">>> Error while opening serial port " CYN "%s" RESET "\n", puerto_str);
    printf(RED ">>> Use rawBoson -p/dev/ttyXXX to select yours." RESET "\n");
    return -1;   // Exit program
  }

  if ( debug_on == 1) {
    print_help();
    printf(YEL ">>> " CYN "%s (%s)" WHT "\n", puerto_str, aux_cad);
    printf(YEL ">>> " CYN "DEBUG (v) mode ON" WHT "\n");
    if ( ascii_on == 1 ) {
      printf(YEL ">>> " CYN "Ascii (a) mode ON" WHT "\n");
    } else {
      printf(YEL ">>> " CYN "Ascii (a) mode OFF" WHT "\n");
    }
    printf(YEL ">>>  \n\n" WHT);
  }

  /* STEP 1 */
  // Build package to SEND using SERIAL
  Boson_Build_FSLP(&ctx, commandID, tx_buffer, num_bytes);
  // Print with Colors
  if (debug_on == 1) {
    printf(YEL ">>>" WHT " Frame to send (%i bytes)\n", ctx.aux_boson_package_len);
    Boson_Print_FSLP(ctx.aux_boson_package, ctx.aux_boson_package_len);
  }

  /* STEP 2 */
  // Do Boson_Bit stuffing
  // It uses as input and provides a new ARRAY and LENGTH to be sent using serial
  Boson_BitStuffing(&ctx);
  // DEBUG  print_package(send array and length);
  //printf(YEL ">>>" WHT " Raw data SENT to serial (%i bytes)\n", boson_stuffed_package_len);
  //print_buffer(boson_stuffed_package, boson_stuffed_package_len);

  /* STEP 3 */
  // Send over serial
  ret = Send_Serial_package(serial, ctx.boson_stuffed_package, ctx.boson_stuffed_package_len);
  if ( ret < 0 ) {  // Function returns -1 if error
    printf(RED ">>> Error while sending bytes " RESET "\n");
    close_port(serial);
    return -1;  // Exit program
  }

  /* STEP 4 */
  // Empty Boson buffers before receiving
  memset(ctx.boson_stuffed_package, 0, ctx.boson_stuffed_package_len);  // Receive from serial
  memset(ctx.aux_boson_package, 0, ctx.aux_boson_package_len);          // After un_stuffing
  // Put a CERO the count of received bytes
  ctx.boson_stuffed_package_len = 0;
  ctx.aux_boson_package_len = 0;

  /* STEP 5 */
  //  Receive over serial
  //  We will use the same buffers reserved for the Stuffed since they are available now
  ret = Receive_Serial_package(serial, &ctx);
  // DEBUG print_package(send array and length);
  //printf(YEL ">>>" WHT " Raw data RECEIVED from serial (%i bytes)\n", boson_stuffed_package_len);
  //print_buffer(boson_stuffed_package, boson_stuffed_package_len);
  if ( ret < 0 ) {  // Function returns <0 if error
    if ( ret==-1) {
      printf(RED ">>> Error while receiving bytes (No START Detected) " RESET "\n");
    } else if ( ret==-2) {
      printf(RED ">>> Error while receiving bytes (Buffer Overflow > 1544)" RESET "\n");
    } else if (ret==-3) {
      printf(RED ">>> Error while receiving bytes (Timeout)" RESET "\n");
    } else {
      printf(RED ">>> Error while receiving bytes " RESET "\n");
    }
    close_port(serial);
    return -1;
  }

  /* STEP 6 */
  //  Boson_undo_bit_stuffing
  //  We will use the same buffers reserved (aux_boson_package) since they are available now
  Boson_BitUnstuffing(&ctx);
  //printf(YEL ">>>" WHT " Raw data after un_stuffing (%i bytes)\n", ctx.aux_boson_package_len);
  //print_buffer(ctx.aux_boson_package, ctx.aux_boson_package_len);

  // Print with Colors
  if (debug_on == 1) {
    printf(YEL ">>>" WHT " Frame received (%i bytes)\n", ctx.aux_boson_package_len);
    Boson_Print_FSLP(ctx.aux_boson_package, ctx.aux_boson_package_len);
    printf(YEL ">>>" WHT "\n");
  }

  /* STEP 7 */
  // Boson check answer and grab DATA if received
  ret = Boson_Check_Received_Frame(&ctx, ctx.aux_boson_package, ctx.aux_boson_package_len);
  if (ret!=0) { // Check FAILS
    if ( ret==-1) {
      printf(RED ">>> Error : Boson Answer didn't come through CHANNEL 0" RESET "\n");
    } else if (ret==-3) {
      printf(RED ">>> Error : Boson Sequence mismatch" RESET "\n");
    } else if (ret==-4) {
      printf(RED ">>> Error : Boson Bad CRC Received " RESET "\n");
    } else if ( ret>0 ) {
      Boson_Status_Error_Codes(ret);
    } else {
      printf(RED ">>> Error : Boson Undefined Error " RESET "\n");
    }
  }

  /* STEP 8 */
  // Print DATA received
  if ( ctx.BosonData_count > 0) {
    if (debug_on == 1) {
      printf(YEL ">>>" WHT " Data received (%i bytes) : ", ctx.BosonData_count);
    }
    printf(CYN);
    for (i=0; i<ctx.BosonData_count; i++) {
      if (ascii_on ==1 ) {  // Just ASCII
        printf("%c", ctx.BosonData[i]);
      } else if (ascii_on==2) {  // ASCII and HEX
        printf("%02X('%c') ", ctx.BosonData[i], ctx.BosonData[i]);
      } else {
        printf("%02X ", ctx.BosonData[i]);
      }
    }
    if (debug_on == 1) {
      printf("\n" YEL ">>>" );
    }
    printf(WHT "\n");
  }

  // close serial port and exit application
  close_port(serial);
  return NoError;
}
/* END OF FILE */
