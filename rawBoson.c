// --------------------------------------
// FLIR Systems 2017 (c)
// --------------------------------------

#include "rawBoson.h"

#define SERIAL_TIMEOUT 500
int debug_on=0;
int ascii_on=0;

// GLOBAL declaration to have access to them
// from the other functions. Not the best practise, but simple and fast.

// ........ Boson Package (Before Bit Stuffing) ................
unsigned char   aux_boson_package[768] = "";   // Max package without bit stuffing 768
unsigned short  aux_boson_package_len  = 0 ;   // To storage real size of package
static unsigned char sequence=0;               // This number increases in every SENT commnand.

// ........ Boson Package (After Bit Stuffing) .............
unsigned char   boson_stuffed_package[1544] = "";   // Max package with bit stuffing 1544
unsigned short  boson_stuffed_package_len   = 0 ;    // To storage real size of package

// ........ Received BOSON requested DATA ..................................
unsigned char  BosonData[768] = "";    // Buffer with DATA from Boson
unsigned short BosonData_count = 0;    // Number of data received

/*
------------------------------------------------------------------------------
--- Aux functions: CRC, Serial port TX/RX , Gettickcount, Hex Operations   ---
------------------------------------------------------------------------------
*/


// Boson is using this method: CRC-16/AUG-CCITT
// https://github.com/meetanthony/crcphp/blob/master/crc16/crc_16_aug_ccitt.php
unsigned short CalcBlockCRC16(unsigned int bufferlen, unsigned char *buffer)
{

  unsigned short ccitt_16Table[] =
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
// If Error returns -1
// If OK returns 0
int Send_Serial_package( unsigned char *package, short package_len) {
    // Clear TX buffer
    flush_buffer_tx(serial);   // Serial GLobal Var
    // send_to_serial returns 0 if Error. 1 if OK
    return send_buffer(serial, package, package_len) -1 ; // to convert to our Error convention
}

// Low level function that waits to receive a package from serial
// It is prepare for Boson, so will start receiving bytes when 0x8E is received, and will finish when 0xAE
// is received... or after a timeout
int Receive_Serial_package() {
  int i=0;
  unsigned char car; // byte read from serial
  int64_t endwait;

	// Clear the reception buffer
  flush_buffer_rx(serial);   // Serial GLobal Var

	// Repeat until START BYTE is received or timeout (10sec) happens!!
	endwait = GetTickCount () + SERIAL_TIMEOUT;
	while ( (GetTickCount() < endwait) )  {
		  if ( rxbyte_waiting(serial) ) {
            car = read_byte(serial)&0xFF;
			      if ( car == 0x8E ) {
                boson_stuffed_package[i++]=0x8E;
                break;
            }
      }
  }
  if ( i==0 ) {
      boson_stuffed_package_len=i;
      return -1; // ERROR NO DATA or START FRAME received
  }

  // Receive data until END of FRAME is received.
  endwait = GetTickCount () + SERIAL_TIMEOUT;
	while ( (GetTickCount() < endwait) )  {
      if ( rxbyte_waiting(serial) ) {
          car = read_byte(serial);
          boson_stuffed_package[i++]=car;
          boson_stuffed_package_len=i;
          if (car == 0xAE) {
              return NoError;   // Package received
          }
          if (i>=1544) {  // MAX Buffer size without receiving END OF FRAME
               return -2;
          }
      }
  }
  return -2; // ERROR TIMEOUT !!
}

// Print Buffer in HEX
void print_buffer(unsigned char *buffer, int bufferlen) {
  for (int i=0; i<bufferlen; i++) {
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
  printf(YEL ">>> " WHT "  v -> verbose, print advanced output on screen" WHT "\n");
  printf(YEL ">>> " WHT "  a -> ascii: print answers in ASCII " WHT "\n");
  printf(YEL ">>> " WHT "  b -> ascii: print answers in ASCII and HEX" WHT "\n");
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
//   sequence                 // This number increases in every SENT commnand.
void Boson_Build_FSLP(
                unsigned long  function,
                unsigned char* Data,
                unsigned short DataCount )
{
        unsigned short aux_crc;
        int i;

        //////////////////
        // Create Header
        aux_boson_package_len = 0;

        // ---- Boson Header
        // Start FLAG
        aux_boson_package[0] = 0x8E;  // Start byte. For Boson is always this. Need bitstuffing later
        // Channel Number
        aux_boson_package[1] = 0x0;   // This Software always uses Channel 0 (Reserved for FLIR Binay Protocol)

        // ---- Boson Payload ( 0 <= N <= 768)

        // Sequence Number: Check that the answer has same number
        // ( 4 - bytes)
        sequence = ( sequence+1 )%0x0A;  // Number from 0 to 9
        aux_boson_package[2] = 0;  // Fix to cero
        aux_boson_package[3] = 0;  // Fix to cero
        aux_boson_package[4] = 0;  // Fix to cero
        aux_boson_package[5] = sequence & 0xFF;

        // Command ID (4 bytes)
        aux_boson_package[6] =  (unsigned char)(( function >> 24 ) & 0xFF);
        aux_boson_package[7] =  (unsigned char)(( function >> 16 ) & 0xFF);
        aux_boson_package[8] =  (unsigned char)(( function >> 8  ) & 0xFF);
        aux_boson_package[9] =  (unsigned char)( function & 0xFF) ;

        // Command status
        // When sending the command is set to 0xFF
        // If receiver is OK should be all 0's
        aux_boson_package[10] = 0xFF;
        aux_boson_package[11] = 0xFF;
        aux_boson_package[12] = 0xFF;
        aux_boson_package[13] = 0xFF;

        // Move pointer in the TX buffer to start adding DATA
        aux_boson_package_len += 14;

        // Data (0 <= N <= 756)
        // TO DO: Convert to memcpy
        for (i=0; i<DataCount; i++)
        {
                aux_boson_package[aux_boson_package_len + i] = Data[i];
        }
        aux_boson_package_len += DataCount;

        // ---- Boson Trailer
        // CRC16 (from Channel number to Last byte of data payload )
        // CRC should be done before BIT STUFFING
        aux_crc = CalcBlockCRC16(aux_boson_package_len-1, aux_boson_package+1);
        aux_boson_package[aux_boson_package_len] = (unsigned char)((aux_crc >> 8) & 0xFF);
        aux_boson_package[aux_boson_package_len + 1] = (unsigned char)(aux_crc & 0xFF);
        // End FLAG (0x0A)
        aux_boson_package[aux_boson_package_len + 2] = 0xAE;

        // Return number of bytes to be sent
        aux_boson_package_len += 3;   // Global variable
}

// Prints Boson Payload before or after bitstuffing has been done
// It prints the Buffer but with Colorization
void Boson_Print_FSLP(unsigned char *data, unsigned short datalen ) {
  short pos=0;
  int   i=0;

  printf(YEL ">>> " WHT);

  for (i=0 ; i<datalen; i++) {
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
      else if ( (14<=pos) && (pos<datalen-3)) {
          printf(CYN "%02X " WHT, data[i]);
          pos++;
      } else {
          printf(WHT "%02X " WHT, data[i]);
          pos++;
      }
  }
  printf("\n");
}

// Once FRAME and BIT_UNSTUFFING has been done, we will
// perform basic checks in the FRAME. Also grab the DATA.
int Boson_Check_Received_Frame(unsigned char *pkg) {
    unsigned short aux_crc;
    unsigned long aux_var=0;

    // Check Channel is ZERO
    if ( pkg[1] != 0 ) {
        return -1;  // Answer not received in channel 0
    }

    // Check STATUS is all ZERO
    aux_var = (long)(pkg[10]<<24) + (long)(pkg[11]<<16) + (long)(pkg[12]<<8) + (long)(pkg[13]);
    if (aux_var != 0) {
        return  (int)aux_var ; // STATUS has errors. Is not ZERO
    }

    // Check Sequence number is what we sent.
    aux_var = (long)(pkg[2]<<24) + (long)(pkg[3]<<16) + (long)(pkg[4]<<8) + (long)(pkg[5]);
    if (aux_var != (long)sequence ) {
        return -3;  // Sequence mistmatch
    }

    // Check CRC
    aux_crc = CalcBlockCRC16(aux_boson_package_len-4, aux_boson_package+1);
    if (aux_crc != ( (pkg[aux_boson_package_len-3] * 256) + pkg[aux_boson_package_len-2]) ) {
        return -4;
    }

    // Grab DATA if any and copy it to BosonData array
    if ( aux_boson_package_len>17 ) { // We got extra DATA
        BosonData_count=aux_boson_package_len-17;
        memcpy(BosonData, aux_boson_package+14, BosonData_count);
    }
    return NoError;   // No error
}


// Boson Bitsuffing
// Input : Boson Package (Before Bit Stuffing) ................
// unsigned char   aux_boson_package[768] = "";   // Max package without bit stuffing 768
// unsigned short  aux_boson_package_len  = 0 ;   // To storage real size of package
//
// Output: Boson Package (After Bit Stuffing) .............--
// unsigned char   boson_stuffed_package[1544] = "";   // Max package with bit stuffing 1544
// unsigned short  boson_stuffed_package_len  = 0 ;            // To storage real size of package
void Boson_BitStuffing() {
  int i;   // aux to reover the frames
  int j=0;

  // Don't include START and END of frame in bitsuffing
  boson_stuffed_package[0]=aux_boson_package[j++];
  // Search for bytes to be sttuffed
  for(i=1; i<aux_boson_package_len-1 ; i++) {
      switch (aux_boson_package[i]) {
        case 0x8E:
            boson_stuffed_package[j++]=0x9E;
            boson_stuffed_package[j++]=0x81;
            break;
        case 0x9E:
            boson_stuffed_package[j++]=0x9E;
            boson_stuffed_package[j++]=0x91;
            break;
        case 0xAE:
            boson_stuffed_package[j++]=0x9E;
            boson_stuffed_package[j++]=0xA1;
            break;
        default:
            boson_stuffed_package[j++]=aux_boson_package[i];
      }
  }
  // Add END of FRAME to stuffed package
  boson_stuffed_package[j++]=aux_boson_package[i];
  // Update new lenght of package to be sent
  boson_stuffed_package_len=j;
}

// Boson BitUnsuffing
// Input: Boson Package (With  Bit Stuffing) .............--
// unsigned char   boson_stuffed_package[1544] = "";   // Max package with bit stuffing 1544
// unsigned short  boson_stuffed_package_len  = 0 ;            // To storage real size of package
//
// Output : Boson Package (No Bit Stuffing) ................
// unsigned char   aux_boson_package[768] = "";   // Max package without bit stuffing 768
// unsigned short  aux_boson_package_len  = 0 ;   // To storage real size of package
void Boson_BitUnstuffing() {
  int i;   // aux to reover the frames
  int j=0;

  // Don't include START and END of frame in bitsuffing
  aux_boson_package[j++]=boson_stuffed_package[0];
  // Search for bytes to be sttuffed
  for(i=1; i<boson_stuffed_package_len-1 ; i++) {
      if ( boson_stuffed_package[i]==0x9E) {
          aux_boson_package[j++]=boson_stuffed_package[i+1]+0xD;  // This is required by Boson
          i++;  // skip one (already used)
      } else {
          aux_boson_package[j++]=boson_stuffed_package[i];
      }
  }
  // Add END of FRAME to stuffed package
  aux_boson_package[j++]=boson_stuffed_package[i];
  // Update new lenght of package to be sent
  aux_boson_package_len=j;

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
  char baudios_str[10];           // Max size is 921600 ... so we give margin
  int  baudios;

  // Boson Function variables
  long commandID=-1;               // Mandatory to receive as input
  char commandID_str[10];          // CommandID is 4 bytes. We give margin as well
  unsigned short num_bytes=0;      // Number of DATA to send
  unsigned char  tx_buffer[768];   // Max Boson Data before bit stuffing

  // Auxiliary variables
  int  i,ret;
  char aux_data_str[2];
  char aux_cad[20];
  int  aux_data;

	// Default BOSON values
	strcpy(puerto_str,"/dev/ttyACM0");
	baudios=921600;

  // Check that are line arguments
  if (argc<=1) {
     print_help();
     printf(YEL ">>>" WHT "\n");
     return -1;
  }

  // Read command line arguments
  for (i=0; i<argc; i++) {
      // Look for verbose mode
      if ( argv[i][0]=='v') {
          debug_on=1;
      } // Look for feedback in ASCII
      else if ( argv[i][0]=='a') {
          ascii_on=1;
      } // Look for feedback in ASCII and HEX
      else if ( argv[i][0]=='b') {
          ascii_on=2;
      }
      // Check Serial Port or Baudrate
      else if ( argv[i][0]=='-' ) {
          if ( argv[i][1]=='p' ) {
              strcpy(puerto_str, argv[i]+2);
			    } else if ( argv[i][1]=='b' ) {
              strcpy(baudios_str, argv[i]+2);
              baudios=atoi(baudios_str);
          }
      }
      // Check for the commandID
      else if ( argv[i][0]=='c') {
          strcpy(commandID_str, argv[i]+1);
          commandID = hex_to_long(commandID_str);
      }
      // Check for FUNCTION DATA
      else if ( argv[i][0]=='x' ) {
          strcpy(aux_data_str, argv[i]+1);
          aux_data=hex_to_int(aux_data_str);
          if ( aux_data<0 ) {  // ERROR parsing data. Bad entry
              print_help();
              if ( aux_data==-1) {
                  printf(YEL ">>> " RED "Error : HexData too LONG (max 1 byte in HEX)" WHT "\n");
              }
              if ( aux_data==-2) {
                  printf(YEL ">>> " RED "Error : Some CHARs are not HEX range " WHT "\n");
              }
              printf(YEL ">>> " RED "Error : data[%i]=0x%s not valid " WHT "\n", num_bytes, aux_data_str);
              return -1;   // Exit program
          }
          if (num_bytes>768) {   // ERROR parsing data. Too many data inputs
            print_help();
            printf(YEL "\n>>> " RED "Error : data num_bytes out of range. Too many DATA IN" WHT "\n");
            return -1;   // Exit program
		      }
         tx_buffer[num_bytes]=aux_data&0xFF;
		     num_bytes++;    // Data to transmit Not includding commandID
	     }
  }  // End of parsing arguments.

  // If commandID < 0 then EXIT with error.
  // Don't even try to Open Serial Port
  if ( commandID < 0 ) {  // ERROR getting the CommandID
        print_help();
        if ( commandID == -1) {
            printf(YEL ">>> " RED "Error : command ID too LONG (max 4 bytes in HEX)" WHT "\n");
        }
        if ( commandID == -2) {
            printf(YEL ">>> " RED "Error : Some CHARs are not HEX range " WHT "\n");
        }
        printf(YEL ">>> " RED "Error : Invalid command ID" WHT "\n");
        return -1;  // Exit main program
  }

	sprintf(aux_cad,"%i,8,n,1",baudios);
	// open serial port
	puerto_serie_conf=str2ps(puerto_str,aux_cad);
	ret=open_port(puerto_serie_conf, &serial);
	if ( ret != 0) {
      print_help();
  		printf(RED ">>> Error while opening serial port " CYN "%s" RESET "\n", puerto_str);
  		printf(RED ">>> Use rawBoson -p/dev/ttyXXX to select yours." RESET "\n");
  		return -1;
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
  Boson_Build_FSLP(commandID, tx_buffer, num_bytes);
  // Print with Colors
  if (debug_on == 1) {
    printf(YEL ">>>" WHT " Frame to send (%i bytes)\n", aux_boson_package_len);
    Boson_Print_FSLP(aux_boson_package, aux_boson_package_len);
  }

  /* STEP 2 */
  // Do Boson_Bitstuffing
  // It uses as input and provides a new ARRAY and LENGHT to be sent using serial
  Boson_BitStuffing();
  // DEBUG  print_package(pasar array y longitud);
  //printf(YEL ">>>" WHT " Raw data SENT to serial (%i bytes)\n", boson_stuffed_package_len);
  //print_buffer(boson_stuffed_package, boson_stuffed_package_len);

  /* STEP 3 */
  // Send over serial
  ret=Send_Serial_package(boson_stuffed_package, boson_stuffed_package_len);
  if ( ret < 0 ) {  // Function returns -1 if error
    printf(RED ">>> Error while sending bytes " RESET "\n");
    close_port(serial);
    return -1;
  }

  /* STEP 4 */
  // Empty Boson buffers before receiving
  memset(boson_stuffed_package, 0, boson_stuffed_package_len);  // Receive from serial
  memset(aux_boson_package, 0, aux_boson_package_len);          // After unstuffing
  // Put a CERO the count of received bytes
	boson_stuffed_package_len=0;
  aux_boson_package_len=0;

  /* STEP 5 */
  //  Receive over serial
  //  We will use the same buffers reserved for the Stuffed since they are available now
  ret=Receive_Serial_package();
  // DEBUG print_package(pasar array y longitud);
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
  //  Boson_undo_bitsuffing
  //  We will use the same buffers reserved (aux_boson_package) since they are available now
  Boson_BitUnstuffing();
  //printf(YEL ">>>" WHT " Raw data after unstuffing (%i bytes)\n", aux_boson_package_len);
  //print_buffer(aux_boson_package, aux_boson_package_len);

  // Print with Colors
  if (debug_on == 1) {
    printf(YEL ">>>" WHT " Frame received (%i bytes)\n", aux_boson_package_len);
    Boson_Print_FSLP(aux_boson_package, aux_boson_package_len);
    printf(YEL ">>>" WHT "\n");
  }

  /* STEP 7 */
  // Boson check answer and grab DATA if received
  ret = Boson_Check_Received_Frame(aux_boson_package);
  if (ret!=0) { // Check FAILS
    if ( ret==-1) {
        printf(RED ">>> Error : Boson Answer didn't come through CHANNEL 0" RESET "\n");
    } else if (ret==-3) {
        printf(RED ">>> Error : Boson Sequence mistmatch" RESET "\n");
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
  if ( BosonData_count>0) {
      if (debug_on == 1) {
          printf(YEL ">>>" WHT " Data received (%i bytes) : ", BosonData_count);
      }
      printf(CYN);
      for (i=0; i<BosonData_count; i++) {
          if (ascii_on ==1 ) {  // Just ASCII
              printf("%c",BosonData[i]);
          } else if (ascii_on==2) {  // ASCII and HEX
              printf("%02X('%c') ",BosonData[i],BosonData[i]);
          } else {
              printf("%02X ",BosonData[i]);
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
