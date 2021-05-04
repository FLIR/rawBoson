#ifndef MAINAPPLICATION_HPP
#define MAINAPPLICATION_HPP
#define SERIAL_TIMEOUT 500
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
#define BYTES_READ	1
#define COLARX		100
#define COLATX		10


#define NOPARITY            0
#define ODDPARITY           1
#define EVENPARITY          2
#define MARKPARITY          3
#define SPACEPARITY         4

#define ONESTOPBIT          0
#define ONE5STOPBITS        1
#define TWOSTOPBITS         2
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <math.h>   // For POW function
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <QObject>
typedef struct PortSettings {
    char port[20];
    int baudrate;
    int databits;
    int parity;
    int stopbits;
} PortSettingsType;
class MainApplication : public QObject
{
    Q_OBJECT
public:
    explicit MainApplication(QObject *parent = nullptr);
    PortSettingsType puerto_serie_conf;

int serial;
int debug_on=0;
int ascii_on=0;

// GLOBAL declaration to have access to them
// from the other functions. Not the best practise, but simple and fast.

// ........ Boson Package (Before Bit Stuffing) ................
unsigned char   aux_boson_package[768];   // Max package without bit stuffing 768
unsigned short  aux_boson_package_len  = 0 ;   // To storage real size of package
unsigned char sequence=0;               // This number increases in every SENT commnand.

// ........ Boson Package (After Bit Stuffing) .............
unsigned char   boson_stuffed_package[1544];   // Max package with bit stuffing 1544
unsigned short  boson_stuffed_package_len   = 0 ;    // To storage real size of package

// ........ Received BOSON requested DATA ..................................
unsigned char  BosonData[768];    // Buffer with DATA from Boson
unsigned short BosonData_count = 0;    // Number of data received

/*
------------------------------------------------------------------------------
--- Aux functions: CRC, Serial port TX/RX , Gettickcount, Hex Operations   ---
------------------------------------------------------------------------------
*/
unsigned short CalcBlockCRC16(unsigned int bufferlen, unsigned char *buffer);
int64_t GetTickCount();
int Send_Serial_package(unsigned char *package, short package_len);
int Receive_Serial_package();
void print_buffer(unsigned char *buffer, int bufferlen);
void print_help();
void Boson_Build_FSLP(unsigned long function, unsigned char *Data, unsigned short DataCount);
void Boson_Print_FSLP(unsigned char *data, unsigned short datalen);
int Boson_Check_Received_Frame(unsigned char *pkg);
void Boson_BitStuffing();
void Boson_BitUnstuffing();
void Boson_Status_Error_Codes(int code);
int initilize(int argc, char *argv[]);
long hex_to_long(char *str);
int hex_to_int(char *str);
int toInt(unsigned char mybyte);
int open_port(PortSettingsType ps, int *handle);
int close_port(int handle);
int send_buffer(int fd, unsigned char *tx_array, short bytes_to_send);
int send_byte(int fd, unsigned char car);
int rxbyte_waiting(int fd);
int txbyte_waiting(int fd);
unsigned char read_byte_time(int fd, int plazo, int *timeout);
unsigned char read_byte(int fd);
int flush_buffer_tx(int fd);
int flush_buffer_rx(int fd);
int send_break(int fd);
PortSettingsType str2ps(char *str1, char *str2);
int setup_serial(int fdes, int baud, int databits, int stopbits, int parity);
signals:

};

#endif // MAINAPPLICATION_HPP
