/*
MIT License

Copyright 2002 Ifara Tecnologias S.L.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/



// Error handling convention
// Returns 0 when no error
// Returns <0 when error
// >0 is the byte read


#ifndef serial_h
#define serial_h

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>



#define BYTES_READ	1
#define COLARX		100
#define COLATX		10
#define HANDLE   	int

#define NOPARITY            0   
#define ODDPARITY           1   
#define EVENPARITY          2   
#define MARKPARITY          3   
#define SPACEPARITY         4   

#define ONESTOPBIT          0   
#define ONE5STOPBITS        1   
#define TWOSTOPBITS         2   


typedef struct PortSettings {
	char port[20];
	int baudrate;
	int databits;
	int parity;
	int stopbits;
} PortSettingsType;

int open_port(PortSettingsType ps,HANDLE *handle);
int setup_serial(int, int, int, int, int);
int close_port(HANDLE handle);
PortSettingsType str2ps (char *str1, char*str2);
int send_buffer(HANDLE fd, unsigned char* tx_array, short bytes_to_send);
int send_byte(HANDLE fd, char car);
int rxbyte_waiting(HANDLE fd);
int txbyte_waiting(HANDLE fd);
char read_byte_time(HANDLE fd);
char read_byte(HANDLE fd);
int flush_buffer_rx(HANDLE fd);
int flush_buffer_tx(HANDLE fd);
int send_break();


#endif
