/*
MIT License

Copyright 2002 Ifara Tecnologias S.L.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


// Error handling convention
// Returns 0 when no error
// Returns <0 when error
// >0 is the byte read


#ifndef SERIAL_H
#define SERIAL_H

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

// Opens serial port
// Input  : PortSettingType
// Output : handle to the device
// Returns 0 on success
// Returns -1 if not capable of opening the serial port handler
// Returns -2 if serial port settings cannot be applied
int open_port(PortSettingsType ps,HANDLE *handle);

// Set the serial port configuration ( baudrate, databits, stopbits, parity)
// Returns  0 if OK
// Returns -1 if cannot get serial port configuration
// Returns -2 if baudrate not supported
// Returns -3 if selected baudrate didn't work
// Returns -4 if databits selection failed
// Returns -5 if parity selection failed
// Returns -6 if stopbits selection failed
// Returns -7 if cannot update new options
int setup_serial(int, int, int, int, int);

// Close the serial port handle
// Returns 0 on success
// Returns -1 on error , and errno is updated.
int close_port(HANDLE handle);

// Define serial port settings
// str1 -> serial port name
// str2 -> serial port setting baud,databits, parity, stop bits
// output PS structure
PortSettingsType str2ps (char *str1, char*str2);

// Send buffer of  bytes
// Input: serial port handler, pointer to first byte of buffer , number of bytes to send
// Returns 0 on success
// Returns -1 on error
int send_buffer(HANDLE fd, unsigned char* tx_array, short bytes_to_send);

// Send one byte
// Input: serial port handler, byte to send
// Returns 0 on success
// Returns -1 on error
int send_byte(HANDLE fd, unsigned char car);

// Check if there is a byte or not waiting to be read (RX)
// Returns 'n' > 0 as numbers of bytes to read
// Returns 0 is there is no byte waiting
int rxbyte_waiting(HANDLE fd);

// Check if there is a byte or not waiting to be sent (TX)
// Returns 'n' > 0 as numbers of bytes to send
// Returns 0 is there is no byte waiting
int txbyte_waiting(HANDLE fd);

// Read a byte within a period of time
// Returns byte read
// Returns -1 if timeout happened.
// timeout = 0 if no timeout, timeout = 1 if timeout
unsigned char read_byte_time(HANDLE fd, int plazo, int *timeout);

// Read a byte. Blocking call. waits until byte is received
// Returns byte read
unsigned char read_byte(HANDLE fd);

// Flush RX buffer
// Returns -1 if error
// Returns 0 if OK
int flush_buffer_rx(HANDLE fd);

// Flush TX buffer
// Returns -1 if error
// Returns 0 if OK
int flush_buffer_tx(HANDLE fd);

// Sends break signal
// Returns -1 if error
// Returns 0 if OK
int send_break();

#endif
