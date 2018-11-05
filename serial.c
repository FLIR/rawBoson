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

#include "serial.h"

// Opens serial port
// Input  : PortSettingType
// Output : handle to the device
// Returns 0 on success
// Returns -1 if not capable of opening the serial port handler
// Returns -2 if serial port settings cannot be applied
int open_port(PortSettingsType ps,HANDLE *handle) {
	if ( (*handle = open(ps.port,O_RDWR | O_NOCTTY | O_NONBLOCK )) == -1 )
	{
		return -1; // cannot open serial port
	}
	else
	{
		if ( setup_serial( *handle, ps.baudrate, ps.databits, ps.stopbits, ps.parity)!=0 ) {
			close(*handle);
			return -2; // cannot apply settings
		}
		else {
			return 0;  // Serial port opened and configured correctly
		}
	}
}

// Close the serial port handle
// Returns 0 on success
// Returns -1 on error , and errno is updated.
int close_port(HANDLE handle) {
	return close(handle);
}

// Send buffer of  bytes
// Input: serial port handler, pointer to first byte of buffer , number of bytes to send
// Returns 0 on success
// Returns -1 on error
int send_buffer(HANDLE fd, unsigned char *tx_array, short bytes_to_send) {
	if ( write(fd, tx_array,bytes_to_send)==-1) {
   		return -1;   // Error
  	}
  	return 0;  // Success
}

// Send one byte
// Input: serial port handler, byte to send
// Returns 0 on success
// Returns -1 on error
int send_byte(HANDLE fd, unsigned char car) {
	if ( write(fd, &car, 1)==-1) {
   		return -1;
  	}
  	return 0;
}

// Check if there is a byte or not waiting to be read (RX)
// Returns 'n' > 0 as numbers of bytes to read
// Returns 0 is there is no byte waiting
int rxbyte_waiting(HANDLE fd) {
	int n;
	if (ioctl(fd,TIOCINQ,&n)==0) {
		return n;
	}
	return 0;
}

// Check if there is a byte or not waiting to be sent (TX)
// Returns 'n' > 0 as numbers of bytes to send
// Returns 0 is there is no byte waiting
int txbyte_waiting(HANDLE fd) {
  int n=0;
  if (ioctl(fd,TIOCOUTQ,&n)==0) {
    return n;
  }
  return 0;
}

// Read a byte within a period of time
// Returns byte read
// Returns -1 if timeout happened.
// timeout = 0 if no timeout, timeout = 1 if timeout
unsigned char read_byte_time(HANDLE fd,int plazo, int *timeout) {
  fd_set leer;
  struct timeval tout;
  int n;
  unsigned char c;

  tout.tv_sec=0;
  tout.tv_usec=plazo;

  FD_ZERO(&leer);
  FD_SET(fd,&leer);

  n=select(fd+2,&leer,NULL,NULL,&tout);
  if (n==0) {
    *timeout=1;
    return -1;
  }
  *timeout=0;

  read(fd,&c,1);
  return c;
}

// Read a byte. Blocking call. waits until byte is received
// Returns byte read
unsigned char read_byte(HANDLE fd) {
	unsigned char c;

	while (!rxbyte_waiting(fd));
	read(fd,&c,1);
	return c;
}

// Flush TX buffer
// Returns -1 if error
// Returns 0 if OK
int flush_buffer_tx(HANDLE fd) {
  if (tcflush(fd,TCOFLUSH)==-1) {
    return -1;
  }
  return 0;
}

// Flush RX buffer
// Returns -1 if error
// Returns 0 if OK
int flush_buffer_rx(HANDLE fd) {
  if (tcflush(fd,TCIFLUSH)==-1) {
    return -1;
  }
  return 0;
}

// Sends break signal
// Returns -1 if error
// Returns 0 if OK
int send_break(HANDLE fd) {
  if (tcsendbreak(fd,1)==-1) {
    return -1;
  }
  return 0;
}

// Define serial port settings
// str1 -> serial port name
// str2 -> serial port setting baud,databits, parity, stop bits
// output PS structure
PortSettingsType str2ps (char *str1, char*str2) {

	PortSettingsType ps;
	char parity;
	char stopbits[4];

	// Default values (just in case)
	ps.baudrate=9600;
	ps.databits=8;
	ps.parity=NOPARITY;
	ps.stopbits=ONESTOPBIT;

	sprintf(ps.port,"%s",str1);

	if (sscanf(str2,"%d,%d,%c,%s",&ps.baudrate,&ps.databits,&parity,stopbits) == 4) {
		switch (parity) {
			case 'e':
				ps.parity=EVENPARITY;
				break;
			case 'o':
				ps.parity=ODDPARITY;
				break;
			case 'm':
				ps.parity=MARKPARITY;
				break;
			case 's':
				ps.parity=SPACEPARITY;
				break;
			case 'n':
				ps.parity=NOPARITY;
				break;
		}
		if (! strcmp(stopbits,"1")) {
			ps.stopbits=ONESTOPBIT;
		}
		else if (! strcmp(stopbits,"1.5")) {
			ps.stopbits=ONE5STOPBITS;
		}
		else if (! strcmp(stopbits,"2")) {
			ps.stopbits=TWOSTOPBITS;
		}
	}
	return ps;
}

// Set the serial port configuration ( baudrate, databits, stopbits, parity)
// Returns  0 if OK
// Returns -1 if cannot get serial port configuration
// Returns -2 if baudrate not supported
// Returns -3 if selected baudrate didn't work
// Returns -4 if databits selection failed
// Returns -5 if parity selection failed
// Returns -6 if stopbits selection failed
// Returns -7 if cannot update new options
int setup_serial(int fdes,int baud,int databits,int stopbits,int parity) {
  int n;
  struct termios options;

  // Get the current options
  if (tcgetattr(fdes,&options) != 0) {
    // error getting the serial port configuration options
    return -1;
  }

	// Set the baud rate
	switch (baud) {
	case 2400:
		n =  cfsetospeed(&options,B2400);
		n += cfsetispeed(&options,B2400);
		break;
	case 4800:
		n =  cfsetospeed(&options,B4800);
		n += cfsetispeed(&options,B4800);
		break;
	case 9600:
		n =  cfsetospeed(&options,B9600);
		n += cfsetispeed(&options,B9600);
		break;
	case 19200:
		n =  cfsetospeed(&options,B19200);
		n += cfsetispeed(&options,B19200);
		break;
	case 38400:
		n =  cfsetospeed(&options,B38400);
		n += cfsetispeed(&options,B38400);
		break;
	case 57600:
		n =  cfsetospeed(&options,B57600);
		n += cfsetispeed(&options,B57600);
		break;
	case 115200:
		n =  cfsetospeed(&options,B115200);
		n += cfsetispeed(&options,B115200);
		break;
	case 230400:
		n =  cfsetospeed(&options,B230400);
		n += cfsetispeed(&options,B230400);
		break;
	case 921600:
		n =  cfsetospeed(&options,B921600);
		n += cfsetispeed(&options,B921600);
		break;

	default:
	  // not supported baudrate
		return -2;
	}
  // If n != 0 then Baud Rate selection didn't work
	if (n != 0) {
		return -3;  // Error settig the baud rate
	}

	// Set the data size
	options.c_cflag &= ~CSIZE;
	switch (databits) {
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		// Not supported data size
		return -4;
	}

	// Set up parity
	switch (parity) {
	case NOPARITY:
		options.c_cflag &= ~PARENB;  // Clear parity enable
		options.c_iflag &= ~INPCK;   // Enable parity checking
		break;
	case ODDPARITY:
		options.c_cflag |= (PARODD | PARENB); // Enable odd parity
		options.c_iflag |= INPCK;  // Disnable parity checking
		break;
	case EVENPARITY:
		options.c_cflag |= PARENB;  // Enable parity
		options.c_cflag &= ~PARODD; // Turn odd off => even
		options.c_iflag |= INPCK;   // Disnable parity checking
		break;
	default:
		// Unsupported parity
		return -5;
	}

	// Set up stop bits
	switch (stopbits) {
	case ONESTOPBIT:
		options.c_cflag &= ~CSTOPB;
		break;
	case TWOSTOPBITS:
		options.c_cflag |= CSTOPB;
		break;
	default:
		// "Unsupported stop bits
		return -6;
	}

	// Set input parity option
	if (parity != NOPARITY)
		options.c_iflag |= INPCK;

	// Deal with hardware or software flow control
	options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS
	//options.c_iflag |= (IXANY); // xon/xoff flow control
	options.c_iflag &= ~(IXON|IXOFF|IXANY); // xon/xoff flow control

	// Output processing
	options.c_oflag &= ~OPOST;  // No output processing
	options.c_oflag &= ~ONLCR;  // Don't convert linefeeds

	// Input processing
	options.c_iflag |= IGNBRK;  // Ignore break conditions
	options.c_iflag &= ~IUCLC;  //  Don't map upper to lower case
	options.c_iflag &= ~BRKINT; // Ignore break signals
	options.c_iflag &= ~INLCR;  // Map NL to CR
	options.c_iflag &= ~ICRNL;  // Map CR to NL

	// Miscellaneous stuff
	options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, set local
	// Linux seems to have problem with the following ?
	//	options.c_cflag |= (IXON | IXOFF); // Software flow control
	options.c_lflag = 0;  // no local flags
	options.c_cflag |= HUPCL; // Drop DTR on close

	// Setup non blocking, return on 1 character
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 1;

	// Clear the line
	tcflush(fdes,TCIFLUSH);

	// Update the options and do it NOW
	if (tcsetattr(fdes,TCSANOW,&options) != 0) {
		return -7;
	}

	return 0;   // Serial port configured correctly
}
