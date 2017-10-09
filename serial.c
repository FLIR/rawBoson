#include "serial.h"
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>


int open_port(PortSettingsType ps,HANDLE *handle) {
	if ( (*handle = open(ps.port,O_RDWR | O_NOCTTY | O_NONBLOCK )) == -1 )
	{
//		printf("Error opening port %s\n",ps.port);
		return ( -1 );
	}
	else
	{
		if ( !setup_serial( *handle, ps.baudrate, ps.databits, ps.stopbits, ps.parity)) {
			close(*handle);
			return (-2);
		}
		else {
//			printf("Port %s opened\n",ps.port);
			return 0;
		}
	}
}


int close_port(HANDLE handle) {
//	printf("Closing serial port\n");
	return close(handle);
}


int send_to_serial(HANDLE fd, unsigned char *tx_array, short bytes_to_send) {
	if ( write(fd, tx_array,bytes_to_send)==-1) {
 	  	printf("Error transmiting serial bytes\n");
   		return 0;
  	}
  	return 1;
}


int send_to_serial_car(HANDLE fd, char car) {
	if ( write(fd, &car, 1)==-1) {
    		printf("Error transmiting serial bytes\n");
   		return 0;
  	}
  	return 1;
}


int car_waiting(HANDLE fd) {
	int n;
	if (ioctl(fd,TIOCINQ,&n)==0) {
		return n;
	}
	return 0;
}


int bufftx_waiting(HANDLE fd){
  int n=0;
  if (ioctl(fd,TIOCOUTQ,&n)==0) {
    return n;
  }
  return 0;
}



char leer_car_plazo(HANDLE fd,int plazo, int *timeout) {
  fd_set leer;
  struct timeval tout;
  int n;
  char c;

  tout.tv_sec=0;
  tout.tv_usec=plazo;

  FD_ZERO(&leer);
  FD_SET(fd,&leer);

  n=select(fd+2,&leer,NULL,NULL,&tout);
  if (n==0) {
    *timeout=1;
    return 0;
  }
  *timeout=0;
  read(fd,&c,1);
  return c;
}


char read_serial_car(HANDLE fd) {
	char c;

	while (!car_waiting(fd));
	read(fd,&c,1);
	return c;
}


int flush_buffer_tx(HANDLE fd) {
  if (tcflush(fd,TCOFLUSH)==-1) {
  //  printf("Error flushing TX buffer\n");
    return 0;
  }
  return 1;
}


int flush_buffer_rx(HANDLE fd) {
  if (tcflush(fd,TCIFLUSH)==-1) {
  //  printf("Error flushing RX buffer\n");
    return 0;
  }
  return 1;
}


int send_break(HANDLE fd) {
  if (tcsendbreak(fd,1)==-1) {
  //  printf("Error when sending BREAK\n");
    return 0;
  }
  return 1;
}



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



int setup_serial(int fdes,int baud,int databits,int stopbits,int parity) {
	int n;
	struct termios options;

	/* Get the current options */
	if (tcgetattr(fdes,&options) != 0) {
		return -1;
	}

	/* Set the baud rate */
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
		printf("Unsupported baud rate\n");
		return -1;
	}
	if (n != 0) {
		return -1;
	}

	/* Set the data size */
	options.c_cflag &= ~CSIZE;
	switch (databits) {
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		printf("Unsupported data size\n");
		return -1;
	}

	/* Set up parity */
	switch (parity) {
	case NOPARITY:
		options.c_cflag &= ~PARENB; /* Clear parity enable */
		options.c_iflag &= ~INPCK; /* Enable parity checking */
		break;
	case ODDPARITY:
		options.c_cflag |= (PARODD | PARENB); /* Enable odd parity */
		options.c_iflag |= INPCK;  /* Disnable parity checking */
		break;
	case EVENPARITY:
		options.c_cflag |= PARENB; /* Enable parity */
		options.c_cflag &= ~PARODD; /* Turn odd off => even */
		options.c_iflag |= INPCK; /* Disnable parity checking */
		break;
	default:
		printf("Unsupported parity\n");
		return -1;
	}

	/* Set up stop bits */
	switch (stopbits) {
	case ONESTOPBIT:
		options.c_cflag &= ~CSTOPB;
		break;
	case TWOSTOPBITS:
		options.c_cflag |= CSTOPB;
		break;
	default:
		printf("Unsupported stop bits\n");
		return -1;
	}

	/* Set input parity option */
	if (parity != NOPARITY)
		options.c_iflag |= INPCK;

	/* Deal with hardware or software flow control */
	options.c_cflag &= ~CRTSCTS; /* Disable RTS/CTS */
	//options.c_iflag |= (IXANY); /* xon/xoff flow control */
	options.c_iflag &= ~(IXON|IXOFF|IXANY); /* xon/xoff flow control */ 

	/* Output processing */
	options.c_oflag &= ~OPOST; /* No output processing */
	options.c_oflag &= ~ONLCR; /* Don't convert linefeeds */

	/* Input processing */
	options.c_iflag |= IGNBRK; /* Ignore break conditions */
	options.c_iflag &= ~IUCLC; /* Don't map upper to lower case */
	options.c_iflag &= ~BRKINT; /* Ignore break signals */
	options.c_iflag &= ~INLCR; /*Map NL to CR */
	options.c_iflag &= ~ICRNL; /*Map CR to NL */

	/* Miscellaneous stuff */
	options.c_cflag |= (CLOCAL | CREAD); /* Enable receiver, set local */
	/* Linux seems to have problem with the following ??!! */
//	options.c_cflag |= (IXON | IXOFF); /* Software flow control */
	options.c_lflag = 0; /* no local flags */
	options.c_cflag |= HUPCL; /* Drop DTR on close */

	/* Setup non blocking, return on 1 character */
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 1;

	/* Clear the line */
	tcflush(fdes,TCIFLUSH);

	/* Update the options and do it NOW */
	if (tcsetattr(fdes,TCSANOW,&options) != 0) {
		return -1;
	}

	return -1;
}
