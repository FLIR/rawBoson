#ifndef comms_h
#define comms_h

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BYTES_READ			1
#define COLARX				100
#define COLATX				10
#define HANDLE   int

#define NOPARITY            0   // comm
#define ODDPARITY           1   // comm
#define EVENPARITY          2   // comm
#define MARKPARITY          3   // comm
#define SPACEPARITY         4   // comm

#define ONESTOPBIT          0   // comm
#define ONE5STOPBITS        1   // comm
#define TWOSTOPBITS         2   // comm


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
int send_to_serial(HANDLE fd, unsigned char* tx_array, short bytes_to_send);
int send_to_serial_car(HANDLE fd, char car);
int car_waiting(HANDLE fd);
char read_serial_car(HANDLE fd);
int flush_buffer_rx(HANDLE fd);
int flush_buffer_tx(HANDLE fd);
int send_break();


#endif
