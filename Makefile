
CC=gcc

CFLAGS=-std=c99 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror
CPPFLAGS=-I.

EXECUTABLE=rawBoson


CFILES= serial.c rawBoson.c bytes.c
OFILES= serial.o rawBoson.o bytes.o

$(EXECUTABLE):	$(OFILES)
		$(CC)  $(CFLAGS) -o  $(EXECUTABLE) $(OFILES)

%.o:	%.c
	$(CC) -c $(CFLAGS)  $< -o $@ 

clean:
	rm -f $(OFILES) $(EXECUTABLE)
