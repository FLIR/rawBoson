
CC=g++ 

CFLAGS= -Wall  


EJECUTABLE=rawBoson


CFILES= serial.c rawBoson.c bytes.c
OFILES= serial.o rawBoson.o bytes.o

$(EJECUTABLE):	$(OFILES)
		$(CC)  $(CFLAGS) -o  $(EJECUTABLE) $(OFILES)

%.o:	%.c
	$(CC) -c $(CFLAGS)  $< -o $@ 

clean:
	rm -f $(OFILES) $(EJECUTABLE)
