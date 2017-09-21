# Definicion del compilador y los parametros de compilacion

CC=g++ 
# Amba
#CC=/usr/local/linaro-armv7ahf-2015.11-gcc5.2/bin/arm-linux-gnueabihf-g++
CFLAGS= -Wall  

# dependencias

EJECUTABLE=rawBoson

#
# Fichero objeto, fuente y .H
#

CFILES= serial.c rawBoson.c bytes.c
OFILES= serial.o rawBoson.o bytes.o

$(EJECUTABLE):	$(OFILES)
		$(CC)  $(CFLAGS) -o  $(EJECUTABLE) $(OFILES)

%.o:	%.cpp
	$(CC) -c $(CFLAGS)  $< -o $@ 

clean:
	rm -f $(OFILES) $(EJECUTABLE)
