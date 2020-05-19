MCU=atmega328p
F_CPU=1200000
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-std=c99 -Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I.
TARGET=blinky-led
SRCS=$(TARGET).c
PORT ?= /dev/ttyUSB0

all:
	${CC} ${CFLAGS} -o ${TARGET}.bin ${SRCS}
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.bin ${TARGET}.hex

flash:
	avrdude -p ${MCU} -b 19200 -c arduino -U flash:w:${TARGET}.hex:i -F -P $(PORT)

clean:
	rm -f *.bin *.hex
