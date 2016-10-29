.PHONY: prog

prog: ovi.hex
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyACM0 -b 115200 -U flash:w:ovi.hex
ovi.hex: ovi $(shell test -e .eeprom && echo .eeprom)
	avr-objcopy -O ihex -R .eeprom ovi ovi.hex
ovi: ovi.o
	avr-gcc -mmcu=atmega328p ovi.o -o ovi
ovi.o: ovi.c
	avr-gcc -Wall -pedantic -std=c99 -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o ovi.o ovi.c
