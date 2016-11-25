.PHONY: prog

prog: ovi.hex
	avrdude -c usbtiny -p ATMEGA128 -U flash:w:ovi.hex
ovi.hex: ovi $(shell test -e .eeprom && echo .eeprom)
	avr-objcopy -O ihex -R .eeprom ovi ovi.hex
ovi: ovi.o
	avr-gcc -mmcu=atmega128 ovi.o -o ovi
ovi.o: ovi.c
	avr-gcc -Wall -pedantic -std=c99 -Os -DF_CPU=7372000UL -mmcu=atmega128 -c -o ovi.o ovi.c
