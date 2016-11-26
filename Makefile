.PHONY: prog

prog2: ovi2.hex
	avrdude -c usbtiny -p ATMEGA128 -U flash:w:ovi2.hex

ovi1.hex: ovi1.elf $(shell test -e .eeprom && echo .eeprom)
	avr-objcopy -O ihex -R .eeprom ovi1.elf ovi1.hex

ovi2.hex: ovi2.elf $(shell test -e .eeprom && echo .eeprom)
	avr-objcopy -O ihex -R .eeprom ovi2.elf ovi2.hex

ovi1.elf: ovi1.o hal1.o
	avr-gcc -mmcu=atmega328p ovi1.o hal1.o -o ovi1.elf

ovi2.elf: ovi2.o hal2.o
	avr-gcc -mmcu=atmega128 ovi2.o hal2.o -o ovi2.elf

ovi1.o: ovi.c
	avr-gcc -Wall -pedantic -std=c99 -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o ovi1.o ovi.c

hal1.o: hal1.c
	avr-gcc -Wall -pedantic -std=c99 -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o hal1.o hal1.c

ovi2.o: ovi.c
	avr-gcc -Wall -pedantic -std=c99 -Os -DF_CPU=7372000UL -mmcu=atmega128 -c -o ovi2.o ovi.c

hal2.o: hal2.c
	avr-gcc -Wall -pedantic -std=c99 -Os -DF_CPU=7372000UL -mmcu=atmega128 -c -o hal2.o hal2.c
