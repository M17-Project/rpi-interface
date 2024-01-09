all: rpi-interface

rpi-interface: rpi-interface.c
	gcc -I M17_Implementations/SP5WWP/include -L M17_Implementations/SP5WWP/lib -O2 -Wall -Wextra rpi-interface.c -o rpi-interface -lm -lm17

install: all
	sudo install rpi-interface /usr/local/bin

clean:
	rm -f rpi-interface