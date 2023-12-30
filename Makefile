all: rpi-interface

rpi-interface: rpi-interface.c
	gcc -I lib -L lib -O2 -Wall rpi-interface.c -o rpi-interface -lm -lm17

install: all
	sudo install rpi-interface /usr/local/bin

clean:
	rm -f rpi-interface