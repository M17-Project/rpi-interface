all: rpi-interface

rpi-interface: rpi-interface.c
	gcc -O2 -Wall rpi-interface.c -o rpi-interface -lm

install: all
	sudo install rpi-interface /usr/local/bin

clean:
	rm -f rpi-interface