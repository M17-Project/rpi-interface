all: rpi-interface

rpi-interface: rpi-interface.c
	gcc -O2 -Wall -Wextra -Wno-unused-result rpi-interface.c -o rpi-interface -lm -lm17 -lzmq -lgpiod

install: all
	sudo install rpi-interface /usr/local/bin

clean:
	rm -f rpi-interface