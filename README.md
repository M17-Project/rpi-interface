# rpi-interface
Raspberry Pi interface for the M17 Project's [CC1200 HAT](https://github.com/M17-Project/CC1200_HAT-hw).

### Building
To build the code using `make`, first install *libzmq3-dev* and *libgpiod-dev*:<br>
`sudo apt install libzmq3-dev libgpiod-dev`

### Command line arguments
```
-r reset the device
-c path to the configuration file
```
The `-c` argument followed by a path to the config text file is mandatory.

### Config file structure
A sample config file is provided - `default_cfg.txt`.
"PA_EN" pin is not used by the CC1200 HAT (set it to any unused pin).

### UART access
The device uses `/dev/ttyAMA0` which is available through `miniuart-bt` overlay. On a Raspberry Pi Zero 2,
use `sudo nano /boot/firmware/config.txt` and add `btoverlay=miniuart-bt` line.
