# rpi-interface
Raspberry Pi interface for the M17 Project's [Remote Radio Unit](https://github.com/M17-Project/rru-rf-hw).

### Command line arguments
```
-r reset the device
-i IPv4 address of the reflector
-c path to the configuration file
```
The `-c` argument followed by a path to the config text file is mandatory.

### Config file structure
A sample config file is provided - `default_cfg.txt`.
"PA_EN" pin is not used by every device (set it to any unused pin if it's not required).

### Flashing the target device
Use the command below to flash the target device over UART (by default that's `/dev/ttyAMA0`).
Make sure the overlay is set to `dtoverlay=miniuart-bt` and you have [stm32flash](https://sourceforge.net/p/stm32flash/wiki/Home/) installed.
You can set the overlay by editing the `/boot/firmware/config.txt` file. This moves `/dev/ttyAMA0` to where UART RX/TX pins are (GPIO15/14).
Flashing over UART: `stm32flash -v -R -i "-20&-21&20,21,:-20,-21,21" -w CC1200_HAT-fw.bin /dev/ttyAMA0`
