# rpi-interface
Raspberry Pi interface for the M17 Project's [CC1200 HAT](https://github.com/M17-Project/CC1200_HAT-hw).

### Command line arguments
```
-r reset the device
-i IPv4 address of the reflector
-c path to the configuration file
```
The `-c` argument followed by a path to the config text file is mandatory.

### Config file structure
A sample config file is provided - `default_cfg.txt`.
"PA_EN" pin is not used by the CC1200 HAT (set it to any unused pin).
