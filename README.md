# teensy-microROS-vl6180

This is a package designed to run two Time-of-Flight (ToF) using a Teensy board. Currently designed for teensy36, with the given pin configuration

ToF sensors are both VL6180s from ST Microelectronics.

## Extra Packages
Custom colcon packages can be included in `./extra_packages`. This is especially useful for custom message packages. Packages can be directly placed in this directory, however it is recommended to use the `extra_packages.repos` file for easy project scaling.

## Building microROS for PlatformIO.

Run the following commands:

```
pio lib install # Install dependencies
pio run # Build the firmware
pio run --target upload # Flash the firmware
```

After the library is compiled for first time the build process will be skipped, to trigger a library build and apply library modifications on your next PlatformIO build:

```
pio run --target clean_microros  # Clean library
```