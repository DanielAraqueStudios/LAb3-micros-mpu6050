# _Sample project_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This is the simplest buildable example. The example is used by command `idf.py create-project`
that copies the project to user specified path and set it's name. For more information follow the [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project)



## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.

## Backend (ESP32-S3 firmware)

This repository includes an ESP32-S3 firmware (in `main/main.c`) that reads motion and temperature
data from an MPU6050 over I2C and streams the sensor values over a UART serial link for real-time
visualization by the provided PyQt6 frontend.

What the firmware does
- Initializes I2C and configures the MPU6050 (accelerometer and gyroscope) into a continuous
	sampling mode with a fixed sample rate (configured in `main.c`).
- Reads raw accelerometer (ax, ay, az) and gyroscope (gx, gy, gz) 16-bit integer samples from the
	MPU6050 at the configured sampling rate.
- Reads the raw temperature register and converts it to degrees Celsius using the MPU6050 formula.
- (Optional) Applies minimal preprocessing on the MCU such as basic scaling to raw sensor units.
- Packages each sample into a simple ASCII CSV line and sends it over UART at 115200 baud. The
	format per line is:

	ax,ay,az,gx,gy,gz,temp\n

	where each value is an integer read directly from the sensor registers. The Python GUI expects
	this format and converts/scales the raw integers into g, °/s and °C for display.

Configuration & behavior
- I2C bus pins, UART port and baud, and sample rate are configured in `main.c` (search for
	I2C and UART init). Adjust those defines to match your hardware wiring.
- The firmware runs a simple FreeRTOS task that polls the MPU6050 at the selected rate and writes
	packed ASCII lines to the UART. It closes the UART cleanly on shutdown.

Flashing and testing
1. Install ESP-IDF and set up your environment following the official docs for your platform.
2. From the project root run:

	 idf.py set-target esp32s3
	 idf.py menuconfig   # (optional) change UART/I2C pins or sample rate
	 idf.py -p /dev/ttyUSB0 flash monitor

3. When the device is running, connect the Python GUI (`frontend/read_mpu_gui.py`) to the same
	 serial port. The GUI expects CSV lines as described above and will convert raw values to human
	 units for plotting and the 3D orientation display.

Notes and troubleshooting
- If the GUI shows garbled data, verify the UART baud rate, port, and confirm the firmware is
	outputting comma-separated integer values (use `idf.py monitor` or a serial terminal).
- If the MPU6050 is not responding, check I2C wiring (SDA, SCL, GND, VCC) and power rails.
- The firmware is intentionally minimal: complex sensor fusion or filtering is performed in the
	Python frontend for flexibility during visualization and experimentation.
