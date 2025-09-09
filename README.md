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

Annotated I2C code excerpt (from `main/main.c`)
--------------------------------------------
Below are the key ESP-IDF I2C calls used by the firmware with short notes. See `main/main.c`
for the full implementation.

```c
// configure and install I2C master driver
esp_err_t i2c_master_init(void)
{
		i2c_config_t conf = { .mode = I2C_MODE_MASTER, /* sda/scl pins, pullups, clk speed */ };
		i2c_param_config(I2C_MASTER_NUM, &conf);
		return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// wake MPU6050 by writing 0x00 to PWR_MGMT_1 (0x6B)
esp_err_t mpu6050_init(void)
{
		uint8_t data[2] = { MPU6050_PWR_MGMT_1, 0x00 };
		// i2c_master_write_to_device sends the address+data as a single write
		return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, sizeof(data), 100 / portTICK_PERIOD_MS);
}

// read 14 bytes starting at ACCEL_XOUT_H (0x3B) with a single combined write-then-read
esp_err_t mpu6050_read(mpu6050_data_t *out)
{
		uint8_t reg = MPU6050_ACCEL_XOUT_H;
		uint8_t buf[14];
		// i2c_master_write_read_device does a write of the register offset, then a read of N bytes
		esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
		if (ret != ESP_OK) return ret;
		// convert big-endian pairs to signed 16-bit values
		out->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
		... // rest of conversion
		return ESP_OK;
}
```

Notes:
- `i2c_master_write_to_device` is used for simple register writes (initialization).
- `i2c_master_write_read_device` is convenient for register pointer + burst reads and is used
	here to fetch accel/gyro/temp in one transaction.
- Timeout values and retries are set via the last parameter (portTICK_PERIOD_MS) and should be
	adjusted for noisy buses or slow peripherals.
