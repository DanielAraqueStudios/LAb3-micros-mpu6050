MPU6050 PyQt6 Live Viewer

Files:
- read_mpu_gui.py : main PyQt6 application. Reads CSV lines from serial and plots values.
- requirements.txt : Python dependencies

Quick start:
1. Install environment (recommend virtualenv):

python3 -m venv venv
. venv/bin/activate
pip install -r requirements.txt

2. Run GUI (adjust port in the app):

python read_mpu_gui.py

Notes:
- The app looks for serial ports and shows them in the dropdown. Select `/dev/ttyUSB0` (where your CSV stream appears) and press Connect.
- Values shown are converted using MPU6050 default scales: accel LSB=16384 per g, gyro LSB=131 per °/s. Temperature conversion uses (raw/340)+36.53.

Example UART output line (one sample per line):

		1024,-34,16384,12,5,-2,3400

This corresponds to: ax,ay,az,gx,gy,gz,temp (raw register integers).

How I2C is handled in the firmware
----------------------------------
The ESP32-S3 firmware in `main/main.c` uses ESP-IDF's I2C master driver to talk to the MPU6050.

Key points:
- I2C address: the MPU6050 default 7-bit address 0x68 is used (AD0 pulled low). If AD0 is high,
	the address becomes 0x69 — check the board wiring.
- Initialization: the firmware writes to the MPU6050 power-management and configuration registers to
	wake the device and set sensor ranges/sample rate. Typical writes include:

	- PWR_MGMT_1 (0x6B): clear the sleep bit to wake the sensor (write 0x00).
	- SMPLRT_DIV (0x19): sample rate divider (affects internal sample rate).
	- CONFIG (0x1A): digital low-pass filter setting.
	- GYRO_CONFIG (0x1B): gyro full-scale range (e.g., 0x00 for ±250 °/s).
	- ACCEL_CONFIG (0x1C): accel full-scale range (e.g., 0x00 for ±2 g).

- Reading sensor data: the firmware performs a single I2C burst read starting at the ACCEL_XOUT_H
	register (0x3B). It reads 14 bytes in sequence which contain:

	ACCEL_XOUT_H (0x3B), ACCEL_XOUT_L (0x3C),
	ACCEL_YOUT_H (0x3D), ACCEL_YOUT_L (0x3E),
	ACCEL_ZOUT_H (0x3F), ACCEL_ZOUT_L (0x40),
	TEMP_OUT_H (0x41), TEMP_OUT_L (0x42),
	GYRO_XOUT_H (0x43), GYRO_XOUT_L (0x44),
	GYRO_YOUT_H (0x45), GYRO_YOUT_L (0x46),
	GYRO_ZOUT_H (0x47), GYRO_ZOUT_L (0x48).

	The firmware converts each 16-bit pair from big-endian two's complement into signed integers
	and packs them into the CSV line sent over UART.

- Error handling: I2C transaction failures are reported via UART log messages (or to the monitor
	when flashed via `idf.py monitor`). The firmware retries or halts depending on the error handling
	code paths in `main.c`.

If you want, I can also add a small annotated excerpt of the actual `main.c` I2C calls to the README
so readers can see the exact ESP-IDF APIs used (i2c_master_write_to_device/i2c_master_read_from_device).
