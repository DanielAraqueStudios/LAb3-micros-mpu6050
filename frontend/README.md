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
