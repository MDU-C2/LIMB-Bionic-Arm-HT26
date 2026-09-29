REQUIRED INSTALLATION PLUGINS:
1. pyserial to read EMG/IMU data from ESP32 via USB
2. pandas to read, handle and filter CSV data
3. matplotlib to visualize EMG and IMU data

There are 2 ways to install all required plugins(run this in the terminal):
1. python -m pip install pyserial pandas matplotlib
(manually)
2. python -m pip install -r requirements.txt
(required Python depenencies from the requirement text file)