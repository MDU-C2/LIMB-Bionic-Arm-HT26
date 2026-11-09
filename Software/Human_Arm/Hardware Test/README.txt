REQUIRED PYTHON PACKAGES:
The Python scripts in this project require the following packages:
1. pyserial
   - Used to read EMG and IMU data from the ESP32 via USB/Serial.
2. pandas
   - Used to read, store, process and filter recorded CSV data.
3. matplotlib
   - Used to plot and visualize EMG and IMU data.


INSTALLATION:
There are two ways to install all required Python packages.

Option 1 - Install manually:
Run the following command in the terminal:
python -m pip install pyserial pandas matplotlib

Option 2 - Install from requirements.txt:
Run:
python -m pip install -r requirements.txt
This installs all Python dependencies listed in the requirements.txt file.

VERIFY INSTALLATION:
To verify that the packages were installed correctly, run:
python -m pip show pyserial pandas matplotlib

NOTE:
Make sure that the Python interpreter used in VS Code is the same Python installation where the packages were installed.
You can check the active Python version with:
python --version