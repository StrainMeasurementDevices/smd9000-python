Getting Started
========================================

Hardware Connection
---------------------------------
This Python package use a USB CDC (serial) device to communicate to an SMD9000 sensor over UART. If you have an SMD9097
cable, connect that to the sensor and computer.
For any other custom USB-UART bridge, ensure you have the right connections going to the sensor.

Determining COM/dev Port
---------------------------------
When you plug in a USB to UART converter, such as an FT232, it will take up a COM port number if on Windows or a device
name on Linux. It is important to know what device number the USB to UART converter uses in order to communicate to it.

Windows
++++++++++++++++++++++++
If you are on Windows, the easiest way to find out the COM port number is to open Device Manager, which is found by
searching for it in the Search taskbar.
Device Manager should be open before you plug in the sensor. Under the "Ports (COM & LPT)" section, open it if it exists
and take note of existing serial devices.
When you plug in the sensor, you will see Device Manager refresh, and under the Port section, you should see a new
device. That is the COM port that is your USB to UART converter.

Linux or UNIX based
++++++++++++++++++++++++
On Linux distributions or UNIX based operating systems like MacOS, open up a terminal. If you run the command
`ls /dev/ | grep 'USB\|ACM'`
You should either see nothing, or some devices if you already have a CDC device connected. Now connect the sensor, and
run the command again. You should see a new device under the name `ttyUSBx` or `ttyACMx`. This is the device name for
the UART to UART converter.

Python Package Installation
---------------------------------
To install the SMD9000 Python package, if you have Python setup on your system simply run the following command after cloning the package repository:
`pip install <PATH-TO-DOWNLOADED-FILE>`

