#!/usr/bin/env python3
"""
    SMD9000 Python Package
"""
import sys
import serial                                   # Import pyserial to interface to the sensor
import serial.tools.list_ports as serial_tools  # Import the list_ports tools from pyserial to find the SMD9000 sensor.
import logging                                  # Import logging to log
import threading                                # Import to create a lock
import dataclasses

class SMD9000ReadException(Exception):
    pass

@dataclasses.dataclass
class SMD9000Info:
    firmware_major_rev: int
    firmware_minor_rev: int
    hardware_rev: str

class SMD9000:
    def __init__(self):
        """Initialization of the SMD9000 class"""
        self.is_connected = False               # Flag to determine if the sensor is connected
        self._ser = None                        # Reference to a serial class object
        self._ser_lock = threading.Lock()       # Create a lock, in case the library is to be used with multiple threads
        self._baud_rate = 115200                # Baud Rate of the sensor
        self._log = logging.getLogger('SMD9000')    # Get a logger with the name 'SMD9000'
        self._read_thread = threading.Thread(target=self._read_thread)

    def get_available_sensors(self) -> list:
        available_list = []
        device_list = serial_tools.comports()
        for x in device_list:
            try:
                if sys.platform == 'win32':
                    if x.device == 'COM1':
                        continue
                    if 'SMD' in x.serial_number and x.vid == 0x0403 and x.pid == 0x6001:
                        available_list.append(x.device)
                # If using a REAL operating system
                else:
                    if 'SMD9000' in x.product and x.vid == 0x0403 and x.pid == 0x6001:
                        available_list.append(x.device)
            except TypeError:
                continue
        return available_list

    def connect(self, device: str = None) -> bool:
        if self.is_connected is True:
            self._log.warning('Already connected to SMD9000')
            return False
        device_connection_list = []
        if device is None:
            device_connection_list = self.get_available_sensors()
        else:
            device_connection_list.append(device)

        for d in device_connection_list:
            try:
                self._ser = serial.Serial(d, self._baud_rate, timeout=1)
            except serial.SerialException:
                self._log.debug('serial.SerialException for device on %s port' % d)
                if self._ser is not None:
                    self._ser.close()
                continue

            self._log.info('Found SMD9000 on port %s' % d)
            self.is_connected = True
            self._ser.flush()
            return True
        return False

    def _read_thread(self):
        """
        This is a thread that continuously reads from the SMD9000 sensor. Used when streaming is enabled
        """
        pass
      
    def get_adc_capture(self):
        """
        Gets a single ADC capture waveform for upstream and downstream measurments
        """
        self._ser_write('adccapture')
        ups = self._ser_read_until()
        dns = self._ser_read_until()
        
        ups = ups.split(',')
        dns = dns.split(',')

    def check_if_device_is_smd9000(self):
        """
        Checks if the connected device is actually an SMD9000 sensor

        Returns: True if the sensor is an SMD9000, False if not
        """
        self._ser_write('name')
        dev_name = self._ser_read_until()
        if 'SMD9000' in dev_name:
            return True
        return False

    def get_info(self) -> SMD9000Info:
        """
        Gets the hardware and firmware revision from the sensor
        """
        self._ser_write('getversion')
        dev_name = self._ser_read_until()             # Read the hardware and firmware revision
        dev_name = dev_name.split(',')                # Split the return string by comma
        dev_name = [d.split(":") for d in dev_name]   # Split the comma seperated string by colon
        dev_name = [d.strip() for d in dev_name]      # Remove whitespacing
        if dev_name[0] != "Hardware:":
            raise UserWarning()
        if dev_name[2] != "Firmware:":
            raise UserWarning()
        ret = SMD9000Info()
        ret.hardware_rev = dev_name[1]
        f = dev_name[3].split(".")
        try:
            ret.firmware_major_rev = int(f[0])
            ret.firmware_minor_rev = int(f[1])
        except ValueError:
            raise UserWarning("Invalid return: Unable to integerize the firmware revision")
        return ret

    def _ser_write(self, s: str) -> None:
        """
            Internal function to write to an unlocked serial device

            Args:
                s: What to write to the SMD9000
            Raises: :exception:`serial.SerialException` if there is an error writing to the port on pyserial's end
        """
        self._ser.write((s+'\n').encode('utf-8'))
        self._log.debug("Written to CQV: %s" % (s+'\n').encode('utf-8'))

    def _ser_read_until(self) -> str:
        """
            Internal function that reads data from the SMD9000

            Returns
                A string of the read data

            Raises:
                :exception:`serial.SerialException` if there is an error reading from the port on pyserial's end
        """
        with self._ser_lock:
            try:
                ret = self._ser.read_until()
                self._log.debug("Read from CQV: %s" % ret)
                ret = ret.decode('utf-8')
                ret = ret.rstrip()
                return ret
            except serial.SerialException:
                raise SMD9000ReadException()
