#!/usr/bin/env python3
"""
    SMD9000 Python Package
"""
import enum
import struct
import enum
import typing
import time
import serial                                   # Import pyserial to interface to the sensor
import serial.tools.list_ports as serial_tools  # Import the list_ports tools from pyserial to find the SMD9000 sensor.
import logging                                  # Import logging to log
import threading                                # Import to create a lock
import dataclasses


class SMD9000ReadException(Exception):
    pass


@dataclasses.dataclass
class SMD9000Revisions:
    firmware_rev: str
    """The firmware revision of the sensor"""
    hardware_rev: str
    """The hardware revision of the sensor"""


@dataclasses.dataclass
class SMD9000Data:
    flowrate: float
    """The flowrate"""
    accum: float
    """The accumulated flowrate"""
    tof: float
    """Time of Flight"""
    sig: int
    """The signal strength from 0 to 2047"""
    stat: int
    """The status code returned from the sensor"""


class SMD9000StatusCodeErrors(enum.Enum):
    NO_ERROR = enum.auto()
    """Nothing occurred, all good!"""
    UNEXPECTED_RESET = enum.auto()
    """The device at some point exponentially reset. This needs to be cleared to go away"""
    LOW_SIGNAL_STRENGTH = enum.auto()
    """Low signal strength from the ultrasonic transducers. This probably means there is an air in the tube."""
    INTERNAL_ALGORITHM_ERROR = enum.auto()
    """Internal sensor algorithm error"""
    GENERAL_ERROR = enum.auto()
    """A general blanket error from the sensor"""


class SMD9000:
    def __init__(self):
        """Initialization of the SMD9000 class"""
        self.is_connected = False               # Flag to determine if the sensor is connected
        self._ser = None                        # Reference to a serial class object
        self._ser_lock = threading.Lock()       # Create a lock, in case the library is to be used with multiple threads
        self._baud_rate = 115200                # Baud Rate of the sensor
        self._log = logging.getLogger('SMD9000')    # Get a logger with the name 'SMD9000'
        self._log_uart = logging.getLogger('SMD9000_UART')    # Get a logger specific for UART communication

        self._read_thread = None        # type: threading.Thread
        self._exit_read_thread_def = False

    def connect(self, device: str) -> bool:
        """
        Connects to a SMD9000 sensor.

        Args:
            device (str, Optional): An optional device name (`COM3` for Win32 or `/dev/ttyUSB0` for UNIX for example) to connect to. If omitted, this function will try to automatically detect one with the :func:`get_available_sensors` function.

        Returns:
            True or False depending on whether the device connected or now. True meaning it found a device and successfully connected to it

        Raises:
            UserWarning: If this class is ready connected to an SMD9000 sensor

        """
        if self.is_connected is True:
            self._log.warning('Already connected to an SMD9000 sensor')
            raise UserWarning('Already connected to an SMD9000 sensor')

        try:
            self._ser = serial.Serial(device, self._baud_rate, timeout=1)
        except serial.SerialException:
            self._log.debug('serial.SerialException for device on %s port' % device)
            if self._ser is not None:
                self._ser.close()
            return False
        self._ser.flush()
        if not self.check_if_device_is_smd9000():
            self._log.warning("Device that is beign connected is not an SMD9000. Select the right device")
            self._ser.close()
            return False
        self._log.info('Found SMD9000 on port %s' % device)
        self.is_connected = True

        return True

    def disconnect(self):
        """
        Disconnects from the sensor
        """
        self._ser.close()
        self.is_connected = False

    def _read_thread_def(self, callback_thread: typing.Union[None, typing.Callable[[SMD9000Data], None]]):
        """
        This is a thread that continuously reads from the SMD9000 sensor
        """
        while not self._exit_read_thread_def:
            if self._ser.in_waiting:
                r = self._ser_read_until()
                # TODO: Check for ok
                d = self._interpret_flow_data(r)
                if callback_thread is not None:
                    callback_thread(d)

    def start_data_stream(self, callback_def):
        """
        Function that start a data stream readout

        Args:
            callback_def: A function that gets called everytime the data stream is updated.
                          This function must accept a single :class`SMD9000Data` variable as it's input.
        """
        self._exit_read_thread_def = False
        if self._read_thread is None:
            self._read_thread = threading.Thread(target=self._read_thread_def, args=(callback_def, ))
        self._read_thread.start()
        self._ser_write('datastreamon')

    def stop_data_stream(self):
        """
        Function that stops a data stream readout
        """
        self._ser_write('datastreamoff')
        self._exit_read_thread_def = True
        self._read_thread.join()
        self._read_thread = None

    def read(self) -> SMD9000Data:
        """
        Reads a single data point off the sensor

        Returns:
            A filled :class:`SMD9000Data` dataclass with the read data.
        """
        self._ser_write('datasingle')
        r = self._ser_read_until()
        d = self._interpret_flow_data(r)
        return d

    def get_meter_constant(self) -> float:
        """
        Get the meter constant currently in use by the sensor

        Returns:
            The meter constant
        """
        self._ser_write('getmc')
        mc = self._ser_read_until()
        mc = mc.split(':')[1].strip()
        mc = self._ieee_float_to_python(mc)
        return mc

    def get_offset(self) -> float:
        """
        Get the offset currently in use by the sensor

        Returns:
            The offset
        """
        self._ser_write('getoffset')
        mc = self._ser_read_until()
        mc = mc.split(':')[1].strip()
        mc = self._ieee_float_to_python(mc)
        return mc

    def set_stream_rate(self, stream_rate: int) -> None:
        """
        Sets the streaming rate for streaming mode

        Args:
            stream_rate (int): The stream rate in Hz, between 1 and 100

        Raises:
            ValueError: If the input stream rate is not withing a valid range
        """
        if not 1 < stream_rate < 100:
            raise ValueError("Invalid streaming rate")
        self._ser_write('setstreamrate {:d}'.format(stream_rate))
        self._check_ack()

    def check_if_device_is_smd9000(self) -> bool:
        """
        Checks if the connected device is actually an SMD9000 sensor

        Returns:
            True if the sensor is an SMD9000, False if not
        """
        self._ser_write('name')
        dev_name = self._ser_read_until()
        if 'SMD9000' in dev_name:
            return True
        return False

    def set_filter_size(self, size: int):
        """
        Sets the sensor's internal boxcar average filter size

        Args:
            size: The size of the boxcar average, only valid from 10 to 500 samples.

        Raises:
            UserWarning: If the size to be set isn't between 10 and 500 inclusive.
        """
        if size < 10 and size > 500:
            raise UserWarning("Invalid filter size")
        self._ser_write('setfilt %d'.format(size))
        self._check_ack()

    def get_revisions(self) -> SMD9000Revisions:
        """
        Gets the hardware and firmware revision from the sensor

        Returns:
            An SMD9000Info dataclass containing the hardware revision, and the firmware major and minor revision

        Raises:
            UserWarning: If the returned firmware numbers are not able to be turned into an integer
            SMD9000ReadException: If an error occured while reading the sensor's versions
        """
        self._ser_write('getversion')
        dev_name = self._ser_read_until()                   # Read the hardware and firmware revision
        hardware_rev, firmware_rev = dev_name.split(',')    # Split the return string by comma
        hardware_rev = [s.strip() for s in hardware_rev.split(":")]
        firmware_rev = [s.strip() for s in firmware_rev.split(":")]
        if hardware_rev[0] != "Hardware":
            raise SMD9000ReadException()
        if firmware_rev[0] != "Firmware":
            raise SMD9000ReadException()
        ret = SMD9000Revisions(hardware_rev=hardware_rev[1], firmware_rev=firmware_rev[1])
        return ret

    def reset_accumulated(self):
        """
        Resets the accumulated flow to zero
        """
        self._ser_write('accumzero')
        self._check_ack()

    def calibration_for_zero(self):
        """
        Performs a zero calibration reading.
        The sensor takes a 10-second reading and records all internal values to be used during calibration with a cumulative average.

        .. note::
            This command does not actually perform the calibration, but only records values for it.
            To actually perform the calibration, call the :func:`set_calibration` function.
        .. note::
            This is a blocking function! This function will block until the sensor gathers data for a zero calibration, which is about 10 seconds.

        Raises:
            TimeoutError: If the sensor does not respond with a *"Cal Done"* after 20 seconds.
            SMD9000ReadException: If the sensor did not return the initial *"Cal Started"*
        """
        self._ser_write('calzero')
        r = self._ser_read_until()
        if r != "Cal Started":
            raise SMD9000ReadException()
        self._wait_for_calibration()

    def calibration_for_flow(self, expected_flow: float):
        """
        Performs a flow calibration reading with a current expected flowrate.
        The sensor takes a 10-second reading and records all internal values to be used during calibration with a cumulative average.

        .. note::
            This command does not actually perform the calibration, but only records values for it.
            To actually perform the calibration, call the :func:`set_calibration` function.
        .. note::
            This is a blocking function! This function will block until the sensor gathers data for a flow calibration, which is about 10 seconds.

        Raises:
            TimeoutError: If the sensor does not respond with a *"Cal Done"* after 20 seconds.
            SMD9000ReadException: If the sensor did not return the initial *"Cal Started"*
        """
        self._ser_write('calflow {:f}'.format(expected_flow))
        r = self._ser_read_until()
        if r != "Cal Started":
            raise SMD9000ReadException()
        self._wait_for_calibration()

    def _wait_for_calibration(self):
        """
        Internal function that waits for the sensor to be calibrated
        """
        start_time = time.time()
        while 1:
            if time.time() - start_time > 20:
                raise TimeoutError("Timeout waiting for a calibration data complete readout")
            r = self._ser_read_until()
            if r == "Cal Done":
                break

    def set_calibration(self):
        """
        Calibrates the sensor using the acquired data from the
        :func:`calibration_get_zero` and :func:`calibration_get_flow` functions
        """
        self._ser_write('setcal')
        self._check_ack()

    def set_tare(self):
        """
        Sets taring based on the current flow
        """
        self._ser_write('tare on')
        self._check_ack()

    def clear_tare(self):
        """
        Clears the sensor's tare if there was one
        """
        self._ser_write('tare off')
        self._check_ack()

    ################################################
    # Lower Level functions
    ################################################
    def _ser_write(self, s: str) -> None:
        """
        Internal function to write to an unlocked serial device

        Args:
            s: What to write to the SMD9000

        Raises: :exception:`serial.SerialException` if there is an error writing to the port on pyserial's end
        """
        with self._ser_lock:
            self._ser.write((s+'\n').encode('utf-8'))
            self._log_uart.debug("Written to SMD9000: %s" % (s+'\n').encode('utf-8'))

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
                self._log_uart.debug("Read from SMD9000: %s" % ret)
                ret = ret.decode('utf-8')
                ret = ret.rstrip()
                return ret
            except serial.SerialException:
                raise SMD9000ReadException()

    def _interpret_flow_data(self, r: str) -> SMD9000Data:
        """
        Internal function that interprets a single read data line and returns a filled `SMD9000Data` dataclass
        Args:
            r: The read string from the sensor

        Returns: A :class:`SMD9000Data` class filled with the received data
        """
        r = r.split(',')
        if len(r) != 5:
            # TODO: Have this exception somehow to the callback thread, or something
            self._log.error("Received {}".format(r))
            raise UserWarning("Error in the received data")
        d = SMD9000Data(flowrate=self._ieee_float_to_python(r[0]),
                        accum=self._ieee_float_to_python(r[1]),
                        tof=self._ieee_float_to_python(r[2]),
                        sig=int(r[3]),
                        stat=int(r[4]))
        return d

    def _check_ack(self) -> None:
        """
        Reads the next returned line from the sensor which is expected to be an acknowledgement
        """
        r = self._ser_read_until()
        if r != "ok":
            self._log.warning("Expected `ok`, instead got {}".format(r))
            raise SMD9000ReadException()

    @staticmethod
    def _ieee_float_to_python(ieee_float: str) -> float:
        """
        Takes a string in the form of an ASCII IEEE754 float, for example BE264F7B, and converts it to a floating
        point number that Python can use
        """
        return struct.unpack("f", struct.pack("I", int(ieee_float, 16)))[0]
