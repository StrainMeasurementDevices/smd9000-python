#!/usr/bin/env python3
"""
    SMD9000 Python Package
"""
import logging                                  # Import logging to log
import enum
import dataclasses
import threading                                # Import to create a lock
import struct
import typing
import time
import serial                                   # Import pyserial to interface to the sensor


class SMD9000ReadException(Exception):
    """An exception that occurs when the sensor returns an unexpected response or did not respond at all"""


@dataclasses.dataclass
class SMD9000Revisions:
    """A dataclass for the sensor's hardware and firmware revision"""
    firmware_rev: str
    """The firmware revision of the sensor"""
    hardware_rev: str
    """The hardware revision of the sensor"""


@dataclasses.dataclass
class SMD9000Data:
    """A dataclass for the sensor's returned data"""
    flowrate: float = None
    """The flowrate"""
    up_tof: float = None
    """The upstream Time of Flight"""
    dn_tof: float = None
    """The downstream Time of Flight"""
    sig: int = None
    """The signal strength from 0 to 2047"""
    stat: int = None
    """The status code returned from the sensor"""


class SMD9000StatusCodeErrors(enum.Enum):
    """Enum for all possible error codes returned from the sensor"""
    NO_ERROR = enum.auto
    """Nothing occurred, all good!"""
    UNEXPECTED_RESET = enum.auto
    """The device at some point exponentially reset. This needs to be cleared to go away"""
    LOW_SIGNAL_STRENGTH = enum.auto
    """Low signal strength from the ultrasonic transducers. This probably means there is an air in the tube."""
    INTERNAL_ALGORITHM_ERROR = enum.auto
    """Internal sensor algorithm error"""
    GENERAL_ERROR = enum.auto
    """A general blanket error from the sensor"""


class SMD9000DatastreamFormat(enum.Enum):
    """Enum that corresponds to a datastream data type"""
    SMD9000_DATA_FLOWRATE = 1
    SMD9000_DATA_UPSTREAM_TOF = 6
    SMD9000_DATA_DOWNSTREAM_TOF = 7
    SMD9000_DATA_SIG_STRENGTH = 4
    SMD9000_DATA_STATUS_CODE = 5


class SMD9000:
    """The main SMD9000 class"""
    def __init__(self):
        """Initialization of the SMD9000 class"""
        self._ser = None                        # Reference to a serial class object
        self._ser_lock = threading.Lock()       # Create a lock, in case the library is to be used with multiple threads
        self._baud_rate = 115200                # Baud Rate of the sensor
        self._log = logging.getLogger('SMD9000')    # Get a logger with the name 'SMD9000'
        self._log_uart = logging.getLogger('SMD9000_UART')    # Get a logger specific for UART communication

        self.datastream_format = None   # type: list[SMD9000DatastreamFormat]
        """An ordered list of the datastream format"""
        self.is_data_streaming = False
        """Whether there is a current data stream on-going"""
        self.is_connected = False
        """Flag to determine if the sensor is connected"""

        self._read_thread = None        # type: threading.Thread
        self._exit_read_thread_def = False
        self._read_thread_other_data = None
        self._read_thread_other_data_event = threading.Event()

    def connect(self, device: str) -> bool:
        """
        Connects to a SMD9000 sensor.

        Args:
            device (str): An optional device name (`COM3` for Win32 or `/dev/ttyUSB0` for UNIX for example) to connect to.

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
            self._log.debug('serial.SerialException for device on %s port', device)
            if self._ser is not None:
                self._ser.close()
            return False
        # Send this command in case there was a previous ongoing data stream that didn't stop for some reason
        self._ser_write('datastreamoff')
        self._ser.flush()
        if not self.check_if_device_is_smd9000():
            self._log.warning("Device that is being connected is not an SMD9000. Select the right device")
            self._ser.close()
            return False
        self._log.info('Found SMD9000 on port %s', device)
        self.is_connected = True
        self.datastream_format = self.get_datastream_format()
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
                try:
                    d = self._interpret_flow_data(r)
                except UserWarning:
                    self._read_thread_other_data = r
                    self._read_thread_other_data_event.set()
                    continue
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
        self.is_data_streaming = True

    def stop_data_stream(self):
        """
        Function that stops a data stream readout
        """
        self._ser_write('datastreamoff')
        self._exit_read_thread_def = True
        self._read_thread.join()
        self._read_thread = None
        self.is_data_streaming = False

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
        mc = float(mc)
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
        mc = float(mc)
        return mc

    def set_stream_rate(self, stream_rate: int) -> None:
        """
        Sets the streaming rate for streaming mode

        Args:
            stream_rate (int): The stream rate in Hz, between 1 and 100

        Raises:
            ValueError: If the input stream rate is not withing a valid range
        """
        if not 1 <= stream_rate <= 100:
            raise ValueError("Invalid streaming rate")
        self._ser_write(f'setstreamrate {stream_rate:d}')
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
        if 10 > size or size > 500:
            raise UserWarning("Invalid filter size")
        self._ser_write(f'setfilt {size:d}')
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

    def calibration_for_flow(self):
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
        self._ser_write('calflow')
        if self.is_data_streaming:
            if not self._read_thread_other_data_event.wait(2):
                raise SMD9000ReadException()
            if self._read_thread_other_data != "Cal Started":
                raise SMD9000ReadException()
        else:
            r = self._ser_read_until()
            if r != "Cal Started":
                raise SMD9000ReadException()
        self._wait_for_calibration()

    def calibration_set_real_flow(self, real_flow: float):
        """
        The real flowrate while the sensor was calibrating with the :func`calibration_for_flow` command

        Args:
            real_flow: The real flowrate to calibrate to

        Raises:
            SMD9000ReadException: If the sensor did not acknowledge the command
        """
        self._ser_write(f'calrealflow {real_flow:.2f}')
        self._check_ack()

    def _wait_for_calibration(self):
        """
        Internal function that waits for the sensor to be calibrated
        """
        start_time = time.time()
        if self.is_data_streaming:
            if not self._read_thread_other_data_event.wait(20):
                raise SMD9000ReadException()
            if self._read_thread_other_data != "Cal Done":
                self._log.warning("Expected `Cal Done`, instead got %s", self._read_thread_other_data)
                raise SMD9000ReadException()
        else:
            while 1:
                if time.time() - start_time > 20:
                    raise TimeoutError("Timeout waiting for a calibration data complete readout")
                r = self._ser_read_until()
                if r == "Cal Done":
                    break
                if r == "Cal Error":
                    raise SMD9000ReadException()

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

    def clear_status_code(self):
        """
        Clears the sensor's current status code
        """
        self._ser_write('clearstatus')
        self._check_ack()

    def auto_gain(self):
        """
        Automatically sets the right gain for the sensor for the right signal strength
        """
        self._ser_write('autogain')
        self._check_ack()

    def set_datastream_format(self, datastream_format: typing.List[SMD9000DatastreamFormat]):
        """
        Sets the datastream format

        Args:
            datastream_format: The datastream format to set the sensor to
        """
        s = 'set_datastream_data'
        for f in datastream_format:
            s += f' {f.value:d}'
        self._ser_write(s)
        self._check_ack()
        self.datastream_format = datastream_format

    def get_datastream_format(self) -> typing.List[SMD9000DatastreamFormat]:
        """
        Reads and returns the sensor's current data stream format

        Returns: The current datastream format
        """
        ds_format = []
        self._ser_write('get_datastream_format')
        data = self._ser_read_until()
        if 'UART Data Stream Format:' not in data:
            self._log.error("Returned value is invalid: No prefix!")
            raise SMD9000ReadException()
        data = data.split(':')[1].strip()
        for d in data:
            try:
                d = int(d)
            except ValueError as e:
                self._log.error("Returned value from get_datastream_format command is invalid")
                raise SMD9000ReadException() from e
            ds_format.append(SMD9000DatastreamFormat(d))
        return ds_format

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
            self._log_uart.debug("Written to SMD9000: %s", (s+'\n').encode('utf-8'))

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
                self._log_uart.debug("Read from SMD9000: %s", ret)
                ret = ret.decode('utf-8')
                ret = ret.rstrip()
                return ret
            except serial.SerialException as e:
                raise SMD9000ReadException() from e

    def _interpret_flow_data(self, r: str) -> SMD9000Data:
        """
        Internal function that interprets a single read data line and returns a filled `SMD9000Data` dataclass
        Args:
            r: The read string from the sensor

        Returns: A :class:`SMD9000Data` class filled with the received data
        """
        r = r.split(',')
        if len(r) != len(self.datastream_format):
            # TODO(Future): Have this exception somehow to the callback thread, or something
            self._log.error("Received %s", r)
            raise UserWarning("Error in the received data")
        d = SMD9000Data()
        for i, r_d in enumerate(r):
            if self.datastream_format[i] == SMD9000DatastreamFormat.SMD9000_DATA_FLOWRATE:
                d.flowrate = float(r_d)
            elif self.datastream_format[i] == SMD9000DatastreamFormat.SMD9000_DATA_UPSTREAM_TOF:
                d.up_tof = float(r_d)
            elif self.datastream_format[i] == SMD9000DatastreamFormat.SMD9000_DATA_DOWNSTREAM_TOF:
                d.dn_tof = float(r_d)
            elif self.datastream_format[i] == SMD9000DatastreamFormat.SMD9000_DATA_SIG_STRENGTH:
                d.sig = int(r_d)
            elif self.datastream_format[i] == SMD9000DatastreamFormat.SMD9000_DATA_STATUS_CODE:
                d.stat = int(r_d)
        return d

    def _check_ack(self) -> None:
        """
        Reads the next returned line from the sensor which is expected to be an acknowledgement
        """
        if self.is_data_streaming:
            if not self._read_thread_other_data_event.wait(2):
                raise SMD9000ReadException()
            if self._read_thread_other_data != "ok":
                self._log.warning("Expected `ok`, instead got %s", self._read_thread_other_data)
                raise SMD9000ReadException()
        else:
            r = self._ser_read_until()
            if r != "ok":
                self._log.warning("Expected `ok`, instead got %s", r)
                raise SMD9000ReadException()

    @staticmethod
    def _ieee_float_to_python(ieee_float: str) -> float:
        """
        Takes a string in the form of an ASCII IEEE754 float, for example BE264F7B, and converts it to a floating
        point number that Python can use
        """
        return struct.unpack("f", struct.pack("I", int(ieee_float, 16)))[0]
