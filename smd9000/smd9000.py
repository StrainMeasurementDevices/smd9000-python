#!/usr/bin/env python3
"""
    SMD9000 Python Package
"""
import logging                                  # Import logging to log
import enum
import dataclasses
import os.path
import queue
import threading                                # Import to create a lock
import struct
import typing
import time
import serial                                   # Import pyserial to interface to the sensor
from crc import Calculator
import smd9000.smd9000_firmware as firmware_updater


class ReadException(Exception):
    """An exception that occurs when the sensor returns an unexpected response or did not respond at all"""
    def __init__(self, msg: str = None):
        super().__init__(msg)
        self.returned_error = msg


class InvalidProfile(Exception):
    """An exception that occurs if an invalid profile is selected to load or save"""


class NotInCalibrationMode(Exception):
    """An exception that occurs if any calibration command is called without being in calibration mode"""


@dataclasses.dataclass
class SensorInfo:
    """A dataclass for the sensor's hardware and firmware revision"""
    firmware_rev: str = None
    """The firmware revision of the sensor"""
    bootloader_rev: str = None
    """The firmware revision of the bootloader"""
    sensor_rev: str = None
    """The hardware revision of the sensor"""
    serial: str = None
    """The serial number of the sensor"""
    pn: str = None
    """The part number of the sensor"""

    def __repr__(self):
        s = f"An FlowDAQ sensor rev {self.sensor_rev}, firmware " \
            f"rev {self.firmware_rev}, bootloader rev {self.bootloader_rev}, " \
            f"with serial number {self.serial} and part number {self.pn}"
        return s


@dataclasses.dataclass
class StatusCode:
    """A dataclass for a returned status code, which includes the code and timestamp"""
    code: int
    time: int


@dataclasses.dataclass
class StatusWord:
    """
    A dataclass for the available status word/bits
    """
    def __init__(self, s_b: str):
        self.hardware_error = False
        self.status_code_in_stack = False
        self.no_signal = False
        self.low_signal = False
        self.high_signal = False
        self.tare = False
        self.cal_meas = False
        self.meas_halted = False
        self.over_range = False
        self.cal_mode_active = False
        self.low_voltage = False

        self.str = s_b
        if s_b.startswith('0b'):
            s_b = s_b[2:]
        self.raw = int(s_b, 2)             # The given integer value directly

        bin_to_dataclass_mapping = {
            0: 'hardware_error',
            1: 'status_code_in_stack',
            2: 'no_signal',
            3: 'low_signal',
            4: 'high_signal',
            5: 'tare',
            6: 'cal_meas',
            7: 'meas_halted',
            8: 'over_range',
            9: 'cal_mode_active',
            11: 'low_voltage',
        }
        for b in bin_to_dataclass_mapping:
            if (self.raw & (1 << b)) != 0:
                setattr(self, bin_to_dataclass_mapping[b], True)


@dataclasses.dataclass
class DataSet:
    """
    A dataclass for the sensor's returned data
    """
    flow: float = None
    """The flowrate"""
    amplitude: float = None
    """The amplitude in dB"""
    status: StatusWord = None
    """The status code returned from the sensor"""


class StreamFormat(enum.Enum):
    """Enum that corresponds to a datastream data type"""
    FLOWRATE = enum.auto()
    AMPLITUDE = enum.auto()
    STATUS_WORD = enum.auto()


@dataclasses.dataclass
class NameWithType:
    name: str
    type: type


available_parameters = {       # type: typing.Dict[str, type]
    'streamRate': int,
    'tareVal': float,
    'filter': str,
}

available_filter_time = ['0.1', '0.2', '0.5', '1', '2']
"""The available filter sizes, in seconds"""


def _boolean_to_str(v: bool):
    """Helper function to return a string on/off from a Python boolean"""
    return {True: 'on', False: 'off'}[v]


class SMD9000:
    """The main SMD9000 class"""
    _StreamFormatLookup = {
        StreamFormat.FLOWRATE: NameWithType('flow', float),
        StreamFormat.AMPLITUDE: NameWithType('amplitude', float),
        StreamFormat.STATUS_WORD: NameWithType('status', StatusWord)
    }
    default_timeout = 0.5
    """the default serial timeout"""

    def __init__(self, device: str = None, baud_rate: int = 115200, ser_async: bool = False):
        """
        Initialization of the SMD9000 class

        If the arguments `device` and `baud_rate` are given, they are saved for when the :func:`connect` function is
            called without arguments. They are also used for when this class is created with a context
            manager (with statement)
        """
        self._ser = None                        # Reference to a serial class object
        self._ser_lock = threading.Lock()       # Create a lock, in case the library is to be used with multiple threads
        self._baud_rate = 115200                # Baud Rate of the sensor
        self._log = logging.getLogger('smd9000')    # Get a logger with the name 'SMD9000'
        self._log_uart = logging.getLogger('smd9000.uart')    # Get a logger specific for UART communication

        self.SMD9000Data = DataSet

        self.datastream_format = None   # type: list[StreamFormat]
        """An ordered list of the datastream format"""
        self.is_data_streaming = False
        """Whether there is a current data stream on-going"""
        self.is_serial_async = ser_async
        """Whether the serial interface is async mode"""
        self.is_connected = False
        """Flag to determine if the sensor is connected"""

        self._read_thread = None        # type: threading.Thread
        self._exit_read_thread_def = False
        # self._read_thread_other_data = None
        # self._read_thread_other_data_event = threading.Event()

        self._async_read_data = queue.Queue()

        self.datastream_callback = None

        self._init_input_args = device, baud_rate

    def __del__(self):
        """In case the object is deleted, it should disconnect"""
        self.disconnect()
        del self

    def __enter__(self):
        if self._init_input_args[0] is None:
            raise UserWarning("Did not specify a COM port when calling the function")
        if not self.connect():
            raise UserWarning("Unable to connect to sensor in context manager")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self, device: str = None, baud_rate: int = 115200) -> bool:
        """
        Connects to a SMD9000 sensor.

        Args:
            device (str): The serial device name (`COM3` for Win32 or `/dev/ttyUSB0` for UNIX for example) to connect to.
                          Optional if you passed that as an argument to the init function
            baud_rate (int): The baud rate of the sensor. Defaults to 115200

        Returns:
            True or False depending on whether the device connected or now. True meaning it found a device and successfully connected to it

        Raises:
            UserWarning: If this class is ready connected to an SMD9000 sensor

        """
        if device is None:
            device, baud_rate = self._init_input_args
        if self.is_connected is True:
            self._log.warning('Already connected to an SMD9000 sensor')
            raise UserWarning('Already connected to an SMD9000 sensor')
        self.baud_rate = baud_rate
        try:
            self._ser = serial.Serial(device, self.baud_rate, timeout=self.default_timeout)
        except serial.SerialException:
            self._log.debug('serial.SerialException for device on %s port', device)
            if self._ser is not None:
                self._ser.close()
            raise
        # Send this command in case there was a previous ongoing data stream that didn't stop for some reason
        self.ser_write('\nset stream off')
        # Device does not acknowledge when the datastream is turned off, so add a small delay
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        try:
            if not self.check_if_device_is_smd9000():
                self._log.warning("Device that is being connected is not an SMD9000. Select the right device")
                self._ser.close()
                return False
            self._log.info('Found SMD9000 on port %s', device)
            self.datastream_format = self.get_stream_format()
            #self._ser.set_buffer_size(rx_size=6400, tx_size=6400)
            self._start_async_read()
        except Exception:
            self._log.exception("Error while connecting to sensor")
            self._ser.close()
            raise
        self.is_connected = True
        return True

    def close(self):
        """
        Alias of :func:`disconnect`
        """
        self.disconnect()

    def disconnect(self):
        """
        Disconnects from the sensor
        """
        self._log.debug('Disconnecting from sensor')
        if self.is_data_streaming:
            self.stop_data_stream()

        self._stop_async_read()

        if self._ser is not None:
            self._ser.close()
        self.is_connected = False

    def _read_thread_def(self):
        """
        This is a thread that continuously reads from the SMD9000 sensor
        """
        r = bytearray()
        while not self._exit_read_thread_def:
            time.sleep(0.01)
            while self._ser.in_waiting:
                try:
                    with self._ser_lock:
                        r += self._ser.read(self._ser.in_waiting)
                    r_split = r.split(b'\n')
                    if not r.endswith(b'\n'):
                        r = r_split[-1]
                        r_split.pop(-1)
                    else:
                        r_split = r_split[:-1]      # ommit the last item of the split, as it will be ''
                        r.clear()

                    for i in r_split:
                        self._log_uart.debug(f"Read Stream: {i}")

                    if self.is_data_streaming:
                        d = []
                        for i in r_split:
                            try:
                                i = i.decode('utf-8').strip()
                                d.append(self.interpret_flow_data(i))
                            except UserWarning:
                                self._async_read_data.put(i)
                                continue
                            if self.datastream_callback is not None:
                                self.datastream_callback(d)
                    else:
                        for i in r_split:
                            self._async_read_data.put(i)
                except Exception:
                    self._log.exception("Exception while reading stream data")
                    raise

    def _stop_async_read(self):
        """Stops the async read thread"""
        self._exit_read_thread_def = True
        if self._read_thread is not None:
            self._read_thread.join()
        self._read_thread = None

    def _start_async_read(self):
        """Starts the async read thread"""
        self._exit_read_thread_def = False
        if self._read_thread is None:
            self._read_thread = threading.Thread(target=self._read_thread_def, daemon=True)
        else:
            raise UserWarning("_read_thread is not None, which should never occur")
        self._read_thread.start()

    def start_data_stream(self, callback_def):
        """
        Function that starts a data stream readout

        Args:
            callback_def: A function that gets called everytime the data stream is updated.
                          This function must accept a single :class:`SMD9000Data` variable as it's input.
        """
        self.ser_write('set stream on')
        self.datastream_callback = callback_def
        self.is_data_streaming = True

    def stop_data_stream(self):
        """
        Function that stops a data stream readout, also joins the readout thread
        """
        self.ser_write('set stream off')
        self.is_data_streaming = False

    def ping(self) -> str:
        """
        Pings the sensor.

        Returns: "pong!" if the sensor responds correctly
        """
        self.ser_write('ping')
        r = self.ser_read_until()
        return r

    def read(self) -> DataSet:
        """
        Reads a single data point off the sensor

        Returns:
            A filled :class:`SMD9000Data` dataclass with the read data.
        """
        self.ser_write('read single')
        r = self.ser_read_until()
        d = self.interpret_flow_data(r)
        return d

    def measure_uncomp(self) -> float:
        """
        Function that measures an uncompensated flow for calibration purposes
        """
        self.ser_write('read acq')
        # Wait if we get a calibration start string
        self.wait_for_return("started meas")
        # And now...We wait
        ret = self.wait_for_return(timeout=20)
        if ret == "uncomp error":
            raise ReadException("Error was raised during calibration")
        if not ret.startswith("uncomp:"):
            raise ReadException("Did not receive correct token")
        ret = ret.split(":")[1]
        try:
            ret = float(ret)
        except TypeError:
            raise ReadException(f"Returned uncomp wasn't a float, instead was {ret}")
        return ret

    def set_stream_rate(self, stream_rate: int) -> None:
        """
        Sets the streaming rate for streaming mode

        Args:
            stream_rate (int): The stream rate in mS

        Raises:
            :exception:`SMD9000ReadException`: If the input stream rate is not withing a valid range
        """
        self.ser_write(f'set streamRate {stream_rate:d}')
        self.check_ack()

    def check_if_device_is_smd9000(self) -> bool:
        """
        Checks if the connected device is actually an SMD9000 sensor

        Returns:
            True if the sensor is an SMD9000, False if not
        """
        self.ser_write('read info pn')
        try:
            dev_name = self.ser_read_until()
        except ReadException as e:
            self._log.debug("Error when reading PN", exc_info=e)
            return False
        if 'SMD9000' in dev_name:
            return True
        return False

    def set_filter_size(self, size: str):
        """
        Sets the sensor's internal boxcar average filter size

        Args:
            size: The size of the boxcar average, only valid from 10 to 500 samples.

        Raises:
            UserWarning: If the size to be set isn't between 10 and 500 inclusive.
        """
        self.ser_write(f'set filter {size:s}')
        self.check_ack()
    
    def set_parameters(self, param: str, val: typing.Any) -> None:
        """
        Sends a `set x y` command to the sensor

        Args:
            param: The parameter to set
            val: The value to set it to

        Raises:
            :exception:`ReadException`: if did get an ack from the sensor back
        """
        if param in available_parameters:
            val = available_parameters[param](val)
        self.ser_write(f"set {param} {val}")
        self.check_ack()

    def get_parameters(self, param: str) -> typing.Union[str, int, float]:
        """
        Gets a parameter from the sensor
        Args:
            param: The parameter to get

        Returns:
            The parameter value from the sensor

        Raises:
            ValueError: if the parameter given is invalid (not in `SMD9000Parameters`)
            :exception:`ReadException`: if unable to convert the returned data to the parameter type
        """
        self.ser_write(f"get {param}")
        p = self.ser_read_until()
        if p == 'command error':
            raise ReadException(f'get {param} return a command error')
        if param in available_parameters:
            try:
                p = available_parameters[param](p)
            except ValueError as e:
                self._log.debug(f"Unable to convert {p} to {available_parameters[param]}")
                raise ReadException() from e
        return p

    def read_info(self) -> SensorInfo:
        """
        Gets the hardware and firmware revision from the sensor

        Returns:
            An :class:`SMD9000Info` dataclass containing the hardware revision, and the firmware major and minor revision

        Raises:
            UserWarning: If the returned firmware numbers are not able to be turned into an integer
            :exception:`ReadException`: If an error occured while reading the sensor's versions
        """
        ret = SensorInfo()
        self.ser_write('read info pn')
        ret.pn = self.ser_read_until()
        self.ser_write('read info firmwareRev')
        ret.firmware_rev = self.ser_read_until()
        self.ser_write('read info bootloaderRev')
        try:
            ret.bootloader_rev = self.ser_read_until()
        except UnicodeDecodeError:
            ret.bootloader_rev = '-'
        self.ser_write('read info serial')
        ret.serial = self.ser_read_until()
        self.ser_write('read info sensorRev')
        ret.sensor_rev = self.ser_read_until()
        return ret

    def read_status_bits(self) -> StatusWord:
        """Reads the status bits of the sensor"""
        self.ser_write('read statusBits')
        ret = self.ser_read_until()
        ret = StatusWord(ret)
        return ret

    def read_status_code_all(self) -> typing.List[StatusCode]:
        """
        Reads all status codes, using `statusCodeDump` read command

        Returns: A list of :class:`StatusCode`
        """
        all_codes = []
        self.ser_write('read statusCodeDump')
        ret = self.ser_read_until()
        assert "Error Stack:" in ret
        while 1:
            ret = self.ser_read_until()
            if "done" in ret:
                break
            all_codes.append(self.parse_status_code(ret))
        return all_codes

    def tare(self):
        """
        A macro/alternative for :func:`set_tare`
        """
        self.set_tare()

    def set_tare(self):
        """
        Sets taring based on the current flow
        """
        self.ser_write('tare')
        self.wait_for_return("started tare")
        ret = self.wait_for_return("done", timeout=20)

    def clear_tare(self):
        """
        Disables the sensor's tare if there was one
        """
        self.ser_write('tare off')
        self.check_ack()

    def set_stream_format(self, datastream_format: typing.List[StreamFormat]):
        """
        Sets the datastream format

        Args:
            datastream_format: The datastream format to set the sensor to as a list of :class:`SMD9000DatastreamFormat`
        """
        s = 'set streamFormat '
        for f in datastream_format:
            try:
                to_append = f'{self._StreamFormatLookup[f].name},'
            except KeyError as e:
                raise ValueError(f"Invalid data stream: {f}") from e
            s += to_append
        self.ser_write(s)
        self.check_ack()

    def get_stream_format(self) -> typing.List[StreamFormat]:
        """
        Reads and returns the sensor's current data stream format

        Returns: The current datastream format
        """
        ds_format = []
        self.ser_write('get streamFormat')
        data = self.ser_read_until()
        if 'Enabled Stream:' not in data:
            self._log.error("Returned value is invalid: No prefix!")
            raise ReadException()
        data = data.split(':')[1].strip()
        data = data.split(',')
        if data[-1] == '':
            data = data[:-1]
        for d in data:
            d = d.strip()
            to_append = None
            for d_lookup in self._StreamFormatLookup:
                if d == self._StreamFormatLookup[d_lookup].name:
                    to_append = d_lookup
                    break
            if to_append is None:
                self._log.error(f"Invalid data stream: {d}")
                raise ReadException()
            ds_format.append(to_append)
        # Also store the latest stream format internally
        self.datastream_format = ds_format
        return ds_format

    @property
    def baud_rate(self) -> int:
        """Property to get and set the baud rate of the sensor"""
        return self._baud_rate

    @baud_rate.setter
    def baud_rate(self, new_baud: int):
        """Property to set the baud rate of the sensor if connected"""
        self._baud_rate = new_baud
        if self.is_connected:
            self.ser_write(f"set uartBaud {new_baud:d}")
            self.check_ack()
            self._ser.flush()
            self._ser.reset_input_buffer()
            self._ser.baudrate = new_baud

    def save_profile(self, n: str):
        """
        Saves a profile

        Args:
            n: The profile name to save as, for example 'U1'
        """
        self.ser_write(f"save {n:s}")
        try:
            self.check_ack()
        except ReadException as e:
            if e.returned_error == 'parameter error':
                raise InvalidProfile() from e
            elif e.returned_error == 'not in cal mode':
                raise NotInCalibrationMode() from e
            elif e.returned_error == 'profile empty':
                raise InvalidProfile() from e
            else:
                raise

    def load_profile(self, n: str):
        """
        Loads a profile

        Args:
            n: The profile name to load, for example 'U1'
        """
        self.ser_write(f"load {n:s}")
        try:
            self.check_ack()
        except ReadException as e:
            if e.returned_error == 'parameter error':
                raise InvalidProfile() from e
            elif e.returned_error == 'not in cal mode':
                raise NotInCalibrationMode() from e
            elif e.returned_error == 'profile empty':
                raise InvalidProfile() from e
            else:
                raise

    def set_cal_mode(self, enable: bool = True):
        """Enables calibration mode"""
        self.ser_write(f"set calMode {_boolean_to_str(enable)}")
        self.check_ack()

    def update_firmware(self, fw_parsed: firmware_updater.FileInfo, status_callback: typing.Callable[[float], None] = None):
        """
        Command that handles uploading a firmware file given

        Args:
            fw_parsed(str): A read and parsed firmware update file
            status_callback: A callback function that gets called as a status update. Input to function needs to be a
                             float with an expected input of 0-100

        Raises:
            FileNotFoundError: When the input firmware path is not a file
            SMD9000InvalidFirmwareException: If the input file is not a valid one
        """
        self.set_cal_mode()
        old_profile = self.read_prf_conf()
        sensor_curr_version = self.read_info().firmware_rev

        if not firmware_updater.SMD9000SecureFWFileReader.check_fw_upgrade(sensor_curr_version, fw_parsed.fw_rev):
            raise firmware_updater.SMD9000InvalidFirmwareException("Firmware version to-from are incompatible")

        old_profile = firmware_updater.SMD9000SecureFWFileReader.update_fw_config(old_profile, sensor_curr_version, fw_parsed.fw_rev)

        self._stop_async_read()
        self._update_firmware_command()
        uploader = firmware_updater.SMD9000SecureFirmwareUploader(self._ser)
        # Intentionally update the baud rate of the serial interface, but not of this class
        self._ser.flush()
        self._ser.baudrate = 115200
        uploader.ping()             # check that we can talk to the device
        uploader.update_baud_rate(1000000)
        self._ser.flush()
        self._ser.baudrate = 1000000
        uploader.ping()  # check that we can talk to the device after baud change

        uploader.erase_memory()     # Erase the current firmware
        flash_sector_len = len(fw_parsed.flash_sectors)
        for f_i, f in enumerate(fw_parsed.flash_sectors):           # Upload all memory sections to fw
            if status_callback is not None:
                status_callback(f_i/flash_sector_len)
            uploader.upload_memory_section(f)
        uploader.check_memory_crc(fw_parsed.flash_crc)
        # Exit the bootloader and jump back into application
        uploader.boot_into_application()

        # Change the internal baud rate as a new firmware will have it be default
        self._baud_rate = 115200
        self._ser.baudrate = 115200
        self._start_async_read()
        time.sleep(0.2)
        self.set_cal_mode()
        self.write_prf_conf(old_profile)

    def read_prf_conf(self) -> typing.List[bytearray]:
        self._stop_async_read()
        self.ser_write("get prfConf")
        d = []
        while 1:
            r = self._read_binary()
            d.append(r[1])
            if r[0] == 0xCD:
                break
        self._start_async_read()
        return d

    def write_prf_conf(self, b_in: typing.List[bytearray]):
        self.ser_write("set prfConf 0")
        self.check_ack()
        s = self._write_binary(0xAB, b_in[0])
        self._ser.write(s)
        self.check_ack()
        s = self._write_binary(0xCD, b_in[1])
        self._ser.write(s)
        self.check_ack()

    def _update_firmware_command(self):
        """Sends a command to start the firmware uploading process"""
        self.ser_write("updateFirmware")
        self.check_ack()

    ################################################
    # Lower Level functions
    ################################################
    def parse_status_code(self, code_str: str) -> StatusCode:
        """
        Parses a status code returned by the sensor.
        For example, converts "0609,10" to a class:`StatusCode` type
        """
        code_str = code_str.split(',')
        code = int(code_str[0].strip(), 16)
        time = int(code_str[1].strip())
        return StatusCode(code, time)

    def ser_write(self, s: str) -> None:
        """
        Internal function to write to an unlocked serial device

        Args:
            s: What to write to the SMD9000

        Raises: :exc:`serial.SerialException` if there is an error writing to the port on pyserial's end
        """
        with self._ser_lock:
            with self._async_read_data.mutex:
                self._async_read_data.queue.clear()
            to_w = (s + '\n').encode('utf-8')
            self._log_uart.debug(f"Written to SMD9000: {to_w}")
            self._ser.write(to_w)

    def ser_read_until(self) -> str:
        """
        Internal function that reads data from the SMD9000

        Returns
            A string of the read data

        Raises:
            :exception:`serial.SerialException` if there is an error reading from the port on pyserial's end
        """
        if self._read_thread is not None:
            # if not self._read_thread_other_data_event.wait(self._ser.timeout):
            #     raise TimeoutError()
            ret = self._async_read_data.get(timeout=self._ser.timeout)
            self._async_read_data.task_done()
            # self._log_uart.debug("Read from SMD9000 while streaming: %s", ret)
        else:
            with self._ser_lock:
                try:
                    ret = self._ser.read_until()
                except serial.SerialException as e:
                    raise ReadException() from e
            self._log_uart.debug(f"Read from SMD9000: {ret}")
        if type(ret) is not str:
            ret = ret.decode('utf-8')
            ret = ret.rstrip()
        return ret

    def interpret_flow_data(self, read_line: str) -> DataSet:
        """
        Internal function that interprets a single read data line and returns a filled `SMD9000Data` dataclass
        Args:
            read_line: The read string from the sensor

        Returns: A :class:`SMD9000Data` class filled with the received data
        """
        r = read_line.split(',')[:-1]
        if len(r) != len(self.datastream_format):
            # TODO(Future): Have this exception somehow to the callback thread, or something
            self._log.warning("Received %s for flow data", read_line)
            raise UserWarning("Error in the received data")
        d = self.SMD9000Data()
        for i, r_d in enumerate(r):
            format_enum = self.datastream_format[i]
            format_spec = self._StreamFormatLookup[format_enum]
            if format_enum == StreamFormat.AMPLITUDE and r_d.endswith('dB'):
                r_d = r_d[:-2]
            try:
                r_d = format_spec.type(r_d)
            except ValueError:
                self._log.warning(f"Unable to convert {r_d} to {format_spec.type}")
                r_d = None
            setattr(d, format_spec.name, r_d)
        return d

    def check_ack(self) -> None:
        """
        Reads the next returned line from the sensor which is expected to be an acknowledgement
        """
        self.wait_for_return("ok")

    def wait_for_return(self, wait_for: str = "", timeout: int = default_timeout) -> str:
        """
        Reads the next returned line from the sensor for a certain expected string
        """
        # if self.is_data_streaming:
        #     if not self._read_thread_other_data_event.wait(timeout):
        #         raise ReadException("")
        #     if wait_for != "" and self._read_thread_other_data != wait_for:
        #         self._log.warning("Expected `%s`, instead got %s", wait_for, self._read_thread_other_data)
        #         raise ReadException(self._read_thread_other_data)
        #     return self._read_thread_other_data
        # else:
        #     try:
        #         if timeout != self.default_timeout:
        #             # Without this, settings can change while UART bridge is sending stuff leading to corruption
        #             self._ser.flush()
        #             self._ser.timeout = timeout
        #         r = self.ser_read_until()
        #         if r != "":
        #             if wait_for != "" and r != wait_for:
        #                 self._log.warning("Expected `%s`, instead got %s", wait_for, r)
        #                 raise ReadException(r)
        #             return r
        #     finally:
        #         if timeout != self.default_timeout:
        #             self._ser.timeout = self.default_timeout
        try:
            if timeout != self.default_timeout:
                # Without this, settings can change while UART bridge is sending stuff leading to corruption
                self._ser.flush()
                self._ser.timeout = timeout
            r = self.ser_read_until()
            if r != "":
                if wait_for != "" and r != wait_for:
                    self._log.warning("Expected `%s`, instead got %s", wait_for, r)
                    raise ReadException(r)
                return r
        finally:
            if timeout != self.default_timeout:
                self._ser.timeout = self.default_timeout

    def _read_binary(self) -> tuple:
        crc_calc = Calculator(firmware_updater.crc_config)

        header_ret = self._ser.read(4)
        # TODO: if 0xFE in len, this will crap out!
        if header_ret[0] != 0xFE:
            raise UserWarning(f"Not FE, {header_ret}")
        return_code = header_ret[1]
        ret_len = struct.unpack("H", header_ret[2:4])[0]
        data = bytearray()

        fe_flag = False
        while ret_len > 0:
            r = self._ser.read(1)
            data += r
            if r == b'\xFE':
                if not fe_flag:
                    fe_flag = True
                    continue
            fe_flag = False
            ret_len -= 1

        # if ret_len != 0:
        #     data = self._ser.read(ret_len)
        # data += self._ser.read(data.count(b'\xFE\xFE'))
        # if data.endswith(b'\xFE'):
        #     data += self._ser.read(1)

        crc_read = self._ser.read(2)
        crc_read += self._ser.read(crc_read.count(b'\xFE\xFE'))
        if crc_read.endswith(b'\xFE'):
            crc_read += self._ser.read(1)

        raw = bytearray(header_ret + data + crc_read)

        crc_read = crc_read.replace(b'\xFE\xFE', b'\xFE')

        crc_calc = crc_calc.checksum(header_ret + data)
        crc_read = struct.unpack("H", crc_read)[0]

        # if crc_calc != crc_read:
        #     raise UserWarning(f"CRC does not match, {crc_read} vs {crc_calc}")

        data = data.replace(b'\xFE\xFE', b'\xFE')

        return return_code, data, raw

    def _write_binary(self, command: int, data: bytearray = None) -> bytes:
        crc_calc = Calculator(firmware_updater.crc_config)

        if data is None:
            data = bytearray()
        if command >= 0xE0:
            raise ValueError("Invalid command")
        comm = bytes([0xFE, command]) + struct.pack('H', len(data))
        comm += firmware_updater.append_fe(data)
        crc_calc = crc_calc.checksum(comm)
        comm += firmware_updater.append_fe(struct.pack("H", crc_calc))
        return comm
