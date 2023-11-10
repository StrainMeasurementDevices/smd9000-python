"""
This file contains the main class that communicates with the SMD9000 sensor in order to upload firmware to it
"""
import io
from crc import Calculator, Configuration, Register
import struct
import logging
import copy
import dataclasses
import typing
import enum

crc_config = Configuration(
    width=16,
    polynomial=0x1021,
    init_value=0xFFFF,
    final_xor_value=0x0000,
    reverse_input=True,
    reverse_output=False,
)


class SMD9000FirmwareAckException(Exception):
    """Gets raised when a command does return an acknowledgment"""
    pass


class SMD9000InvalidFirmwareException(Exception):
    """Gets raised when the input firmware file is not a valid one"""
    pass


class SMD9000FirmwareGenericException(Exception):
    """Any generic exception catching"""
    pass


class SMD9000SecureFWCommand(enum.IntEnum):
    PING                        = 0x01
    BOOTLOADER_REV              = 0x02
    ERASE_MEMORY                = 0x10
    WRITE_MEMORY_DATA           = 0x11
    COMMAND_RESET_INTO_MAIN     = 0x20
    COMMAND_CHECK_MEMORY_CRC    = 0x30
    UPDATE_BAUD_RATE            = 0x40


@dataclasses.dataclass
class FileInfo:
    """
    This is what is returned by SMD9000SecureFWFileReader.read(), which includes all the information
    needed to upload a new firmware
    """
    file_rev: bytes = None          # The file revision
    compatible_bl: bytes = None     # The compatible bootloader major version
    fw_rev: bytes = None            # The revision of the firmware to be uploaded
    flash_sectors: typing.List[bytearray] = dataclasses.field(default_factory=list)     # The flash sectors
    flash_crc: int = None           # CRC for memory


class SMD9000SecureFWFileReader:
    """
    Class that handles reading an SMD firmware file and separating it out
    """
    def __init__(self) -> None:
        self.log = logging.getLogger('SMD9000SecureFWFileReader')
        self.crc_calc = Register(crc_config)

    def read(self, file_path) -> FileInfo:
        self.crc_calc.init()
        fw_info = FileInfo()
        with open(file_path, 'rb') as f:
            t = self._read_crc(f, len("SMD9000Firmware"))
            if t != "SMD9000Firmware".encode('utf-8'):
                raise SMD9000InvalidFirmwareException()
            fw_info.file_rev = self._read_crc(f, 1)
            # todo: check file rev
            fw_info.compatible_bl = self._read_crc(f, 1)
            fw_info.fw_rev = self._read_crc(f, 4)
            n_flash = self._read_crc(f, 2)
            n_flash = struct.unpack("H", n_flash)[0]
            for i in range(n_flash):
                fw_info.flash_sectors.append(bytearray(self._read_crc(f, 144)))
            _ = self._read_crc(f, 2)
            fw_info.flash_crc = struct.unpack("H", self._read_crc(f, 2))[0]
            duck = self._read_crc(f, len('duck!'))
            if duck != "duck!".encode('utf-8'):
                raise SMD9000InvalidFirmwareException()
            crc_f = f.read(2)

            # check if file is empty (it should)
            if len(f.read(1)) != 0:
                raise SMD9000InvalidFirmwareException()

            crc_f = struct.unpack("H", crc_f)[0]
            crc_e = self.crc_calc.digest()
            if crc_f != crc_e:
                print(crc_f, crc_e)
                raise SMD9000InvalidFirmwareException()
        return fw_info

    def _read_crc(self, f, n) -> bytes:
        """
        Reds into the file, while adding the read bytes into the crc
        Args:
            f: The file object to read from
            n: The number of bytes

        Returns: The bytes read from the file
        """
        b = f.read(n)
        self.crc_calc.update(b)
        return b


class SMD9000SecureFirmwareUploader:
    """
    The main class that handles uploading to the firmware
    """
    def __init__(self, ser: io.RawIOBase) -> None:
        self._ser = ser
        self.log = logging.getLogger('smd9000Firmware')
        self.crc_calc = Calculator(crc_config)

    def ping(self):
        ret = self.query(SMD9000SecureFWCommand.PING)
        if ret != 'pong!':
            raise UserWarning("Did not return Pong!")

    def get_bootloader_rev(self):
        ret = self.query(SMD9000SecureFWCommand.BOOTLOADER_REV)
        return ret

    def upload_memory_section(self, mem_section: bytearray):
        if len(mem_section) != 144:
            raise UserWarning("Invalid memory section")
        self.command(SMD9000SecureFWCommand.WRITE_MEMORY_DATA, mem_section)

    def erase_memory(self):
        self.command(SMD9000SecureFWCommand.ERASE_MEMORY)

    def boot_into_application(self):
        self.command(SMD9000SecureFWCommand.COMMAND_RESET_INTO_MAIN)

    def check_memory_crc(self, crc: int):
        crc = struct.pack("H", crc)
        self.command(SMD9000SecureFWCommand.COMMAND_CHECK_MEMORY_CRC, crc)

    def update_baud_rate(self, new_baud: int):
        new_baud = {115200: b'\x01', 1000000: b'\x02'}[new_baud]
        self.command(SMD9000SecureFWCommand.UPDATE_BAUD_RATE, new_baud)

    def command(self, command: SMD9000SecureFWCommand, data: bytes = None):
        r = self.query(command, data)
        if r != 'ok':
            raise UserWarning(f"Did not get ACK back, instead got {r}")

    def query(self, command: SMD9000SecureFWCommand, data: bytes = None):
        p = self._craft_packet(command, data)
        self._ser.write(p)
        r = self.get_response()
        return r

    def get_response(self) -> str:
        ret = self._ser.readline()
        crc_read = self._ser.read(2)

        crc_read = struct.unpack("H", crc_read)[0]
        crc_calc = self.crc_calc.checksum(ret)

        if crc_calc != crc_read:
            raise UserWarning("CRC does not match")

        ret = ret.decode().strip()
        return ret

    def _craft_packet(self, command: SMD9000SecureFWCommand, data: bytearray = None) -> bytes:
        if data is None:
            data = bytearray()
        if len(data) > 250:
            self.log.warning("Lenght is greater than 250, not accepting")
            raise SMD9000FirmwareGenericException()
        if command >= 0xE0:
            self.log.warning("Invalid command")
            raise SMD9000FirmwareGenericException()
        comm = bytes([0xFE, command, len(data)])
        comm += self._append_fe(data)
        crc_calc = self.crc_calc.checksum(comm)
        comm += self._append_fe(struct.pack("H", crc_calc))
        return comm

    @staticmethod
    def _append_fe(inp: bytes):
        output = copy.copy(inp)
        i = output.find(b'\xFE')
        while i != -1:
            output = output[:i] + b'\xFE' + output[i:]
            i = output.find(b'\xFE', i+2)
        return output
