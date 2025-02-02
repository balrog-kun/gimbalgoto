import struct
from collections import namedtuple
from logging import getLogger

import serial

from simplebgc.command_ids import *
from simplebgc.commands import ControlOutCmd, RawCmd

logger = getLogger(__name__)

MessageHeader = namedtuple(
    'MessageHeader',
    'start_character command_id payload_size header_checksum')

MessagePayload = namedtuple(
    'MessagePayload',
    'payload checksum')

Message = namedtuple(
    'Message',
    'start_character command_id payload_size header_checksum payload checksum')

class DataShortException(Exception):
    pass

# The following factor is used to convert degrees to the units used by the
# SimpleBGC 2.6 serial protocol.
degree_factor = 0.02197265625
degree_per_sec_factor = 0.1220740379

v1_start_char = 0x3e
v2_start_char = 0x24

v1_message_format = '<BBBB{}sB'
v2_message_format = '<BBBB{}sH'

v1_payload_format = '<{}sB'
v2_payload_format = '<{}sH'

# Default to version 2
version = 2

def get_v2_crc(command_id: int, payload: bytes) -> int:
    crc = 0
    def add_byte(byte):
        nonlocal crc
        for _ in range(8):
            # The difference here from the usual implementation is that
            # input is processed from LSB to MSB so we get better results
            # by doing the whole calculation with crc bits inverted
            if crc >> 15 != byte & 1:
                crc = (crc << 1) ^ 0x8005
            else:
                crc <<= 1
            byte >>= 1
            crc &= 0xffff

    payload_size = len(payload)
    add_byte(command_id)
    add_byte(payload_size)
    add_byte(command_id + payload_size)
    for byte in payload:
        add_byte(byte)

    return crc

def validate_crc(message: Message) -> None:
    if (message.command_id + message.payload_size) & 255 != message.header_checksum:
        raise Exception('Wrong header checksum')

    if message.start_character == v1_start_char:
        checksum = sum(message.payload) & 255
    else:
        checksum = get_v2_crc(message.command_id, message.payload)

    if checksum != message.checksum:
        raise Exception('Wrong header checksum')

def create_message(command_id: int, payload: bytes = b'') -> Message:
    payload_size = len(payload)
    start_char = v1_start_char if version == 1 else v2_start_char
    checksum = sum(payload) % 255 if version == 1 else get_v2_crc(command_id, payload)

    return Message(start_character=start_char,
                   command_id=command_id,
                   payload_size=payload_size,
                   header_checksum=(command_id + payload_size) & 255,
                   payload=payload,
                   checksum=checksum)


def pack_message(message: Message) -> bytes:
    message_format = (v1_message_format if version == 1 else v2_message_format).format(message.payload_size)
    return struct.pack(message_format, *message)


def unpack_message(data: bytes) -> (Message, int):
    if len(data) < 5: # Header + v1 CRC
        raise DataShortException()

    header = MessageHeader._make(struct.unpack('<BBBB', data[:4]))
    if header.start_character not in [v1_start_char, v2_start_char]:
        raise Exception('Wrong header start char')

    if (header.command_id + header.payload_size) & 255 != header.header_checksum:
        raise Exception('Wrong header checksum')

    crc_size = 2 if header.start_character == v2_start_char else 1
    cmd_size = 4 + header.payload_size + crc_size

    if len(data) < cmd_size:
        raise DataShortException()

    message_format = (v1_message_format if version == 1 else v2_message_format).format(header.payload_size)
    message = Message._make(struct.unpack(message_format, data[:cmd_size]))
    validate_crc(message)
    return message, cmd_size


def read_message(connection: serial.Serial, payload_size: int) -> Message:
    # 5 is the length of the header + v1 checksum size
    response_data = connection.read(5 + payload_size)
    if response_data[0] == v2_start_char:
        response_data += connection.read(1)
    # print('received response', response_data)
    message, consumed = unpack_message(response_data)
    assert consumed == len(response_data)
    return message


def read_message_header(connection: serial.Serial) -> MessageHeader:
    header_data = connection.read(4)
    logger.debug(f'received message header data: {header_data}')
    return MessageHeader._make(struct.unpack('<BBBB', header_data))


def read_message_payload(connection: serial.Serial,
                         header: MessageHeader) -> MessagePayload:
    crc_size = 2 if header.start_character == v2_start_char else 1
    payload_data = connection.read(header.payload_size + crc_size)
    logger.debug(f'received message payload data: {payload_data}')
    payload_format_format = v2_payload_format if header.start_character == v2_start_char else v1_payload_format
    payload_format = payload_format_format.format(header.payload_size)
    return MessagePayload._make(struct.unpack(payload_format, payload_data))


def read_cmd(connection: serial.Serial) -> RawCmd:
    header = read_message_header(connection)
    logger.debug(f'parsed message header: {header}')
    assert header.start_character in [v1_start_char, v2_start_char]
    checksum = (header.command_id + header.payload_size) % 256
    assert checksum == header.header_checksum
    payload = read_message_payload(connection, header.payload_size)
    if header.start_character == v1_start_char:
        checksum = sum(payload.payload) & 255
    else:
        checksum = get_v2_crc(header.command_id, payload.payload)
    assert checksum == payload.checksum
    logger.debug(f'parsed message payload: {payload}')
    return RawCmd(header.command_id, payload.payload)


def control_gimbal(
        yaw_mode: int = 1,
        yaw_speed: int = 0,
        yaw_angle: int = 0,
        pitch_mode: int = 1,
        pitch_speed: int = 0,
        pitch_angle: int = 0) -> None:
    logger.debug(' '.join((
        f'control_gimbal:',
        f'yaw_mode={yaw_mode}',
        f'yaw_speed={yaw_speed}',
        f'yaw_angle={yaw_angle}',
        f'pitch_mode={pitch_mode}',
        f'pitch_speed={pitch_speed}',
        f'pitch_angle={pitch_angle}')))
    yaw_angle = int(yaw_angle / degree_factor)
    pitch_angle = int(pitch_angle / degree_factor)
    control_data = ControlOutCmd(
        roll_mode=2, roll_speed=0, roll_angle=0,
        pitch_mode=pitch_mode, pitch_speed=pitch_speed, pitch_angle=pitch_angle,
        yaw_mode=yaw_mode, yaw_speed=yaw_speed, yaw_angle=yaw_angle)
    message = create_message(CMD_CONTROL, control_data.pack())
    packed_message = pack_message(message)
    connection = serial.Serial('/dev/rfcomm0', baudrate=115200, timeout=10)
    connection.write(packed_message)
    message = read_message(connection, 1)


if __name__ == '__main__':
    from time import sleep

    # rotate_gimbal(yaw_speed=-100)
    # rotate_gimbal(yaw_speed=100)
    # sleep(3)
    # rotate_gimbal(yaw_speed=0)
    control_gimbal(
        pitch_mode=2, pitch_speed=300, pitch_angle=0,
        yaw_mode=2, yaw_speed=300, yaw_angle=0)
    sleep(3)
    control_gimbal(
        pitch_mode=2, pitch_speed=300, pitch_angle=-60,
        yaw_mode=2, yaw_speed=300, yaw_angle=360)
    sleep(3)
    control_gimbal(
        pitch_mode=2, pitch_speed=300, pitch_angle=0,
        yaw_mode=2, yaw_speed=300, yaw_angle=0)

# 1u – 1 byte unsigned
# 1s – 1 byte signed
# 2u – 2 byte unsigned (little-endian order)
# 2s – 2 byte signed (little-endian order)
# 4f – float (IEEE-754 standard)
# 4s – 4 bytes signed (little-endian order)
# string – ASCII character array, first byte is array size
# Nb – byte array size N

# CMD_BOARD_INFO
# >   id  size  check  payload  check
# 62  86    1    87       1       1
# 62  86    0    86              0
