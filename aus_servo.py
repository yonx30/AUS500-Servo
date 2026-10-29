"""25KG/35KG AUS RS485 servo protocol (v1.0). Requires pyserial."""
import struct
import threading
import warnings
import serial
import numpy as np


# Variables - Only configure if technical/programming datasheet specifies otherwise
REQUEST_HEADER = b'\x12\x4c'
RESPONSE_HEADER = b'\x05\x1c'
BAUDRATE = 115200
ANGLE_LIMITS = [-135, 135]


class ServoError(Exception):
    """Base class for servo communication errors."""

class ServoTimeout(ServoError):
    """No complete response received before timeout."""

class ServoProtocolError(ServoError):
    """Malformed or unexpected servo response."""


class AUSServoBus:
    """Serial RS485 bus; one request at a time, suitable for multiple servo IDs.

    Pass an existing serial-like object via `transport` for testing or for
    hardware-specific RS485 direction control. The object must implement
    write(), read(), and optionally reset_input_buffer().
    """

    def __init__(self, port:str=None, timeout:float=0.2, serial_bus=None):
        if timeout <= 0:
            raise ValueError('timeout must be positive')
        if serial_bus is None:
            if not port:
                raise ValueError('port is required without serial_bus')
            serial_bus = serial.Serial(port=port, baudrate=BAUDRATE,
                                      bytesize=serial.EIGHTBITS,
                                      parity=serial.PARITY_NONE,
                                      stopbits=serial.STOPBITS_ONE,
                                      timeout=timeout)
            self._owns_serial = True
        else:
            self._owns_serial = False
        self.serial = serial_bus
        self._lock = threading.RLock()


    def close_serial(self):
        '''Closes serial connection if this object owns it'''
        if self._owns_serial: self.serial.close()


    def _read_exact(self, size: int) -> bytes:
        '''Helper function for _read_response'''
        chunks = bytearray()
        while len(chunks) < size:
            chunk = self.serial.read(size - len(chunks))
            if not chunk:
                raise ServoTimeout(f'timed out reading response ({len(chunks)}/{size} bytes)')
            chunks.extend(chunk)
        return bytes(chunks)


    def _read_response(self, expected_command: int, expected_payload_size: int) -> bytes:
        '''Find response header; tolerate a few stray/echoed bytes.'''
        matched = 0
        with self._lock:
            for _ in range(512):
                b = self._read_exact(1)[0]
                if b == RESPONSE_HEADER[matched]:
                    matched += 1
                    if matched == 2:
                        break
                else:
                    matched = 1 if b == RESPONSE_HEADER[0] else 0
            else:
                # raise ServoProtocolError('response header not found')
                warnings.warn("Warning: No response received; check if servo is active")
                return None
            cmd, length = self._read_exact(2)
            payload = self._read_exact(length)
            checksum = self._read_exact(1)[0]

        if (sum(RESPONSE_HEADER + bytes((cmd, length)) + payload) & 0xFF) != checksum:
            raise ServoProtocolError('response checksum mismatch')
        if cmd != expected_command:
            raise ServoProtocolError(f'expected command 0x{expected_command:02X}, got 0x{cmd:02X}')
        if length != expected_payload_size:
            raise ServoProtocolError(f'expected {expected_payload_size} payload bytes, got {length}')

        return payload

    def _servo_id(self, value: int) -> int:
        '''Checks if servo id is a valid integer'''
        if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 254:
            raise ValueError('Servo ID must be an integer from 0 to 254')
        return value


    def _send_request(self, command:int, payload:bytes) -> None:
        """Encode an outgoing request, including the modulo-256 checksum."""
        with self._lock:
            body = REQUEST_HEADER + bytes((command, len(payload))) + payload
            packet = body + struct.pack('<B', sum(body)%256) #bytes(sum(body)%256) #f"{sum(body)%256:02x}"\
            self.serial.reset_input_buffer()
            self.serial.write(packet)
            self.serial.flush() # Blocks request until output is fully sent


    def set_angle(self, servo_id:int, angle:float, duration_ms:int=100) -> None:
        """Request motion for specified servo_id to specified target angle in specified duration.
        Clips angle to range (default: [-135, 135] for AUS500 25kg & 35kg variants, change to [-180, 180] for 45kg version)
        """
        servo_id = self._servo_id(servo_id)
        if not ANGLE_LIMITS[0] <= angle <= ANGLE_LIMITS[1]:
            original_angle = angle
            angle = np.clip(angle, ANGLE_LIMITS[0], ANGLE_LIMITS[1])
            warnings.warn(f'Command angle {original_angle}\N{DEGREE SIGN} exceeds limits {ANGLE_LIMITS}, clipping to {angle}', UserWarning)
        if not isinstance(duration_ms, (int, float)):
            raise ValueError(f'duration_ms {duration_ms} must be an int or float') 
        elif not 0 <= duration_ms <= 65535:
            raise ValueError(f'Duration {duration_ms}ms exceeds limits [0, 65535]')

        # Power function disabled, manual says to keep at 0 soz
        # if not isinstance(power, int) or not 0 <= power <= 65535: 
        #     raise ValueError('power must be within 0..65535')

        payload = struct.pack('<BhHH', servo_id, int(angle * 10), duration_ms, 0) # Power hardcoded to 0
        self._send_request(0x08, payload)

        # Tested servo does not send a response after set angle so disabled
        # response = self._read_response(0x08, servo_id, 1)
        # return struct.unpack('<B', response)


    def read_angle(self, servo_id: int) -> float:
        """Read current angle in degrees (signed 16-bit tenths)"""
        servo_id = self._servo_id(servo_id)
        payload = struct.pack('<B', servo_id)
        self._send_request(0x0A, payload)

        response = self._read_response(0x0A, 3)
        response = struct.unpack('<Bh', response[:3])
        response_servo_id, angle = response[0], response[1] / 10.0
        if response is None:
            raise ServoProtocolError('No response received')
        elif response_servo_id != servo_id:
            raise ServoProtocolError('Response servo ID mismatch')
        return angle
    

    def set_id(self, current_id: int, new_id: int) -> bool:
        """Change ID. Connect only one servo with current_id to avoid collisions.
        Returns True is ID change successful, else False"""
        current_id = self._servo_id(current_id)
        new_id = self._servo_id(new_id)
        payload = struct.pack('<BBB', current_id, 0x22, new_id)
        self._send_request(0x04, payload)

        response = self._read_response(0x04, 3)
        return bool(struct.unpack('<BBB', response)[2])
    

    def ping(self, servo_id:int) -> bool:
        """Check comms with target servo, returns bool indicating comms success"""
        servo_id = self._servo_id(servo_id)
        payload = struct.pack('<B', servo_id)
        self._send_request(0x01, payload)
        response = self._read_response(0x01, 1)
        if response is not None and struct.unpack('<B', response)[0] == servo_id:
            return True
        else:
            return False 


def test_servo(port):
    '''Test servo functions including ping, read angle and set angle, then tests modifying servo_id 0 to 0 (will return True if successful).'''
    import time
    servo = AUSServoBus(port, timeout=0.2)
    servo_id = 0
    print(f"Servo {servo_id} ping: {servo.ping(servo_id)}")
    print(f"Servo {servo_id} angle: {servo.read_angle(servo_id)}")

    for angle, duration in [(90, 800), (-90, 800), (355, 3000), (-135, 3000), (0, 500)]:
        print(f"Servo moving to {angle} in {duration}ms")
        servo.set_angle(servo_id, angle, duration)
        time.sleep(duration/1000 + 0.5)
        print(f"Servo {servo_id} angle: {servo.read_angle(servo_id)}")
        time.sleep(0.5)
        # print(angle, duration)

    print(f"Modify servo ID: {servo.set_id(servo_id, 0)}")
    servo.close_serial()


if __name__ == '__main__':
    PORT = "/dev/ttyUSB0"  # Change if needed
    test_servo(PORT)