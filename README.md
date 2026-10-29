# AUS500 Servo Library
Library to use AUS500 series servo. Contains AUSServoBus with functions to read angle, set angle, configure ID and ping servos on the created RS485 serial bus.

Tested on Hydrocean ACT-AU25S500-1 with a USB-RS485 converter on Ubuntu 24.04.


## Quickstart:
1. Create servo bus object.
- Initialise with own serial bus\
`servo = AUSServoBus(port, timeout=0.2)`
- Alternatively, initialise with existing serial bus. Bus must be 115200 baud, 8 bits bytesize, no parity, stopbits 1 based on datasheet.\
`servo = AUSServoBus(ser)`

2. Ping to check if servo with specified servo_id is connected. Default servo_id seems to be 0.\
`servo.ping(servo_id)`

3. Read angle of specified servo.\
`servo.read_angle(servo_id)`

4. Set angle and move duration of specified servo.\
`servo.set_angle(servo_id, angle, duration)`

5. Modify servo_id. Ensure only 1 servo of specified current id is connected!\
`servo.set_id(current_servo_id, new_servo_id)`

6. Running aus_servo.py as main will run the specified checks for servo_id 0 and port /dev/ttyUSB0 - modify parameters as required.


