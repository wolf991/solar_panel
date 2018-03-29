import serial
import time

# define constants for commands
PIN_0 = chr(0)
PIN_1 = chr(1)
PIN_2 = chr(2)
PIN_3 = chr(3)
OK = chr(0)
VREF = 5.0 # reference voltage for ADC

SERIAL_DEVICE = serial.Serial(port='/dev/ttyS0',
                              baudrate=2400,
                              parity=serial.PARITY_NONE,
                              stopbits=serial.STOPBITS_ONE,
                              bytesize=serial.EIGHTBITS,
                              timeout=2)

def send_cmd(command):
    SERIAL_DEVICE.write(command)

def recieve_ack():
    return ord(SERIAL_DEVICE.read()) == OK

def send_reading_cmd(pin):
    SERIAL_DEVICE.write(pin)

def recieve_reading():
    return SERIAL_DEVICE.read()

def calc_voltage(reading):
    return ord(reading) * VREF / 256

def get_voltage(pin):
    send_reading_cmd(pin)
    return calc_voltage(recieve_reading())

if __name__ == '__main__':
    print get_voltage(PIN_0)
    print get_voltage(PIN_1)
    print get_voltage(PIN_2)
    print get_voltage(PIN_3)
