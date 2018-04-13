import pymongo
import serial
import time

# define constants for commands
PIN_0 = chr(0)
PIN_1 = chr(1)
PIN_2 = chr(2)
PIN_3 = chr(3)
MOVE_RIGHT = chr(4)
MOVE_LEFT = chr(5)
MOVE_RIGHT_FULL = chr(6)
MOVE_LEFT_FULL = chr(7)
OK = chr(0)
VREF = 5.0 # reference voltage for ADC

SERIAL_DEVICE = serial.Serial(port='/dev/ttyS0',
                              baudrate=2400,
                              parity=serial.PARITY_NONE,
                              stopbits=serial.STOPBITS_ONE,
                              bytesize=serial.EIGHTBITS,
                              timeout=3)

DB = pymongo.MongoClient().readings

def send_cmd(command):
    SERIAL_DEVICE.write(command)

def recieve_ack():
    return SERIAL_DEVICE.read() == OK

def send_reading_cmd(pin):
    SERIAL_DEVICE.write(pin)

def recieve_reading():
    return SERIAL_DEVICE.read()

def calc_voltage(reading):
    if not reading:
        return 0
    return ord(reading) * VREF / 256

def get_voltage(pin):
    send_reading_cmd(pin)
    return calc_voltage(recieve_reading())

def save_readings_to_db(*readings):
    data = {'v{}'.format(i): v for i, v in enumerate(readings)}
    data.update({'t': int(time.time())})
    DB.readings.insert(data)

def check_panels():
    while True:
        v1 = get_voltage(PIN_0)
        v2 = get_voltage(PIN_1)
        save_readings_to_db(v1, v2)
        diff = v1 - v2

        if abs(v1 - v2) < 0.2:
            # v1 and v2 are close enough together
            break
        elif v1 > v2:
            send_cmd(MOVE_RIGHT_FULL)
        elif v2 > v1:
            send_cmd(MOVE_LEFT_FULL)

if __name__ == '__main__':
    # print get_voltage(PIN_0)
    # print get_voltage(PIN_1)
    # print get_voltage(PIN_2)
    # print get_voltage(PIN_3)
    # while True:
        # send_cmd(MOVE_RIGHT)
        # if not recieve_ack():
            # break
        # print 'Done'
        # time.sleep(1)
    # check_panels()
    send_cmd(MOVE_RIGHT_FULL)
    send_cmd(MOVE_RIGHT_FULL)
    send_cmd(MOVE_LEFT_FULL)
    send_cmd(MOVE_LEFT_FULL)
