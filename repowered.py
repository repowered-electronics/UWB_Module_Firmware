import serial
from ctypes import *

HDDR_LEN = 2
STOP_BYTE = 0xA5

NUM_FIELDS = 8
CONFIG_FIELD_SIZE = 1
FIELD_SIZE = 4

FIELD_SELF_ID 	= 0x00
FIELD_MODE 		= 0x01
FIELD_CHANNEL 	= 0x02
FIELD_SAMPLES_PER_RANGE 	= 0x03
FIELD_NUMBER_OF_ANCHORS 	= 0x04
FIELD_X = 0x05
FIELD_Y = 0x06
FIELD_Z = 0x07

CMD_READ_CONFIG		= 0x11
CMD_READ_ANCHORS 	= 0x12
CMD_SET_CONFIG 		= 0x22
CMD_SET_COORDS      = 0x23
CMD_RANGE 			= 0x33
CMD_RESTART 		= 0x44
CMD_RESET 			= 0x55
CMD_SAVE_CONFIG 	= 0x66

class UwbConfig:
    def __init__(self, comport):
        self.ser = serial.Serial(comport)
        self.ser.timeout = 1.0
        self.ser.baudrate = 115200
        self.id = 0
        self.mode = 0
        self.channel = 2
        self.samples_per_range = 0
        self.number_of_anchors = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    fields = property(lambda self: [\
        c_uint(self.id), \
        c_uint(self.mode), \
        c_uint(self.channel), \
        c_uint(self.samples_per_range), \
        c_uint(self.number_of_anchors), \
        c_float(self.x), \
        c_float(self.y), \
        c_float(self.z)], \
        None, None, None)

    def set_config(self):
        packet = bytearray(HDDR_LEN + NUM_FIELDS*(FIELD_SIZE+1) + 1)
        ind = 0
        packet[ind] = CMD_SET_CONFIG
        ind += 1
        packet[ind] = NUM_FIELDS*(FIELD_SIZE+1)
        ind += 1

        for i in range(0, NUM_FIELDS):
            packet[ind] = i # id of the field to read
            ind += 1
            field = pointer(self.fields[i]) # get a pointer to the field
            field = cast(field, POINTER(c_ubyte))
            packet[ind] = field[3] 
            ind += 1
            packet[ind] = field[2]
            ind += 1
            packet[ind] = field[1]
            ind += 1
            packet[ind] = field[0]
            ind += 1

        packet[ind] = STOP_BYTE

        print(packet)
        self.ser.write(packet)

        try:
            hddr = self.ser.read(HDDR_LEN)
            body = self.ser.read(hddr[1])
            stop = self.ser.read(1)
            return hddr + body + stop
        except IndexError:
            print("Header didn't come through")
            return None


    def read_config(self):
        packet = bytearray(HDDR_LEN + NUM_FIELDS + 1)
        ind = 0
        packet[ind] = CMD_READ_CONFIG
        ind += 1
        packet[ind] = NUM_FIELDS
        ind += 1

        for i in range(0, NUM_FIELDS):
            packet[ind] = i # id of the field to read
            ind += 1

        packet[ind] = STOP_BYTE

        print(packet)
        self.ser.write(packet)

        try:
            hddr = self.ser.read(HDDR_LEN)
            body = self.ser.read(hddr[1])
            stop = self.ser.read(1)
            retval = hddr + body + stop
        except IndexError:
            print("Header didn't come through")
            retval = None

        print(len(retval))

        return retval


def config_to_str(arr):
    print(arr)
    retval =  "self id           : {:d}\r\n"
    retval += "mode              : {:d}\r\n"
    retval += "channel           : {:d}\r\n"
    retval += "samples-per-range : {:d}\r\n"
    retval += "number of anchors : {:d}\r\n"
    retval = retval.format(arr[FIELD_SELF_ID + HDDR_LEN], arr[FIELD_MODE + HDDR_LEN], arr[FIELD_CHANNEL + HDDR_LEN], arr[FIELD_SAMPLES_PER_RANGE + HDDR_LEN], arr[FIELD_NUMBER_OF_ANCHORS + HDDR_LEN])
    return retval

def read_config(ser):
    packet = bytearray(3)
    packet[0] = CMD_READ_CONFIG
    packet[1] = 0x00
    packet[2] = STOP_BYTE

    if ser.is_open and ser.timeout != None:
        written = ser.write(packet)
        print("{:d} bytes written".format(written))
        rx_hddr = ser.read(2) # read the header

        if rx_hddr[0] == CMD_READ_CONFIG:
            rx_body = ser.read(rx_hddr[1] + 1) # include the stop byte

            rx_packet = rx_hddr + rx_body
            print("read {:d} bytes from {:s}".format(len(rx_packet), ser.port))
            print(rx_packet)
            return rx_packet
        else:
            print("Port {:s} not ready yet".format(ser.port))

            return bytearray()

def set_config(uwb_config, ser):
    packet = bytearray(HDDR_LEN + CONFIG_FIELD_SIZE*NUM_FIELDS + 1)
    packet[0] = CMD_SET_CONFIG
    packet[1] = CONFIG_FIELD_SIZE*NUM_FIELDS
    print(uwb_config.fields)
    for i in range(0,NUM_FIELDS):
        packet[HDDR_LEN + i] = uwb_config.fields[i]

    packet[len(packet)-1] = STOP_BYTE
    print(packet)
    ser.write(packet)
    print(ser.read(3))

