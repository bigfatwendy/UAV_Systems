""" Author - Kiel Rochow
Creating a seperate file, which is a function to be called to read and 
return the chr signal to flight platform

"""

import serial
import time
import sys
import datetime
import string


# From remote receiver tutorial
def align_serial(ser):
    """Aligns the serial stream with the incoming Spektrum packets
    Spektrum Remote Receivers (AKA Spektrum Satellite) communicate serially
    in 16 byte packets at 125000 bits per second (bps)(aka baud) but are
    compatible with the standard 115200bps rate. We don't control the output
    transmission timing of the Spektrum receiver unit and so might start
    reading from the serial port in the middle of a packet transmission.
    To align the reading from the serial port with the packet transmission,
    we use the timing between packets to detect the interval between packets
    Packets are communicated every 11ms. At 115200 bps, a bit is read in 
    approximately 8.69us, so a 16 byte (128 bit)
    packet will take around 1.11ms to be communicated, leaving a gap of about
    9.89ms between packets. We align our serial port reading with the protocol
    by detecting this gap between reads.
    Note that we do not use the packet header contents because
        1) They are product dependent. Specifically, "internal" Spektrum
        receivers indicate the system protocol in the second byte of the header
        but "external" receivers do not. Further, different products are
        use different protocols and indicate this using the
        system protocol byte.
        2) Other bytes in the packet may take on the same value as the header
        contents. No bit patterns of a byte are reserved, so any byte in the
        data payload of the packet could match the values of the header bytes.
    Inputs
    ------
    ser: serial.Serial instance
        serial port to read from
    """
    data = None
    # read in the first byte, might be a long delay in case the transmitter is
    # off when the program begins
    ser.read(1)
    dt = 0
    # wait for the next long delay between reads
    dt_threshold = 0.010 # pick some threshold between 8.69us and 9.89ms
    while dt < dt_threshold:
        start = time.time()
        ser.read()
        dt = time.time()-start
    # consume the rest of the packet
    ser.read(15)
    # should be aligned with protocol now

MASK_CH_ID = 0b11111100 # 0x7800
SHIFT_CH_ID = 2
MASK_SERVO_POS_HIGH = 0b00000011 # 0x07FF
def parse_channel_data(data):
    """Parse a channel's 2 bytes of data in a remote receiver packet
    Inputs
    ------
    data: 2 byte long string (currently only supporting Python 2)
        Bytes within the remote receiver packet representing a channel's data
    Outputs
    -------
    channel_id, channel_data
    """
    ch_id = (ord(data[0]) & MASK_CH_ID) >> SHIFT_CH_ID
    ch_data = (
        ((ord(data[0]) & MASK_SERVO_POS_HIGH) << 8) | ord(data[1]))
    #ch_data = 988 + (ch_data >> 1)
    return ch_id, ch_data


#channel and servo position to 2 byte character string
def chpos2chr(ch, pos): 
	val = (ch << 10) | pos
	CHR = chr((val >> 8) & 0xff) + chr(val & 0xff)
	return CHR


#pre-amble for align


data = None



#Pre-Amble for Read and write
thr_ch = 1
ail_ch = 2
ele_ch = 3
rud_ch = 4
aux1_ch = 5
aux2_ch = 6


#Align function
def align():
        ser = serial.Serial(
        port="/dev/serial0", baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE)
        
        align_serial(ser) 

def read():
        ser = serial.Serial(
        port="/dev/serial0", baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE)
        
        N_CHAN = 13
        data = None
        servo_pos = [0 for i in range(N_CHAN)]
        
        data_buf = ser.read(16)
        data = data_buf[2:]
        for i in range(7):
            ch_id, s_pos = parse_channel_data(data[2*i:2*i+2])
            servo_pos[ch_id] = s_pos
        return data_buf[:2], servo_pos


def dataWrite(pre, servo_pos):
        #Pre-Amble for Read and write
        thr_ch = 1
        ail_ch = 2
        ele_ch = 3
        rud_ch = 4
        aux1_ch = 5
        aux2_ch = 6
        datawrite = pre + chpos2chr(aux1_ch, servo_pos[aux1_ch]) + chpos2chr(ail_ch, servo_pos[ail_ch]) + chpos2chr(ele_ch, servo_pos[ele_ch) + chpos2chr(rud_ch, servo_pos[rud_ch]) + chpos2chr(aux2_ch, servo_pos[aux2_ch]) + chpos2chr(thr_ch, servo_pos[thr_ch])
        return datawrite



