"""Monitors the UART port on a Raspberry Pi 3 for Spektrum serial packets

Assumes the packets follow the Remote Receiver format
Forwards the packets on the TX pin of the serial port, so you can pass the
packets on the the flight control board
"""


import serial
import time
import sys
import datetime
import string
import threading

class Reader:

    def __init__(self, name, log):

        self.name = name
        self.values = [0,0,0,0,0,0,0]
        self.read = True
        self.stream = []
        self.log = log
        self._lock = threading.Lock()
        self.MASK_CH_ID = 0b11111100 # 0x7800
        self.SHIFT_CH_ID = 2
        self.MASK_SERVO_POS_HIGH = 0b00000011 # 0x07FF
        self.ser = serial.Serial(
            port="/dev/serial0", baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE)

        N_CHAN = 13
        self.servo_position = [0 for i in range(N_CHAN)]

    def align_serial(self, ser):
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

    def parse_channel_data(self, data):
        """Parse a channel's 2 bytes of data in a remote receiver packet

        Inputs
        ------
        data: 2 byte long string (currently only supporting Python 2)
            Bytes within the remote receiver packet representing a channel's data

        Outputs
        -------
        channel_id, channel_data
        """
        ch_id = ((data[0]) & self.MASK_CH_ID) >> self.SHIFT_CH_ID
        ch_data = (
            (((data[0]) & self.MASK_SERVO_POS_HIGH) << 8) | (data[1]))
        #ch_data = 988 + (ch_data >> 1)
        #print(ch_id)
        #print(ch_data)
        return ch_id, ch_data

    def convert(self, positions, rawdata):
        newdata = rawdata
        return newdata
        
    
    ###KIELS Stuff
    def chpos2bytes(seld, ch, pos): #TB for two byte
	    val = (ch << 10) | pos
	    #return bytes([(val >> 8) & 0xff, val & 0xff]) 
	    return (val).to_bytes(2, byteorder='big')
	    
	def dataWrite(self, pre, servo_pos):
        #Pre-Amble for Read and write
        thr_ch = 6
        ail_ch = 3
        ele_ch = 1
        rud_ch = 4
        aux1_ch = 0
        aux2_ch = 5 #mode channel
        datawrite = pre + chpos2bytes(aux1_ch, servo_pos[aux1_ch]) + chpos2bytes(ail_ch, servo_pos[ail_ch]) + chpos2bytes(ele_ch, servo_pos[ele_ch) + chpos2bytes(rud_ch, servo_pos[rud_ch]) + chpos2bytes(aux2_ch, servo_pos[aux2_ch]) + chpos2bytes(thr_ch, servo_pos[thr_ch])
        return datawrite
	    
	    

    
    def ReaderThread(self, name):
        self.align_serial(self.ser)
        print('I am reading now and saving values to Values')
        data = None
        data_buf = None
        modeCh = 5
        try:
            while self.read:
                ### TODO: this is where we will be repeating until we have enough
                ### bites to read a signal and decode it
                with self._lock:
                    data_buf = self.ser.read(16)
                    #print(data_buf)
                    data = data_buf[2:]
                    pre = data_buf[:2]
                    #print(data)
                    for i in range(7):
                        #print(data[2*i:2*i+2])
                        ch_id, s_pos = self.parse_channel_data(data[2*i:2*i+2])
                        if ch_id > 6:
                            ch_id = 6
            	    #print("ch_id: " + str(ch_id) + " pos: " + str(s_pos))
                        self.servo_position[ch_id] = s_pos
            	   # servo_chanel[i] = ch_id
                    self.log.write("%4d, %4d, %4d, %4d, %4d, %4d\n"%tuple(
                        self.servo_position[:6]))
                    self.values = self.servo_position[:7]
                    #datawrite = convert(servo_position, data_buf)
                    
                    ### if statement to check mode
                    if modeCh < 350: #Manual Mode
                      servo_pos = servo_position
                    elif modeCh > 500: #Autonomous
                      servo_pos = [1024 - x for x in servo_position]
                    else:
                      servo_pos = [1024 - x for x in servo_position]
                    
                    datawrite = self.datawrite(pre, servo_pos)
                    
                    self.ser.write(datawrite)
        except Exception as e:
            self.ser.close()
            print(e)

                #stream here
            #time.sleep(1)

    def SaveValues(self, listToSave):
        self.stream = listToSave

    def GetValues(self):
        ### TODO: this is where we need to read the controller values.
        return self.values

    def run(self):

        dataThread = threading.Thread( target = self.ReaderThread, args = ("Reader Thread", ))
        dataThread.start()

        return dataThread

    def Stop(self):
        self.read = False

def main():
    print("AUX1____Roll____Pitch____Yaw____AUX2____Throttle")
    data = None
    #servo_chanel = [0 for i in range(N_CHAN)]
    MyDateTime = datetime.datetime.now()
    #date = MyDateTime.isoformat()
    #date = date.translate(string.maketrans("",""),":.-")
    #logfile = open("Reciever" + date + ".csv","w+")
    #logfile.write("AUX1____Roll____Pitch____Yaw____AUX2____Throttle\n")
    reader = Reader("reader", None)
    try:
        reader.align_serial(reader.ser)
        while True:
            data_buf = reader.ser.read(16)
            print(data_buf)
            data = data_buf[2:]
            print(data)
            for i in range(7):
                print(data[2*i:2*i+2])
                ch_id, s_pos = reader.parse_channel_data(data[2*i:2*i+2])
    	    #print("ch_id: " + str(ch_id) + " pos: " + str(s_pos))
                reader.servo_position[ch_id] = s_pos
            # servo_chanel[i] = ch_id
            sys.stdout.write(
                "%4d	%4d	%4d	%4d	%4d	%4d	%4d	%4d\r"%tuple(
                reader.servo_position[:8]))

            sys.stdout.flush()

            reader.ser.write(data_buf)
    except(KeyboardInterrupt, SystemExit):
        reader.ser.close()
        logfile.close()
    except(Exception) as ex:
        print(ex)
        reader.ser.close()
        logfile.close()

if __name__ == "__main__":
    main()
