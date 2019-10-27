""" Author - Kiel Rochow
Using Functions file to run main, to read signal in and either, convert signal back to platform
Or using autopilot convert controls into signal

"""

import readCovertFunc as cf



print("AUX1____Roll____Pitch____Yaw____AUX2____Throttle")


"""
#log inputs
MyDateTime = datetime.datetime.now()
date = MyDateTime.isoformat()
date = date.translate(string.maketrans("",""),":.-")
logfile = open("Reciever" + date + ".csv","w+")
logfile.write("AUX1____Roll____Pitch____Yaw____AUX2____Throttle\n")
"""



try:
    cf.align()
    while True:
        pre, servo_pos = cf.read()
        
        if servo_pos[aux1_ch] > 290:
          servo_pos = [1024 - x for x in servo_pos]
          datawrite = cf.dataWrite(pre, servo_pos)
        else:
          datawrite = cf.dataWrite(pre, servo_pos)
        
        sys.stdout.write(
            "%4d	%4d	%4d	%4d	%4d	%4d	%4d	%4d\r"%tuple(
            servo_pos[:8]))
        sys.stdout.flush()

						
	

	"""
        #datawrite = data_buf[4:6]
        datawrite = TB(ail_ch,ctrlIn(ail_ch))
        sys.stdout.write("{}\r".format(chr(datawrite)))
        #sys.stdout.write("{}\r".format(ord(datawrite[0])))
        #sys.stdout.write("{}\r".format(ord(datawrite[1])))
	#logfile.write("%4d, %4d, %4d, %4d, %4d, %4d\n"%tuple(
            #servo_position[:6]))
        #uses function convert, to convert incoming data
        #datawrite = [data_buf[:2], data_buf[2:4]] # TB(ail_ch,ctrlIn(ail_ch)) + TB(ele_ch,ctrlIn(ele_ch)) + TB(rud_ch,ctrlIn(rud_ch)) + data_buf[10:12] + TB(thr_ch,ctrlIn(thr_ch)) + data_buf[12:16]
        #print("{}".format(datawrite))
        #sys.stdout.write(
         #   "%4d	%4d	%4d	%4d	%4d	%4d	%4d	%4d\r"%tuple(
          #  datawrite[:8]))
	"""

	#writes data for platform
        ser.write(datawrite)
except(KeyboardInterrupt, SystemExit):
    ser.close()
    #logfile.close()
except(Exception) as ex:
    print(ex)
    ser.close()
    #logfile.close()
