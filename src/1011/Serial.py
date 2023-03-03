#출처 : https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=sueya89&logNo=221601039995

import serial
import datetime as dt
import csv

ser = serial.Serial('COM25', 115200)
ser.close()
ser.open()

try:
    while True:
        data = float(ser.readline().decode())
        i = dt.datetime.now().strftime('%H:%M:%S')
        with open("pid.csv", 'a') as fp:
            wr = csv.writer(fp, dialect='excel')
            wr.writerow([i, data])
            
except KeyboardInterrupt:
    ser.close()