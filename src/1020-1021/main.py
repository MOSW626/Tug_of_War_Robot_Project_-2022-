# com6 : 모터부
# com8 : 센서부

import time
from datetime import datetime
from multiprocessing import Pool, Process
import os

import serial

port = 'com9'
rate = 115200


def getSensor(name):
    arduino = serial.Serial(port, rate)
    time.sleep(1)
    now = datetime.now()
    title = str(now.strftime('%Y-%m-%d %H.%M.%S')) + ".csv"
    f = open(title, 'w')
    f.close()
    while True:
        
        f = open(title, 'a')
        get = arduino.readline()
        try:
        # _ = arduino.readline() # trash0
            get = get.decode('utf-8')
        except:
            continue
        print("get", get)
        print(get.split())
        if len(get.split()) <= 1:
            continue
        data = get.split()[1]
        print("data:", data)

        f.write(data+"\n")
        
        f.close()
 
if __name__ == '__main__':
    print('pid of main:', os.getpid())
 
    p1 = Process(target=getSensor, args=("proc_sensor",))
    p1.start()
    p1.join()