import rospy
import os, sys, time
import datetime

import numpy as np
import rospkg
import json

import threading
import serial
import struct
import select

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
    
serial_struct = struct.Struct("<BHI16sHBBI3Bb3Bf2H3B2qiB9f4B")

rp = rospkg.RosPack()
SAVE_DIR = rp.get_path('gnss_ros') + '/config/'


class Node:
    def __init__(self, gnss_port, gnss_baudrate):
        self.course = []
        self.gnss_port = gnss_port
        self.gnss_baudrate = gnss_baudrate
        
        self.rate1 = rospy.Rate(100)
        self.rate2 = rospy.Rate(50)
        self.rate3 = rospy.Rate(20)
        
        if os.name == 'nt':
            self.settings = None
        else:
            self.settings = termios.tcgetattr(sys.stdin)
        
        self.cmd = None
        
        self.serial_no = None
        
        self.latitude = None
        self.longitude = None
        
        self.ser = None
        
        self.operation = True
        
        self.lock = threading.Lock()
        
        self.gnss_thread = threading.Thread(target=self.gnss_task, args=())
        self.keyin_thread = threading.Thread(target=self.keyin_task, args=())
        self.path_save_thread = threading.Thread(target=self.path_save_task, args=())
    
    def get_key(self):
        if os.name == 'nt':
            timeout = 0.1
            start_time = time.time()
            
            while(1):
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif time.time() - start_time > timeout:
                    return ''
                
        else:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
                
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        
    def connect_to_gnss(self):
        self.ser = serial.Serial(
            port=self.gnss_port,
            baudrate=self.gnss_baudrate,
            parity=serial.PARITY_NONE,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01
        )
        
        if self.ser.isOpen():
            return True
        else:
            return False
            
    def parse_gnss_data(self):
        self.rx_buffer += self.ser.readall()
        
        while len(self.rx_buffer) > 114:
            start_idx = -1
            search_range = len(self.rx_buffer) - 110
            
            for i in range(search_range):
                if self.rx_buffer[i:i+4] == b'FITO':
                    start_idx = i+4
                    break
            
            if start_idx != -1:
                rcv = self.rx_buffer[start_idx: start_idx+110]
                self.rx_buffer = self.rx_buffer[start_idx+110:]
                
                unpacked = serial_struct.unpack(rcv)
                
                latitude = unpacked[21] * 1e-10
                longitude = unpacked[22] * 1e-10
                
        return latitude, longitude
    
    def save_data(self):
        self.lock.acquire()
        course = self.course
        self.lock.release()
        
        if len(course) >= 2:
            data = {'COURSE': self.course}
            t = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            save_dir = SAVE_DIR + '/' + t + '.json'
            
            with open(save_dir, 'w') as f:
                json.dump(data, f)
            print('Course saved.')
        else:
            print('No data saved.')
        
    def gnss_task(self):
        op = True
        lat = None
        lon = None
        
        while op:
            lat, lon = self.parse_gnss_data()
            
            self.lock.acquire()
            op = self.operation
            if lat is not None:
                self.latitude = lat
                self.longitude = lon
            self.lock.release()
            self.rate1.sleep()           
            
        self.ser.close()
    
    def keyin_task(self):
        op = True
        
        while op:
            cmd = self.get_key()
            
            self.lock.acquire()
            if cmd == 'q':
                self.save_data()
                self.operation = False
                break
            elif cmd != '':
                self.cmd = cmd
            
            op = self.operation
            self.lock.release()
                
            self.rate2.sleep()
            
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def path_save_task(self):
        op = True
        cmd = None
        
        while op:
            self.lock.acquire()
            op = self.operation
            if self.cmd == 's':
                self.course.append([self.latitude, self.longitude])
                self.cmd = None
            self.lock.release()
            self.rate3.sleep()
            
    def run(self):
        try:
            self.gnss_thread.start()
            self.keyin_thread.start()
            self.path_save_thread.start()
        finally:
            if self.gnss_thread.is_alive():
                self.gnss_thread.join()
            if self.keyin_thread.is_alive():
                self.keyin_thread.join()
            if self.path_save_thread.is_alive():
                self.path_save_thread.jion()
                

if __name__ == '__main__':
    rospy.init_node('SAVE_PATH', anonymous=True)
    
    gnss_port = ''
    gnss_baudrate = 115200
    
    node = Node(gnss_port=gnss_port,
                gnss_baudrate=gnss_baudrate)
    
    node.run()