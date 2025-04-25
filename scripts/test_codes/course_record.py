#!/usr/bin/env python3

import rospy
import rospkg
import os, sys, time
import threading

import pandas as pd
import numpy as np

import serial
import struct
import datetime

from gnss_ros.msg import Monitor
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

PI = 3.14159265359
DATE        = 0
TIME        = 1
LAT         = 2
LON         = 3
MODE        = 4
CNO         = 5
ALT         = 6
ROLL        = 7
PITCH       = 8
YAW         = 9
GPS_VAL     = 10
RSSI        = 11
BAT         = 12
ACC_N       = 13
ACC_E       = 14
ACC_D       = 15
VEL_N       = 16
VEL_E       = 17
VEL_D       = 18
ODOM_X      = 19
ODOM_Y      = 20
ODOM_TH     = 21

CMD_ACC     = 1
CMD_DEC     = 2
CMD_ROT_L   = 3
CMD_ROT_R   = 4
CMD_STOP    = 9
CMD_REC     = 11
CMD_QUIT    = 99

CMD_LIST = [CMD_ACC, CMD_DEC, CMD_ROT_L, CMD_ROT_R, CMD_STOP, CMD_REC, CMD_QUIT]

V_MAX = 0.15
W_MAX = 30

serial_struct = struct.Struct("<BHI16sHBBI3Bb3Bf2H3B2qiB9f4B")

def deg2rad(deg):
    return deg * PI / 180.0

def rad2deg(rad):
    return rad * 180.0 / PI

rp = rospkg.RosPack()
root_dir = rp.get_path('gnss_ros')


class Recorder:
    def __init__(self, gnss_port, gnss_baudrate, odom_enable, drive_enable=True):
        self.rate1 = rospy.Rate(100)
        self.rate2 = rospy.Rate(100)
        self.rate3 = rospy.Rate(100)
        
        self.gnss_port = gnss_port
        self.gnss_baudrate = gnss_baudrate
        self.ser = None
        self.rx_buffer = bytearray()
        
        self.cmd = None
        self.odom_enable = odom_enable
        self.drive_enable = drive_enable
        
        self.cur_v = 0.0
        self.cur_w = 0.0
        
        self.vmax = V_MAX
        self.wmax = deg2rad(W_MAX)
        self.acc = V_MAX / 20.0
        self.alp = W_MAX / 20.0
        
        self.prev_t = time.time()
        
        self.operation      = True
        self.record_flag    = False
        self.save_data_flag = False
        self.quit_flag      = False
        
        self.serial_number  = None
        
        self.attrs = ['DATE', 'TIME', 'MODE', 'CNO', 'LAT', 'LON', 'ALT', 'ROLL', 'PITCH', 'YAW',
                      'GPS_VAL', 'RSSI', 'BAT', 'ACC_N', 'ACC_E', 'ACC_D', 'VEL_N', 'VEL_E', 
                      'VEL_D', 'ODOM_X', 'ODOM_Y', 'ODOM_TH']
        
        self.date       = 0.0
        self.time       = 0.0
        self.latitude   = 0.0
        self.longitude  = 0.0
        self.altitude   = 0.0
        self.roll       = 0.0
        self.pitch      = 0.0
        self.yaw        = 0.0
        self.gps_val    = 0.0
        self.rssi       = 0.0
        self.bat        = 0.0
        self.acc_n      = 0.0
        self.acc_e      = 0.0
        self.acc_d      = 0.0
        self.vel_n      = 0.0
        self.vel_e      = 0.0
        self.vel_d      = 0.0
        self.cno        = 0.0
        self.mode       = 0.0
        
        self.odom_x     = 0.0
        self.odom_y     = 0.0
        self.odom_th    = 0.0
        
        self.drive_flag = [0, 0, 0, 0, 1]
        
        self.max_lines  = 2000
        self.line_no    = 0
        self.db = np.zeros((self.max_lines, len(self.attrs)), dtype=float)
        self.dbs = []
        
        self.pub_cnt = 1
        
        self.cmd_sub        = rospy.Subscriber('/cmd', UInt8, self.cmd_callback)
        if self.drive_enable:
            self.cmd_vel_sub    = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        else:
            self.cmd_vel_pub    = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.monitor_pub    = rospy.Publisher('/monitor', Monitor, queue_size=10)
        if self.odom_enable:
            self.odom_sub   = rospy.Subscriber('/odom', Odometry, self.odom_callback)
            
        self.lock = threading.Lock()
        
        self.drive_thread   = threading.Thread(target=self.drive_task, args=())
        self.gnss_thread    = threading.Thread(target=self.gnss_task, args=())
        self.record_thread  = threading.Thread(target=self.record_task, args=())
        
    def cmd_callback(self, msg: UInt8):
        cmd = msg.data
        if cmd in CMD_LIST:
            self.lock.acquire()
            if cmd == CMD_QUIT:
                self.quit_flag = True
                if self.record_flag:
                    self.record_flag = False
                    self.save_data_flag = True
                else:
                    self.operation = False
            elif cmd == CMD_REC:
                if self.record_flag:
                    self.record_flag = False
                    self.save_data_flag = True
                else:
                    self.record_flag = True
                    print('[RECORD] Start recording.')
            elif cmd == CMD_ACC:
                if self.drive_enable:
                    self.drive_flag = [1, 0, 0, 0, 0]
            elif cmd == CMD_DEC:
                if self.drive_enable:
                    self.drive_flag = [0, 1, 0, 0, 0]
            elif cmd == CMD_ROT_L:
                if self.drive_enable:
                    self.drive_flag = [0, 0, 1, 0, 0]
            elif cmd == CMD_ROT_R:
                if self.drive_enable:
                    self.drive_flag = [0, 0, 0, 1, 0]
            elif cmd == CMD_STOP:
                if self.drive_enable:
                    self.drive_flag = [0, 0, 0, 0, 1]
                
            self.lock.release()
            
    def cmd_vel_callback(self, msg: Twist):
        self.lock.acquire()
        self.cur_v = msg.linear.x
        self.cur_w = msg.angular.z
        self.lock.release()
    
    def odom_callback(self, msg: Odometry):
        self.lock.acquire()
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_th = msg.pose.pose.orientation.z
        self.lock.release()
    
    def publish_monitoring_msg(self):
        msg = Monitor()
        msg.header.stamp = rospy.Time.now()
        self.lock.acquire()
        msg.x = self.odom_x
        msg.y = self.odom_y
        msg.th = self.odom_th
        msg.lat = self.latitude
        msg.lon = self.longitude
        msg.roll = self.roll
        msg.pitch = self.pitch
        msg.yaw = self.yaw
        msg.acc_n = self.acc_n
        msg.acc_e = self.acc_e
        msg.acc_d = self.acc_d
        msg.vel_n = self.vel_n
        msg.vel_e = self.vel_e
        msg.vel_d = self.vel_d
        self.lock.release()
        self.monitor_pub.publish(msg)
    
    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.cur_v
        msg.angular.z = self.cur_w
        
        self.cmd_vel_pub.publish(msg)
    
    def calc_odom(self):
        cur_t = time.time()
        dt = cur_t - self.prev_t
        self.prev_t = cur_t
        self.odom_th += dt * self.cur_w
        self.odom_x += (dt * self.cur_v) * np.cos(self.odom_th)
        self.odom_y += (dt * self.cur_v) * np.sin(self.odom_th)
        
    def v_profile(self, dir=1):
        if self.cur_w != 0.0:
            self.cur_w = 0.0
            
        if dir == 1:
            if self.vmax <= self.cur_v + self.acc:
                self.cur_v = self.vmax
            else:
                self.cur_v += self.acc
                
        elif dir == -1:
            if -self.vmax >= self.cur_v - self.acc:
                self.cur_v = -self.vmax
            else:
                self.cur_v -= self.acc
                
    def w_profile(self, dir=1):
        if self.cur_v != 0.0:
            self.cur_v = 0.0
        if dir == 1:
            if self.wmax <= self.cur_w + self.alp:
                self.cur_w = self.wmax
            else:
                self.cur_w += self.alp
        elif dir == -1:
            if -self.wmax >= self.cur_w - self.alp:
                self.cur_w = -self.wmax
            else:
                self.cur_w -= self.alp
                
    def stop_profile(self):
        if self.cur_v > 0.0:
            if self.cur_v - self.acc <= 0.0:
                self.cur_v = 0.0
            else:
                self.cur_v -= self.acc
        elif self.cur_v < 0.0:
            if self.cur_v + self.acc >= 0.0:
                self.cur_v = 0.0
            else:
                self.cur_v += self.acc
        
        if self.cur_w > 0.0:
            if self.cur_w - self.alp <= 0.0:
                self.cur_w = 0.0
            else:
                self.cur_w -= self.alp
        elif self.cur_w < 0.0:
            if self.cur_w + self.alp >= 0.0:
                self.cur_w = 0.0
            else:
                self.cur_w += self.alp
        
    def connect(self):
        retry_cnt = 0
        ret = False
        
        while True:
            self.ser = serial.Serial(
                port=self.gnss_port,
                baudrate=self.gnss_baudrate,
                parity=serial.PARITY_NONE,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001
            )
            
            time.sleep(0.05)
            
            if self.ser.isOpen():
                ret = True
                break
            else:
                try:
                    self.ser.close()
                except:
                    pass
                retry_cnt += 1
                
                if retry_cnt == 10:
                    break
        return ret
    
    def parse_msg(self):
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
                
                if self.serial_number is None:
                    self.serial_number = unpacked[3].decode()
                year = unpacked[4] * 1.0
                month = unpacked[5] * 1.0
                day = unpacked[6] * 1.0
                date = year * 10 ** 4 + month * 100 + day
                t = unpacked[7] * 1.0
                mode = unpacked[9] * 1.0
                cno = unpacked[10] * 1.0
                rssi = unpacked[11] * 1.0
                bat = unpacked[12] * 1.0
                gps_val = unpacked[14] * 1.0
                latitude = unpacked[21] * 1e-10
                longitude = unpacked[22] * 1e-10
                altitude = unpacked[23] * 1e-3
                roll = unpacked[25]
                pitch = unpacked[26]
                yaw = unpacked[27]
                acc_n = unpacked[28]
                acc_e = unpacked[29]
                acc_d = unpacked[30]
                vel_n = unpacked[31]
                vel_e = unpacked[32]
                vel_d = unpacked[33]
                
                self.lock.acquire()
                self.date = date
                self.time = t
                self.mode = mode
                self.cno = cno
                self.rssi = rssi
                self.bat = bat
                self.gps_val = gps_val
                self.latitude = latitude
                self.longitude = longitude
                self.altitude = altitude
                self.roll = roll
                self.pitch = pitch
                self.yaw = yaw
                self.acc_n = acc_n
                self.acc_e = acc_e
                self.acc_d = acc_d
                self.vel_n = vel_n
                self.vel_e = vel_e
                self.vel_d = vel_d
                self.lock.release()
            else:
                self.rx_buffer = self.rx_buffer[search_range:]
    
    def insert_datum(self):
        self.lock.acquire()
        self.db[self.line_no, :] = [self.date, self.time, self.mode, self.cno, self.latitude, 
                                    self.longitude, self.altitude, self.roll, self.pitch, self.yaw, 
                                    self.gps_val, self.rssi, self.bat, self.acc_n, self.acc_e, 
                                    self.acc_d, self.vel_n, self.vel_e, self.vel_d, self.odom_x, 
                                    self.odom_y, self.odom_th]
        self.line_no += 1
        if self.line_no == self.max_lines:
            tmp_df = pd.DataFrame(self.db, columns=self.attrs)
            self.dbs.append(tmp_df)
            self.line_no = 0
        self.lock.release()
        
    def save_data(self):
        date = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        if self.line_no != 0:
            tmp_df = pd.DataFrame(self.db[:self.line_no, :], columns=self.attrs)
            self.dbs.append(tmp_df)
        for i, db in enumerate(self.dbs):
            db.to_csv(root_dir + '/scripts/FITO/{}_{}_{}.csv'.format(self.serial_number, date, i), index=False)
        #rst db
        self.line_no = 0
        self.db = np.zeros((self.max_lines, len(self.attrs)), dtype=float)
        self.dbs = []
        
        print('[RECORD] Data saved.')
    
    def print_info(self):
        self.lock.acquire()
        odom_x = self.odom_x
        odom_y = self.odom_y
        odom_th = self.odom_th
        lat = self.latitude
        lon = self.longitude
        alt = self.altitude
        r = self.roll
        p = self.pitch
        y = self.yaw
        vn = self.vel_n
        ve = self.vel_e
        vd = self.vel_d
        self.lock.release()
        
        print('===========================================\r')
        print('Odometry | x: {:.4f}, y: {:.4f}, th: {:.4f}\r'.format(odom_x, odom_y, odom_th))
        print('GNSS:    | LAT: {:.5f}, LON: {:.5f}, ALT: {:.2f}\r'.format(lat, lon, alt))
        print('IMU:     | R: {:.3f}, P: {:.3f}, Y: {:.3f}\r'.format(rad2deg(r), rad2deg(p), rad2deg(y)))
        print('VEL:     | V_n: {:.3f} m/s, V_e: {:.3f} m.s, V_d: {:.3f} m/s\r'.format(vn, ve, vd))
        print('===========================================\r')
    
    def drive_task(self):
        op = True
        drive_flag = [0, 0, 0, 0, 1]
        odom_enable = self.odom_enable
        drive_enable = self.drive_enable
        print('[DRIVE] Thread start.')
        
        while op:
            self.lock.acquire()
            op = self.operation
            drive_flag = self.drive_flag
            self.lock.release()
            if not odom_enable:
                self.calc_odom()
            
            if drive_enable:
                if drive_flag[0] == 1:
                    self.v_profile(1)
                elif drive_flag[1] == 1:
                    self.v_profile(-1)
                elif drive_flag[2] == 1:
                    self.w_profile(1)
                elif drive_flag[3] == 1:
                    self.w_profile(-1)
                elif drive_flag[4] == 1:
                    self.stop_profile()

                self.publish_cmd_vel()
            
            self.rate1.sleep()
        
        self.cur_v = 0
        self.cur_W = 0
        self.publish_cmd_vel()
        print('[DRIVE] Thread terminated.')
               
    def gnss_task(self):
        op = True
        
        print('[GNSS_SERIAL] Thread start.')
        
        if not self.connect():
            self.lock.acquire()
            self.operation = False
            self.lock.release()
            print('[GNSS_SERIAL] GNSS CONNECTION FAILED')
            return
        try:
            while op:
                self.lock.acquire()
                op = self.operation
                self.lock.release()
                
                self.parse_msg()
                self.rate2.sleep()
        finally:
            self.ser.close()
            print('[GNSS_SERIAL] Thread terminated.')
            
    def record_task(self):
        op = True
        record_flag = False
        save_data_flag = False
        print('[RECORD] Thread start.')
        cnt = 1
        
        if not self.connect():
            self.lock.acquire()
            self.operation = False
            self.lock.release()
            return
        
        while op:
            self.lock.acquire()
            op = self.operation
            record_flag = self.record_flag
            save_data_flag = self.save_data_flag
            self.lock.release()
            
            if record_flag:
                self.insert_datum()
            
            if save_data_flag:
                print('[RECORD] Stop recording.')
                self.save_data()
                self.lock.acquire()
                self.save_data_flag = False
                if self.quit_flag:
                    self.operation = False
                self.lock.release()                
            
            if cnt == 1000:
                self.print_info()
                cnt = 1
            else:
                cnt += 1
            
            self.rate3.sleep()
        
        print('[RECORD] Thread terminated.')
        
    def run(self):
        try:
            if not self.odom_enable:
                self.drive_thread.start()
            self.gnss_thread.start()
            self.record_thread.start()
            
        finally:
            if self.drive_thread.is_alive():
                self.drive_thread.join()
            if self.gnss_thread.is_alive():
                self.gnss_thread.join()
            if self.record_thread.is_alive():
                self.record_thread.join()
                

if __name__ == '__main__':
    rospy.init_node('gnss_record', anonymous=True)
    
    GNSS_PORT = '/dev/fitogether'
    GNSS_BAUDRATE = 115200
    ODOM_ENABLE = False
    DRIVE_ENABLE = False
    
    if rospy.has_param('~gnss_port'):
        GNSS_PORT = rospy.get_param('~gnss_port')
    if rospy.has_param('~gnss_baudrate'):
        GNSS_BAUDRATE = rospy.get_param('~gnss_baudrate')
    if rospy.has_param('~odom_enable'):
        ODOM_ENABLE = rospy.get_param('~odom_enable')
    if rospy.has_param('~drive_enable'):
        DRIVE_ENABLE = rospy.get_param('~drive_enable')
    
    node = Recorder(
        gnss_port=GNSS_PORT,
        gnss_baudrate=GNSS_BAUDRATE,
        odom_enable=ODOM_ENABLE,
        drive_enable=DRIVE_ENABLE
    )
    
    node.run()
    
    