#!/usr/bin/env python3

import rospy
import rospkg
import os, sys, time
import pyproj
import numpy as np
import threading

import serial
import struct
# from 
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import UInt8
from gnss_ros.msg import GNSS
import tf.transformations

ORIGIN_LON = None # 
ORIGIN_LAT = None # 

MSG_LENGTH = 110

PI = 3.14159265359

SYSTEM_SHUTDOWN_CODE = 99

# TO-DO : merge string into cofig. file
serial_struct = struct.Struct("<BHI16sHBBI3Bb3Bf2H3B2qiB9f4B")
epsg4326 = pyproj.CRS.from_string("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs")
epsg5179 = pyproj.CRS.from_string("+proj=tmerc +lat_0=38 +lon_0=127.5 +k=0.9996 +x_0=1000000 +y_0=2000000 +ellps=GRS80 +units=m +no_defs")

rp = rospkg.RosPack()
root_dir = rp.get_path('gnss_ros')

def deg2rad(deg):
    return deg * PI / 180.0

def rad2deg(rad):
    return rad * 180.0 / PI


class GNSSnode:
    def __init__(self, port, baudrate, display_rawdata=False, origin_coord = None):
        self.serial_rate = rospy.Rate(100)
        self.convert_rate = rospy.Rate(20)
        self.operation = True
        
        self.GNSS_updated = False
        self.GNSS_available = False
        self.transformer = pyproj.Transformer.from_crs(epsg4326, epsg5179)
        
        if origin_coord is not None and len(origin_coord) == 2:
            self.origin_longitude = origin_coord[0]
            self.origin_latitude = origin_coord[1]
            self.origin_e, self.origin_n = self.transformer.transform(self.origin_longitude, selforigin_latitude)
        else:
            self.origin_longitude = None
            self.origin_latitude = None
            
        self.origin_e   = None
        self.origin_n   = None
        self.e          = None
        self.n          = None
        self.rel_e      = None
        self.rel_n      = None
        
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        
        self.rx_buffer = bytearray()
        
        self.display = display_rawdata
        
        self.GNSS_data = {
            'serial_number': None,
            'date': None,
            'time': None,
            'latitude': None,
            'longitude': None,
            'altitude': None,
            'mode': None,
            'cn0': None,
            'IMU': {
                'roll': None,
                'pitch': None,
                'yaw': None,    
            },
            'gps_valid': None,
            'RSSI': None,
            'Battery': None,
            'acceleration': {
                'n': None,
                'e': None,
                'd': None,
            },
            'velocity': {
                'n': None,
                'e': None,
                'd': None
            }
        }
        
        self.GNSS_topic = '/GNSS' # TO-DO -> read from topic list file
        self.odom_topic = '/GNSS/odom' # TO-DO -> read from topic list file
        self.system_shutdown_topic = '/cmd' # TO-DO -> synchronize with ltrp_ros
        
        self.GNSS_pub = rospy.Publisher(self.GNSS_topic, GNSS, queue_size=10) # MSG cutomization
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10) # MSG customization
        self.system_shutdown_sub = rospy.Subscriber(self.system_shutdown_topic, self.system_shutdown_callback)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.lock = threading.Lock()
        
        self.serial_thread = threading.Thread(target=self.serial_task, args=())
        self.convert_thread = threading.Thread(target=self.convert_task, args=())
        
    def system_shutdown_callback(self, msg: UInt8):
        if msg.data == SYSTEM_SHUTDOWN_CODE:
            print('[SYSTEM] SYSTEM SHUTDOWN')
            self.lock.acquire()
            self.operation = False
            self.lock.release()
        
    def serial_connect(self):
        retry_cnt = 0
        ret = False
        
        while True:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
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
    
    def publish_GNSS_msg(self):
        msg = GNSS()
        msg.header.stamp = rospy.Time.now()
        self.lock.acquire()
        data = self.GNSS_data
        msg.WGS84_origin.x = self.origin_longitude
        msg.WGS84_origin.y = self.origin_latitude
        msg.UTM_origin.x = self.origin_e
        msg.UTM_origin.y = self.origin_n
        msg.coordinate.x = self.rel_e
        msg.coordinate.y = self.rel_n
        msg.UTM.x = self.e
        msg.UTM.y = self.n        
        self.lock.release()
        
        msg.serial_no = data['serial_number']
        msg.date = data['date']
        msg.time = data['time']
        msg.mode = data['mode']
        msg.cn0 = data['cn0']
        msg.gps_valid = data['gps_valid']
        msg.battery = data['bat']
        msg.RSSI = data['RSSI']
        msg.WGS84_origin.z = 0.0
        msg.WGS84.x = data['longitude']
        msg.WGS84.y = data['latitude']
        msg.WGS84.z = data['altitude']
        msg.UTM_origin.z = 0.0
        msg.UTM.z = data['altitude']
        msg.coordinate.z = data['altitude']
        msg.velocity.x = data['velocity']['n']
        msg.velocity.y = data['velocity']['e']
        msg.velocity.z = data['velocity']['d']
        msg.acceleration.x = data['acceleration']['n']
        msg.acceleration.y = data['acceleration']['e']
        msg.acceleration.z = data['acceleration']['d']
        self.GNSS_pub.publish(msg)
    
    def publish_GNSS_odom_msg(self):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        
        self.lock.acquire()
        x = self.rel_e
        y = self.rel_n
        roll = self.GNSS_data['IMU']['roll']
        pitch = self.GNSS_data['IMU']['pitch']
        yaw = self.GNSS_data['IMU']['yaw']
        self.lock.release()
        odom_quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        # msg.pose.pose.orientation.z = yaw
        msg.pose.pose.orientation = Quaternion(*odom_quat)
        msg.child_frame_id = 'base_link'
        self.odom_pub.publish(msg)
        self.odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            rospy.Time.now(),
            "base_link",
            "odom"
        )
    
    def parse_GNSS_msg(self):
        self.rx_buffer += self.ser.readall()
        while len(self.rx_buffer) > MSG_LENGTH + 4:
            start_idx = -1
            search_range = len(self.rx_buffer) - MSG_LENGTH
            for i in range(search_range):
                if self.rx_buffer[i:i+4] == b'FITO':
                    start_idx = i+4
                    break
            if start_idx != -1:
                tmp_msg = self.rx_buffer[start_idx: start_idx + MSG_LENGTH]
                self.rx_buffer = self.rx_buffer[start_idx+MSG_LENGTH:]
                
                unpacked = serial_struct.unpack(tmp_msg)
                data = {
                    'serial_number': None,
                    'date': None,
                    'time': None,
                    'latitude': None,
                    'longitude': None,
                    'altitude': None,
                    'mode': None,
                    'cn0': None,
                    'IMU': {
                        'roll': None,
                        'pitch': None,
                        'yaw': None,    
                    },
                    'gps_valid': None,
                    'RSSI': None,
                    'Battery': None,
                    'acceleration': {
                        'n': None,
                        'e': None,
                        'd': None,
                    },
                    'velocity': {
                        'n': None,
                        'e': None,
                        'd': None
                    }
                }
                
                data['serial_number']       = unpacked[3].decode()
                year = unpacked[4]
                month = unpacked[5]
                day = unpacked[6]
                data['date']                = year * 10000 + month * 100 + day
                data['time']                = unpacked[7]
                data['mode']                = unpacked[9]
                data['cn0']                 = unpacked[10]
                data['RSSI']                = unpacked[11]
                data['battery']             = unpacked[12]
                data['gps_valid']           = unpacked[14]
                data['latitude']            = unpacked[21] * 1e-10
                data['longitude']           = unpacked[22] * 1e-10
                data["altitude"]            = unpacked[23] * 1e-3
                data['IMU']['roll']         = unpacked[25]
                data['IMU']["pitch"]        = unpacked[26]
                data['IMU']['yaw']          = unpacked[27]
                data['acceleration']['n']   = unpacked[28]
                data['acceleration']['e']   = unpacked[29]
                data['acceleration']['d']   = unpacked[30]
                data['velocity']['n']       = unpacked[31]
                data['velocity']['e']       = unpacked[32]
                data['velocity']['d']       = unpacked[33]
                                
                self.lock.acquire()
                if not self.GNSS_available:
                    self.GNSS_available = True
                self.GNSS_data = data
                if self.origin_latitude is None:
                    self.origin_latitude = data['latitude']
                    self.origin_longitude = data['longitude']
                    self.origin_e, self.origin_n = self.transformer.transform(self.origin_longitude, self.origin_latitude)
                self.lock.release()
            else:
                self.rx_buffer = self.rx_buffer[search_range:]

    def convert_coordinate(self):
        self.lock.acquire()
        lon = self.GNSS_data['longitude']
        lat = self.GNSS_data['latitude']
        self.lock.release()
        
        lon = lon // 1 + (lon % 1) * 100 / 60.0
        lat = lat // 1 + (lat % 1) * 100 / 60.0
        
        self.e, self.n = self.transformer.transform(lon, lat)
        if self.origin_e is None:
            self.origin_e = self.e
            self.origin_n = self.n
        
        self.GNSS_updated = False
        self.rel_e = self.e - self.origin_e
        self.rel_n = self.n - self.origin_n
        
    def serial_task(self):
        op = True
        print('[GNSS_SERIAL] Thread initiated.') 
        try:
            if self.serial_connect():
                while op:
                    self.parse_GNSS_msg()
                    
                    self.lock.acquire()
                    op = self.operation
                    self.lock.release()
                    self.serial_rate.sleep()
            else:
                print('[GNSS_SERIAL] ERROR - CONNECTION_FAILED.')
        finally:    
            try:
                self.ser.close()
            except: 
                pass
            print('[GNSS_SERIAL] Thread terminated')
    
    def convert_task(self):
        op = True 
        GNSS_available = False
        GNSS_updated = False
        print('[PROJECTION] Thread initiated.')
        try:
            while op:
                self.lock.acquire()
                GNSS_available = self.GNSS_available
                GNSS_updated = self.GNSS_updated
                op = self.operation
                self.lock.release()
                if GNSS_available:
                    if GNSS_updated:
                        self.convert_coordinate()
                    self.publish_GNSS_msg()
                
            self.convert_rate.sleep()
                
        finally:
            print('[PROJECTION] Thread terminated.')
            pass
    
    def run(self):
        try:
            self.serial_thread.start()
            self.convert_thread.start()
        finally:
            if self.serial_thread.is_alive():
                self.serial_thread.join()
            if self.convert_thread.is_alive():
                self.convert_thread.join()
        
if __name__ == '__main__':
    rospy.init_node('GNSS', anonymous=True)
    
    rp = rospkg.RosPack()
    root_pkg = 'gnss_ros'
    root_dir = rp.get_path('gnss_ros')
    PORT = '/dev/fitogether'
    BAUDRATE = 115200
    DISPLAY_RAWDATA = False
    ORIGIN = None
    
    if rospy.has_param('~GNSS_port'):
        PORT = rospy.get_param('~GNSS_port')
    if rospy.has_param('~GNSS_baudrate'):
        BAUDRATE = rospy.get_param('~GNSS_baudrate')
    if rospy.has_param('~display_rawdata'):
        DISPLAY_RAWDATA = rospy.get_param('~display_rawdata')
    if rospy.has_param('~origin_longitude'):
        ORIGIN_LON = rospy.get_param('~origin_longitude') # in degree
    if rospy.has_param('~origin_latitude'):
        ORIGIN_LAT = rospy.get_param('~origin_latitude')
    
    if ORIGIN_LON is not None or ORIGIN_LAT is not None:
        ORIGIN = (ORIGIN_LON, ORIGIN_LAT)
    
    node = GNSSnode(
        port=PORT,
        baudrate=BAUDRATE,
        display_rawdata=DISPLAY_RAWDATA,
        origin_coord=ORIGIN
    ) 
    
    node.run()