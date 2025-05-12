import os, sys
import rospy
from std_msgs.msg import Bool, UInt8, Float64
from geometry_msgs.msg import Twist
from irova_gnss_ros.msg import *

import cv2

import traceback
import datetime

import socket
import numpy as np
import pandas as pd
import pyproj
from rospkg import RosPack

import time
import threading

from queue import Queue
'''
Root package : irova_gnss_ros
Serial node: 
'''

rp = RosPack()
ROOT_PKG = 'irova_gnss_ros'
ROOT_PKG_DIR = rp.get_path(ROOT_PKG)

STANDALONE = True

UDP_CMD_TX_THREAD   = 1
UDP_CMD_RX_THREAD   = 2
UDP_DATA_THREAD     = 3
NMEA_THREAD         = 4
VISUALIZER_THREAD   = 5
LOCALIZER_THREAD    = 6
CONSOLE_THREAD      = 7
THREAD_LIST = [UDP_CMD_TX_THREAD, UDP_CMD_RX_THREAD, UDP_DATA_THREAD, NMEA_THREAD, 
               VISUALIZER_THREAD, LOCALIZER_THREAD]

HEIGHT = 500
WIDTH = 1200

def degmin2deg(degmin: float):
    deg = degmin // 100.0
    min = degmin  % 100.0
    
    return deg + min / 60.0

def deg2rad(deg):
    return deg * np.pi / 180.0 
 
 
class UDPReceiver:
    def __init__(self, ego_ip, udp_ip):
        self.ip_addr    = ego_ip
        self.UDP_IP     = udp_ip
        self.UDP_PORT   = 20000
        
        self.sock = None

    def run(self, NmeaBuffer: Queue, Operation: Queue):
        print('[UDP] Thread started.')
        
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind((self.UDP_IP, self.UDP_PORT))
            
            while True:
                try:
                    if Operation.qsize() != 0:
                        break
                    data, addr = sock.recvfrom(1024)
                    
                    if len(data) > 0: # Queue size never exceed 20
                        if NmeaBuffer.qsize() == 20:
                            _ = NmeaBuffer.get()
                        NmeaBuffer.put(data)
                    time.sleep(0.05)
                except Exception as e:
                    print('[UDP] Error: {}'.format(e))
                    Operation.put(False)
                    break
            
            print('[UDP] Thread terminated.')
            
            
class NMEAParser:
    def __init__(self, localization_update_interval=0.05, status_display_interval=1.0):
        self.nmea_byte_buffer           = bytearray()
        
        self.UTC                        = None
        self.latitude                   = None
        self.longitude                  = None
        self.altitude                   = 0.0
        self.n_flag                     = 'N'
        self.e_flag                     = 'E'
        self.gps_quality                = 'Position fix unavailable'
        self.mode                       = 'Unidentified'
        self.control_mode               = 'Unidentified'
        self.fix_mode                   = 'Fix not available'
        self.data_validity              = 'Data Not Valid'
        self.hdop                       = 0.0
        self.vdop                       = 0.0
        self.pdop                       = 0.0
        self.course                     = 0.0
        self.course_m                   = 0.0
        self.speed                      = 0.0
        self.speed_kmh                  = 0.0
        
        self.last_display_time          = 0.0
        self.status_display_interval    = status_display_interval
        
        self.update_interval            = localization_update_interval
        self.last_update_time           = 0.0

        self.status_display             = False

        self.rtk_status_update_flag     = False
        
    def string_chksum(self, string):
        val = 0
        for ch in string:
            val = val ^ ord(ch)
        return val
    
    def bytes_chksum(self, barray):
        val = 0x00
        for b in barray:
            val = val ^ b
        return val

    def search_nmea_message_packet(self, barray: bytearray):
        messages = []
        tmp_barray = self.nmea_byte_buffer + barray
        
        while True:
            start_idx   = tmp_barray.find(b'$')
            end_idx     = tmp_barray.find(b'\r\n')
            
            if start_idx != -1 and end_idx != -1:
                messages.append(tmp_barray[start_idx: end_idx])
                tmp_barray = tmp_barray[end_idx + 2:]
                continue
            else:
                break
        self.nmea_byte_buffer = barray
        return messages
        
    def parse_message_packet(self, barray: bytearray):
        header, chksum = barray.split(b'*')
        chksum_cmpr = self.bytes_chksum(header[1:])
        
        if chksum_cmpr == int(chksum.decode('utf-8'), 16):
            segments = header.split(b',')
            if segments[0].endswith(b'GGA'):
                self.parse_gga_msg(segments)
            elif segments[0].endswith(b'GLL'):
                self.parse_gll_msg(segments)
            elif segments[0].endswith(b'GSA'):
                self.parse_gsa_msg(segments)
            elif segments[0].endswith(b'GSV'):
                self.parse_gsv_msg(segments)
            elif segments[0].endswith(b'RMC'):
                self.parse_rmc_msg(segments)
            elif segments[0].endswith(b'VTG'):
                self.parse_vtg_msg(segments)
            else:
                pass
    
    def parse_gga_msg(self, segments: list):
        if segments[1] != b'':
            self.UTC            = self.convert_utc(segments[1])
        if segments[2] != b'':
            self.latitude       = degmin2deg(float(segments[2]))
        if segments[3] != b'':
            self.n_flag         = segments[3].decode('ascii')
        if segments[4] != b'':
            self.longitude      = degmin2deg(float(segments[4]))
        if segments[5] != b'':
            self.e_flag         = segments[5].decode('ascii')
        if segments[6] != b'':
            tmp_gps_quality     = self.check_gps_quality(segments[6])
            if tmp_gps_quality != self.gps_quality:
                self.rtk_status_update_flag = True
                self.gps_quality = tmp_gps_quality
        if segments[8] != b'':
            self.hdop           = float(segments[8])
        if segments[9] != b'':
            self.altitude       = float(segments[9])
    
    def parse_gll_msg(self, segments: list):
        if segments[1] != b'':
            self.latitude       = degmin2deg(float(segments[1]))
        if segments[2] != b'':
            self.n_flag         = segments[2].decode('ascii')
        if segments[3] != b'':
            self.longitude      = degmin2deg(float(segments[3]))
        if segments[4] != b'':
            self.e_flag         = segments[4].decode('ascii')
        if segments[5] != b'':
            self.UTC            = self.convert_utc(segments[5])
        if segments[6] != b'':
            self.data_validity  = self.check_data_validity(segments[6])
    
    def parse_gsa_msg(self, segments: list):
        # print(len(segments), segments)
        if segments[1] != b'':
            self.control_mode   = self.check_control_mode(segments[1])
        if segments[2] != b'':
            tmp_fix_mode        = self.check_fix_type(segments[2])
            if tmp_fix_mode != self.fix_mode:
                self.rtk_status_update_flag = True
                self.fix_mode = tmp_fix_mode
        if segments[4] != b'':
            self.pdop           = float(segments[4])
        if segments[5] != b'':
            self.hdop           = float(segments[5])
        if segments[6] != b'':
            self.vdop           = float(segments[6])
    
    def parse_gsv_msg(self, segments: list):
        pass
    
    def parse_rmc_msg(self, segments: list):
        # print(len(segments), segments)
        if segments[1] != b'':
            self.UTC            = self.convert_utc(segments[1])
        if segments[2] != b'':
            self.data_validity  = self.check_data_validity(segments[2])
        if segments[3] != b'':
            self.latitude       = degmin2deg(float(segments[3]))
        if segments[4] != b'':
            self.n_flag         = segments[4].decode('ascii')
        if segments[5] != b'':
            self.longitude      = degmin2deg(float(segments[5]))
        if segments[6] != b'':
            self.e_flag         = segments[6].decode('ascii')
        if segments[7] != b'':
            self.speed          = float(segments[7])
        if segments[8] != b'':
            self.course         = float(segments[8])
        if segments[12] != b'':
            self.mode           = self.check_algorithm_mode(segments[12])
    
    def parse_vtg_msg(self, segments: list):
        if segments[1] != b'':
            self.course         = float(segments[1])
        if segments[3] != b'':
            self.course_m       = float(segments[3])
        if segments[5] != b'':
            self.speed          = float(segments[5])
        if segments[7] != b'':
            self.speed_kmh      = float(segments[7])
        if segments[9] != b'':
            tmp_mode            = self.check_algorithm_mode(segments[9])
            if tmp_mode != self.mode:
                self.rtk_status_update_flag = True
                self.mode = tmp_mode
        
    def check_gps_quality(self, mark):
        if mark == b'0':
            return 'Position fix unavailable'
        elif mark == b'1':
            return 'Valid position fix (SPS mode)'
        elif mark == b'2':
            return 'Valid position fix (DGPS mode)'
        elif mark == b'4':
            return 'RTK fixed ambiguity solution'
        elif mark == b'5':
            return 'RTK floating ambiguity solution'
        elif mark == b'6':
            return 'Dead reckoning mode'
        elif mark == b'7':
            return 'Manual input mode (fixed position)'
        elif mark == b'8':
            return 'Simulation mode'
        elif mark == b'9':
            return 'WAAS (SBAS)'
        
    def check_data_validity(self, mark):
        if mark == b'A':
            return 'Valid'
        elif mark == b'V':
            return 'Data Not Valid'
        else:
            return 'Unidentified'
        
    def check_control_mode(self, mark):
        if mark == b'M':
            return 'Manual'
        elif mark == b'A':
            return 'Automatic'
        else:
            return 'Unidentified'
    
    def check_fix_type(self, mark):
        if mark == b'1':
            return 'Fix not available'
        elif mark == b'2':
            return '2D'
        elif mark == b'3':
            return '3D'
        else:
            return 'Unidentified'
    
    def check_algorithm_mode(self, mark):
        if mark == b'N':
            return 'Data not valid'
        elif mark == b'A':
            return 'Autonomous mode'
        elif mark == b'D':
            return 'Differential mode'
        elif mark == b'E':
            return 'Estimated(Dead-reckoning) mode'
        elif mark == b'F':
            return 'RTK-Floting'
        elif mark == b'R':
            return 'RTK-Fixed'
        elif mark == b'M':
            return 'Manual input'
        elif mark == b'P':
            return 'Precise'
        elif mark == b'S':
            return 'Simulator'
        else:
            return 'Unidentified'
    
    def update_rtk_status(self, RtkStatus: Queue):
        record = (self.gps_quality, self.mode, self.fix_mode)
        RtkStatus.put(record)

    def convert_utc(self, barray: bytearray):
        string = barray.decode('ascii')
        res = '{}:{}:{}'.format(string[:2], string[2:4], string[4:])
        return res
    
    def display_status(self):
        if time.time() - self.last_display_time >= self.status_display_interval:
            print('===================================================================')
            print('                           [NMEA] Status                           ')
            print('                                                                   ')
            print('UTC:            {}'.format(self.UTC))
            print('Longitude:      {:12.6f} deg {}, {:12.6f} deg {}'.format(self.latitude, self.n_flag, self.longitude, self.e_flag))
            print('Altitude:       {:.2f} m'.format(self.altitude))
            print('Fix mode:       {} ({})'.format(self.fix_mode, self.gps_quality))
            print('Mode:           {} ({})'.format(self.mode, self.control_mode))
            print('Course angle:   {} (magnetic: {})'.format(self.course, self.course_m))
            print('Speed:          {} knots ({} km/h)'.format(self.speed, self.speed_kmh))
            print('HDOP/VDOP/PDOP: {} / {} / {}'.format(self.hdop, self.vdop, self.pdop))
            print('Data validity:  {}'.format(self.data_validity))
            print('===================================================================')
            
            self.last_display_time = time.time()
            
    def update_localization_info(self, LocalizationInfoBuffer: Queue):
        if self.latitude is None or self.longitude is None:
            return False
        else:
            if time.time() - self.last_update_time >= self.update_interval:
                record = (self.UTC, self.longitude, self.e_flag, self.latitude, self.n_flag, self.fix_mode, self.altitude)
                if LocalizationInfoBuffer.qsize() == 1: # Always use the newest data
                    _ = LocalizationInfoBuffer.get()
                # print('NMEA')
                # print(record)
                LocalizationInfoBuffer.put(record)
    
    def run(self, 
            NmeaBuffer: Queue, 
            LocalizationInfoBuffer: Queue, 
            RtkStatus: Queue,
            Operation: Queue):
        print('[NMEA] Parsing thread started.')
        
        while True:
            if Operation.qsize() != 0:
                break
            
            tmp_q_size = NmeaBuffer.qsize()
            for _ in range(tmp_q_size):
                tmp_data = NmeaBuffer.get()
                messages = self.search_nmea_message_packet(tmp_data)
                
                for m in messages:
                    self.parse_message_packet(m)
            if self.rtk_status_update_flag:
                self.update_rtk_status(RtkStatus)
                self.rtk_status_update_flag = False

            self.update_localization_info(LocalizationInfoBuffer)

            if self.status_display:
                if self.latitude is not None:
                    if time.time() - self.last_display_time >= self.status_display_interval:
                        self.display_status()
            time.sleep(0.05)
            
        print('[NMEA] Parsing thread terminated.')


class Localizer:
    def __init__(self):
        self.EPSG       = 'epsg5179'
        self.transformer = None
        
        self.cluster_size = 2
        self.e_cluster  = []
        self.n_cluster  = []
        
        self.utc        = None
        self.longitude  = None
        self.e_flag     = 'E'
        self.latitude   = None
        self.n_flag     = 'N'
        self.altitude   = None
        self.fix_mode   = None
        
        self.origin_e   = None
        self.origin_n   = None
        self.origin_latitude = None
        self.origin_longitude = None
        
        # Relative coordinate to the origin
        self.prev_e     = None
        self.prev_n     = None
        self.prev_th    = None
        self.prev_ve    = None
        self.prev_vn    = None
        self.e          = None
        self.n          = None
        self.th         = None
        self.ve         = None
        self.vn         = None
        self.omega      = None
        self.est_e      = None
        self.est_n      = None
        self.est_th     = None
        
        self.prev_t     = None
        self.cur_t      = 'NO_UTC_INFO'
        self.delta_t    = None
        self.delta_e    = None
        self.delta_n    = None
        self.delta_th   = None

        self.update_interval = 0.5
        self.last_update = 0.0
        
        # Linear kalman filter
        if STANDALONE:
            self.initial_uncertainty = 1
            self.A = np.eye(2) # State matrix
            self.H = np.eye(2)
            self.P = np.eye(2, dtype=float) * self.initial_uncertainty**2 # Initial state covariance matrix
            self.Q = np.array([[0.025, 0],
                               [0, 0.025]])
            self.R = np.eye(2, dtype=float) * 0.025
            
        else:
            self.A = np.array([[1, 1, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 1],
                            [0, 0, 0, 1]], dtype=float)
            self.H = np.array([[1, 0, 0, 0],
                            [0, 0, 1, 0]])
            self.Q = 0.01 * np.eye(4, dtype=float)
            self.R = np.eye(4, dtype=float) * 0.0625
            self.P = 4 * np.eye(4, dtype=float)
            self.X = np.array([0, 0, 0, 0], dtype=float).T

        self.default_configuration()

    def default_configuration(self):
        # EPSG4326
        proj4_from_string = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs"
        
        if self.EPSG == 'epsg5179':
            self.COORD_SYS_NAME = "UTM-K (GRS80) - Naver map"
            proj4_to_string = "+proj=tmerc +lat_0=38 +lon_0=127.5 +k=0.9996 +x_0=1000000 +y_0=2000000 +ellps=GRS80 +units=m +no_defs"
        elif self.EPSG == 'epsg5181':
            self.COORD_SYS_NAME = "GRS80, falseX:40000, falseY:50000, - Central origin (Republic of korea) - Kakao map"
            proj4_to_string = "+proj=tmerc +lat_0=38 +lon_0=128 +k=0.9999 +x_0=400000 +y_0=600000 +ellps=GRS80 +towgs84=-115.80,474.99,674.11,1.16,-2.31,-1.63,6.43 +units=m +no_defs"
        elif self.EPSG == 'epsg5185':
            self.COORD_SYS_NAME = "GRS80, falseY:60000 - West origin (Republic of korea)"
            proj4_to_string = "+proj=tmerc +lat_0=38 +lon_0=125 +k=1 +x_0=200000 +y_0=600000 +ellps=GRS80 +units=m +no_defs"
        elif self.EPSG == 'epsg5186':
            self.COORD_SYS_NAME = "GRS80, falseY=60000 - East origin (Republic of korea)"
            proj4_to_string = "+proj=tmerc +lat_0=38 +lon_0=127 +k=1 +x_0=200000 +y_0=600000 +ellps=GRS80 +units=m +no_defs"
        elif self.EPSG == 'epsg5187':
            self.COORD_SYS_NAME = 'GRS80, falseY:60000 - Central origin (Republic of korea)'
            proj4_to_string = "+proj=tmerc +lat_0=38 +lon_0=129 +k=1 +x_0=200000 +y_0=600000 +ellps=GRS80 +units=m +no_defs"
    
        self.transformer = pyproj.transformer.Transformer.from_crs(proj4_from_string, proj4_to_string)
    
    def convert_lat_lon_to_x_y(self, localization_info, origin_buffer: Queue):
        "UTC/longitude/e_flag/latitude/n_flag/fix_mode/altitude"
        self.UTC        = localization_info[0]
        self.longitude  = localization_info[1]
        self.e_flag     = localization_info[2]
        self.latitude   = localization_info[3]
        self.n_flag     = localization_info[4]
        self.fix_node   = localization_info[5]
        self.altitude   = localization_info[6]
        # print('CONV')
        # print(localization_info)
        # print([localization_info[i] for i in range(len(localization_info))])
        if self.UTC == self.cur_t:
            return False
        e, n  = self.transformer.transform(self.longitude, self.latitude)
        # print(self.longitude, self.latitude, e, n)

        if self.origin_e is None:
            self.origin_e = e
            self.origin_n = n
            self.origin_longitude = self.longitude
            self.origin_e_flag = self.e_flag
            self.origin_latitude = self.latitude
            self.origin_n_flag = self.n_flag
            origin_info = (self.EPSG, 
                           self.origin_e, 
                           self.origin_n, 
                           self.origin_longitude, 
                           self.origin_e_flag, 
                           self.origin_latitude, 
                           self.origin_n_flag)
            origin_buffer.put(origin_info)
            # print('ORIG')
            # print(self.origin_e_flag)
            # print('Send: {}'.format(origin_info))
        
        rel_e = e - self.origin_e
        rel_n = n - self.origin_n
        
        self.e_cluster.append(rel_e)
        self.n_cluster.append(rel_n)
        
        if len(self.e_cluster) == self.cluster_size:
            mean_e = np.mean(self.e_cluster)
            mean_n = np.mean(self.n_cluster)
            
            self.prev_e = self.e
            self.prev_n = self.n
            
            self.e = mean_e # - self.origin_e
            self.n = mean_n # - self.origin_n
            
            self.prev_t = self.cur_t
            self.cur_t = self.UTC
            
            self.e_cluster = []
            self.n_cluster = []
            
            if self.prev_e is not None:
                self.compute_velocity()
            return True
        
        else:
            return False

    def compute_time_delta(self):
        prev_t_seg = self.prev_t.split(':')
        cur_t_seg = self.cur_t.split(':')
        
        prev_h = float(prev_t_seg[0])
        prev_m = float(prev_t_seg[1])
        prev_s = float(prev_t_seg[2].split('.')[0])
        prev_ms = float(prev_t_seg[2].split('.')[1])
        
        cur_h = float(cur_t_seg[0])
        cur_m = float(cur_t_seg[1])
        cur_s = float(cur_t_seg[2].split('.')[0])
        cur_ms = float(cur_t_seg[2].split('.')[1])
        dd = 0
        dh = cur_h - prev_h
        dm = cur_m - prev_m
        ds = cur_s - prev_s
        dms = cur_ms - prev_ms
        
        if dms < 0:
            dms = dms % 1.0
            ds -= 1
        if ds < 0:
            ds = ds % 60.0
            dm -= 1
        if dm < 0:
            dm = dm % 60.0
            dh -= 1
        if dh < 0:
            dd += 1
            dh = dh % 24
            
        self.delta_t = dd * 24 * 60 * 60 + \
                       dh * 60 * 60 + \
                       dm * 60 + \
                       ds + dms
        
    def compute_velocity(self):
        self.compute_time_delta()
        self.delta_e = self.e - self.prev_e
        self.delta_n = self.n - self.prev_n
        # print(self.prev_t, self.cur_t)
        # print(self.e, self.prev_e, self.n, self.prev_n)
        # print(self.delta_t, self.delta_e, self.delta_n)
        self.prev_th = self.th
        self.th = np.arctan2(self.delta_e, self.delta_n)
        
        self.prev_ve = self.ve
        self.prev_vn = self.vn
        self.ve = self.delta_e / self.delta_t
        self.vn = self.delta_n / self.delta_t
        
    def kalman_filter(self, marker_buffer: Queue, filter="LKF"):
        if not STANDALONE:
            if filter == 'LKF':
                X = np.array([self.prev_e, self.prev_ve, self.prev_n, self.prev_vn]).T # state
                Z = np.array([self.e, self.ve, self.n, self.vn]).T # Measurement
                
                Xp = self.A.dot(X)
                Pp = self.A.dot(self.P).dot(self.A.T) + self.Q
                
                K = Pp.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R))
                X = Xp + K.dot(Z - self.H.dot(Xp))
                self.P = Pp - K.dot(self.H).dot(Pp)
                
                self.est_e  = X[0]
                self.est_ve = X[1]
                self.est_n  = X[2]
                self.est_vn = X[3]
                
                self.est_th = np.arctan2(self.est_ve, self.est_vn)
            elif filter == 'EKF':
                pass
            elif filter == 'UKF':
                pass
        
        else:
            if self.prev_e is not None:
                if filter == 'LKF':
                    u = np.array([self.delta_e, self.delta_n]).T # delta motion
                    state = np.array([self.prev_e, self.prev_n]).T
                    tmp_state = self.A.dot(state) + u
                    Pp = self.A.dot(self.P).dot(self.A.T) + self.Q
                    
                    z = np.array([self.e, self.n]).T # Measurement
                    K = Pp.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R))
                    
                    est_state = tmp_state + K.dot(z - self.H.dot(tmp_state))
                    self.P = Pp - K.dot(self.H).dot(Pp)
                    
                    self.est_e = est_state[0]
                    self.est_n = est_state[1]
                    
                    delta_e_est = self.est_e - self.e
                    delta_n_est = self.est_n - self.n
                    self.est_th = np.arctan2(delta_n_est, delta_e_est)
                elif filter == 'EKF':
                    pass
                elif filter == 'UKF':
                    pass
                
        marker_record = ((self.est_e, self.est_n, self.est_th), (self.e, self.n, self.th), self.UTC)
        marker_buffer.put(marker_record)

    def dummy_filter(self, marker_buffer: Queue):
        now = datetime.datetime.now().strftime('%Y%m%d:%H.%M.%S.%f').split(':')[1].split('.')
        dummy_utc = '{}:{}:{}.{}'.format(now[0], now[1], now[2], now[3])
        marker_record = ((None, None, None,), (self.e, self.n, self.th), dummy_utc)
        marker_buffer.put(marker_record)
        # print(marker_record)
    
    def run(self, 
            LocalizationInfoBuffer: Queue, 
            MarkerBuffer: Queue, 
            OriginBuffer: Queue, 
            Operation: Queue):
        print('[LOCALIZER] Thread started.')
        
        while True:
            if Operation.qsize() != 0:
                break
            
            tmp_q_size = LocalizationInfoBuffer.qsize()
            for _ in range(tmp_q_size):
                tmp_data = LocalizationInfoBuffer.get()
                if self.convert_lat_lon_to_x_y(tmp_data, OriginBuffer):
                    if time.time() - self.last_update >= self.update_interval:
                        self.dummy_filter(MarkerBuffer)
                        self.last_update = time.time()
                    # self.kalman_filter(marker_buffer)
        
        print('[LOCALIZER] Thread terminated.')

class Visualizer:
    def __init__(self):
        self.map                    = None
        self.default_map            = None
        
        self.scale_step             = [0.0001, 0.0005, 0.001, 0.005, 0.01, 0.05, 0.1, 
                                       0.5, 1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]
        self.HEIGHT                 = 1000
        self.scale                  = self.scale_step[0]
        
        self.min_e                  = None
        self.max_e                  = None
        self.min_n                  = None
        self.max_n                  = None
        
        self.origin_e               = 0.0
        self.origin_n               = 0.0
        self.origin_latitude        = 0.0
        self.origin_longitude       = 0.0
        self.altitude               = 0.0

        self.origin_e_flag          = 'E'
        self.origin_n_flag          = 'N'
        
        self.n_point_to_display     = 10
        self.est_markers_to_display = []
        self.mea_markers_to_display = []
        
        self.whole_trace_flag       = False
        
        # trace
        self.est_markers            = []
        self.mea_markers            = []
        self.est_marker_chunk       = []
        self.mea_marker_chunk       = []
        
        self.trace_flag             = False
        self.trace_chunk_size       = 1000
        self.trace_record_index     = 0
        self.trace_chunk            = np.zeros((self.trace_chunk_size, 7), dtype=float) # UTC, Measurements(E, N, TH)
        self.trace_chunks           = []
        
        # RTK status
        self.gps_quality            = 'Unidentified'
        self.mode                   = 'Unidentified'
        self.fix_mode               = 'Fix not available'
        self.rtk_status_colors      = [(0, 0, 0), (0,0,255), (255,20,20), (20,255,20)]
        
    def convert_utc(self, string: str):
        h,m,s = string.strip(':')
        return float('{}{}{}'.format(h,m,s))
        
    def create_default_map(self):
        tmp_map = np.zeros((1000, 1000, 3), dtype=np.uint8)
        
        for i in range(1, 10):
            cv2.line(tmp_map, (i*100,0), (i*100,999), (150,150,150))
            cv2.line(tmp_map, (0,i*100), (999,i*100), (150,150,150))
            
        for i in range(10):
            cv2.circle(tmp_map, (500,500), (i+1)*100, (150,150,150), 1)
            
        cv2.line(tmp_map, (0,500), (999,500), (255,255,255), 1)
        cv2.line(tmp_map, (500,0), (500,999), (255,255,255), 1)
        cv2.circle(tmp_map, (500,500), 5, (255,255,255), -1)
        
        if 100 * self.scale <= 1e-2:
            unit = 'mm'
            factor = 1e4
        elif 100 * self.scale <= 1e2:
            unit = 'm'
            factor = 1e0
        else:
            unit = 'km'
            factor = 1e-4
        
        for i in range(4):
            cv2.putText(tmp_map, "{:.2f} {}".format((4-i) * 100 * self.scale * factor, unit), 
                        ( 35+i*100,           490), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
            cv2.putText(tmp_map, "{:.2f} {}".format((i+1) * 100 * self.scale * factor, unit), 
                        (610+i*100,           490), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
            cv2.putText(tmp_map, "{:.2f} {}".format((4-i) * 100 * self.scale * factor, unit), 
                        (      510, -10+(i+1)*100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
            cv2.putText(tmp_map, "{:.2f} {}".format((i+1) * 100 * self.scale * factor, unit), 
                        (      510,     590+i*100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
        
        # configuration info
        cv2.rectangle(tmp_map, (0,0), (400, 50), (210,210,210), -1)
        cv2.putText(tmp_map, 'Origin: {:9.6f} {}, {:10.6f} {}'.format(self.origin_latitude, 
                                                                      self.origin_n_flag, 
                                                                      self.origin_longitude, 
                                                                      self.origin_e_flag),
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1)
        cv2.putText(tmp_map, '       {:12.4f} m (E), {:12.4f} m (N)'.format(self.origin_e,
                                                                            self.origin_n),
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1)
        
        cv2.rectangle(tmp_map, (0, 980), (999, 999), (200,200,200),-1)
        if self.gps_quality == 'RTK floating ambiguity solution':
            c = self.rtk_status_colors[2]
        elif self.gps_quality == 'RTK fixed ambiguity solution':
            c = self.rtk_status_colors[3]
        elif self.gps_quality == 'Position fix unavailable':
            c = self.rtk_status_colors[1]
        else:
            c = self.rtk_status_colors[0]
        cv2.putText(tmp_map, "{}".format(self.gps_quality), (10, 995), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1)
        if self.mode == 'RTK-Floating':
            c = self.rtk_status_colors[2]
        elif self.mode == 'RTK-Fixed':
            c = self.rtk_status_colors[3]
        elif self.mode == 'Data not valid':
            c = self.rtk_status_colors[1]
        else:
            c = self.rtk_status_colors[0]
        cv2.putText(tmp_map, '{}'.format(self.mode),     (410, 995), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1)
        cv2.putText(tmp_map, '{}'.format(self.fix_mode), (710, 995), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)
        
        self.default_map = tmp_map.copy()
        
    def set_origin(self, origin_info):
        self.epsg               = origin_info[0]
        self.origin_e           = origin_info[1]
        self.origin_n           = origin_info[2]
        self.origin_longitude   = origin_info[3]
        self.origin_e_flag      = origin_info[4]
        self.origin_latitude    = origin_info[5]
        self.origin_n_flag      = origin_info[6]
        self.create_default_map()
        # print('RCV: {}'.format(origin_info))

    def update_rtk_status(self, tmp_rtk_status):
        self.gps_quality    = tmp_rtk_status[0]
        self.mode           = tmp_rtk_status[1]
        self.fix_mode       = tmp_rtk_status[2]

        self.create_default_map()

    def update_scale(self, tmp_markers):
        if tmp_markers[0] == (None, None, None):
            new_e = tmp_markers[1][0]
            new_n = tmp_markers[1][1]
        else:
            new_e = max(tmp_markers[0][0], tmp_markers[1][0])
            new_n = max(tmp_markers[0][1], tmp_markers[1][1])
        
        if self.min_e is None:
            self.min_e = new_e
            self.max_e = new_e
            self.min_n = new_n
            self.max_n = new_n
        else:
            self.min_e = min(new_e, self.min_e)
            self.max_e = max(new_e, self.max_e)
            self.min_n = min(new_n, self.min_n)
            self.max_n = max(new_n, self.max_n)
            
        max_delta = max([np.abs(self.min_e), np.abs(self.min_n), np.abs(self.max_e), np.abs(self.max_n)])
        
        scale = self.scale
        if max_delta >= self.scale_step[-1] * self.HEIGHT / 2:
            scale =1e6
        else:
            flag = False
            for i in range(len(self.scale_step)-1):
                if (max_delta < self.scale_step[-(i+1)] * self.HEIGHT / 2) and (max_delta >= self.scale_step[-(i+2)] * self.HEIGHT / 2):
                    scale = self.scale_step[-(i+1)]
                    flag = True
                    break
            if not flag:
                scale = self.scale_step[0]
        
        if self.scale != scale:
            self.scale = scale
            
            self.create_default_map()
        
    def append_point(self, tmp_markers):
        # (est_e, est_n, est_th), (e, n, th)
        if len(self.mea_markers_to_display) == self.n_point_to_display:
            del self.mea_markers_to_display[0]
        self.mea_markers_to_display.append(tmp_markers[1])
        
        if not STANDALONE:
            if len(self.est_markers_to_display) == self.n_point_to_display:
                del self.est_markers_to_display[0]
            self.est_markers.append(tmp_markers[0])
        
    def draw_markers(self):
        tmp_map = self.default_map.copy()
        length = len(self.mea_markers_to_display)
        prev_p_mea = None
        
        for i in range(length):
            alpha = (1 - (length-i)/(length+1))
            p_mea = self.mea_markers_to_display[i]
            if prev_p_mea is not None:
                cv2.line(tmp_map, (int(prev_p_mea[0]/self.scale)+500, 500-int(prev_p_mea[1]/self.scale)),
                         (int(p_mea[0]/self.scale)+500, 500-int(p_mea[1]/self.scale)),
                         (0,int(255*alpha),0), 2)
            prev_p_mea = p_mea
            cv2.circle(tmp_map, (int(p_mea[0]/self.scale)+500, 500-int(p_mea[1]/self.scale)),
                       5,
                       (0,0,int(255*alpha)),1)
        
        if not STANDALONE:
            length = len(self.est_markers_to_display)
            prev_p_est = None    
            for i in range(length):
                alpha = (1 - (length-i)/(length+1))
                p_est = self.est_markers_to_display[i]
                cv2.circle(tmp_map, (int(p_est[0]/self.scale)+500, 500-int(p_est[1]/self.scale)),
                           (0, int(255*alpha), int(155*alpha)), 1)
            prev_p_est = p_est
            cv2.rectangle(tmp_map, (600, 0), (1000, 50), (210, 210, 210), -1)    
            cv2.putText(tmp_map, "Measurement: {:.2f} m, {:.2f} m ({}, {})".format(prev_p_est[0], prev_p_est[1], self.origin_e_flag, self.origin_n_flag),
                        (610, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1)
        else:
            cv2.rectangle(tmp_map, (600, 0), (1000,50), (210, 210, 210), -1)
        
        # print(prev_p_mea)
        if prev_p_mea is not None:
            cv2.putText(tmp_map, "Measurement: {:.2f} m, {:.2f} m ({}, {})".format(prev_p_mea[0], 
                                                                                   prev_p_mea[1], 
                                                                                   self.origin_e_flag, 
                                                                                   self.origin_n_flag),
                        (610, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1)
        else:
            cv2.putText(tmp_map, "Measurement: - m, - m ({}, {})".format(self.origin_e_flag, 
                                                                         self.origin_n_flag),
                        (610, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1)
        
        self.map = tmp_map.copy()
            
    def run(self, 
            MarkerBuffer: Queue, 
            OriginBuffer: Queue, 
            RtkStatus: Queue, 
            Operation: Queue):
        print('[VIS] Thread started.')
        
        self.create_default_map()
        
        while True:
            if Operation.qsize() != 0:
                break
            
            if RtkStatus.qsize() != 0:
                tmp_rtk_status = RtkStatus.get()
                self.update_rtk_status(tmp_rtk_status)

            if OriginBuffer.qsize() != 0:
                tmp_origin_data = OriginBuffer.get()
                self.set_origin(tmp_origin_data)

            tmp_q_length = MarkerBuffer.qsize()
            
            for _ in range(tmp_q_length):
                tmp_markers = MarkerBuffer.get()
                # print(tmp_markers)
                if self.trace_flag:
                    self.trace_chunk[self.trace_record_index,1] = self.convert_utc(tmp_markers[2])
                    self.trace_chunk[self.trace_record_index,1:] = tmp_markers[1]
                    if tmp_markers == (None, None, None):
                        self.trace_chunk[self.trace_record_index,4:] = tmp_markers[1]
                    else:
                        self.trace_chunk[self.trace_record_index,4:] = tmp_markers[2]
                    if self.trace_record_index + 1 == self.trace_chunk_size:
                        self.trace_chunks.append(self.trace_chunk.copy())
                        self.trace_chunk *= 0.0
                        self.trace_record_index = 0
                self.append_point(tmp_markers)
                self.update_scale(tmp_markers)
            self.draw_markers()
            
            cv2.imshow('res', self.map)
            
            key = cv2.waitKey(10) 
            if key == ord('q'):
                Operation.put(False)
                break
            
            time.sleep(0.1)
            
        try:
            cv2.destroyAllWindows()
        except:
            pass
        print('[VIS] Thread terminated.')
        
        
class RemoteControl:
    def __init__(self, ego_ip, udp_ip):
        self.EGO_IP             = ego_ip
        self.UDP_IP             = udp_ip
        self.UDP_PORT_TX        = 20003
        self.UDP_PORT_RX        = 20002
        
        self.tx_message         = "No Message"
        self.rx_message         = "No Message"
        
        self.operation_chk_msg  = b'ROS remote command: Connection check'
        self.launch_msg         = b'ROS remote command: launch'
        self.stop_msg           = b'ROS remote command: stop'
        self.shutdown_msg       = b'ROS remote command: system shutdown'
        self.reboot_msg         = b'ROS remote command: system reboot'
        
        self.connection_resp    = b'ROS robot remote: Connection check'
        self.disconnection_resp = b'ROS robot remote: Disconnected'
        self.launch_resp        = b'ROS robot remote: launch'
        self.stop_resp          = b'ROS robot remote: stop'
        self.shutdown_resp      = b'ROS robot remote: system shutdown'
        self.reboot_resp        = b'ROS robot remote: system reboot'
        
        self.__rate = rospy.Rate(100)
        
        self.cmd_vel_lin = 0.0
        self.cmd_vel_ang = 0.0
        
        self.v_lin_max = 0.7
        self.v_lin_min = 0.0
        self.v_ang_min = deg2rad(-30)
        self.v_ang_max = deg2rad(30)
        self.v_lin_step = 0.02
        self.v_ang_step = deg2rad(5)
        
        self.console_default_image = None
        self.console_image = None
        
        self.thread_ready = [False for _ in range(6+1)]
        self.thread_ready[-1] = True # console thread
        
        self.remote_control_topic = '/cmd_vel'
        
        self.cmd_vel_pub = rospy.Publisher(self.remote_control_topic, 
                                           Twist, queue_size=10)
        
        self.command_list = ['1. Launch robot',
                             '2. Shutdown ROS',
                             '3. System reboot',
                             '4. System shutdown']
        
        self.control_instruction = '''
        [Moving around]
            w           w/x: accel/decelerate
        a   s   d       a/d: turn left/right
            x           space, s: force stop

                        q: Quit program
        '''
        
    def constrain(self, input_vel, lower_b, upper_b):
        if input_vel < lower_b:
            input_vel = lower_b
        elif input_vel > upper_b:
            input_vel = upper_b
        else:
            input_vel = input_vel
        
        return input_vel
    
    def check_linear_limit_velocity(self, v):
        return self.constrain(v, self.v_lin_min, self.v_lin_max)
    
    def check_angular_limit_velocity(self, w):
        return self.constrain(w, self.v_ang_min, self.v_ang_max)
    
    def publish_cmd_vel_msg(self):
        msg = Twist()
        msg.linear.x = self.cmd_vel_lin
        msg.angular.z = self.cmd_vel_ang
        self.cmd_vel_pub.publish(msg)
    
    def publish_command_message(self, key, Command:Queue, Operation: Queue):
        ret = True
        if key == ord('w'):
            self.cmd_vel_lin = self.check_linear_limit_velocity(self.cmd_vel_lin + self.v_lin_step)
            self.publish_cmd_vel_msg()
        elif key == ord('s'):
            self.cmd_vel_lin = self.check_linear_limit_velocity(self.cmd_vel_lin - self.v_lin_step)
            self.publish_cmd_vel_msg()
        elif key == ord('a'):
            self.cmd_vel_ang = self.check_angular_limit_velocity(self.cmd_vel_ang + self.v_ang_step)
            self.publish_cmd_vel_msg()
        elif key == ord('d'):
            self.cmd_vel_ang = self.check_angular_limit_velocity(self.cmd_vel_ang - self.v_ang_step)
            self.publish_cmd_vel_msg()
        elif key in [ord(' '), ord('s')]:
            self.cmd_vel_lin = 0.0
            self.cmd_vel_ang = 0.0
            self.publish_cmd_vel_msg()
            
        elif key == ord('1'):
            Command.put(1)
        elif key == ord('2'):
            Command.put(2)
        elif key == ord('3'):
            Command.put(3)
        elif key == ord('4'):
            Command.put(4)
        
        elif key == ord('q'):
            Operation.put(False)
            ret = False
            
        return ret
    
    def draw_default_console_image(self):
        host_ip = self.EGO_IP
        udp_ip = self.UDP_IP
        tx_port = self.UDP_PORT_TX
        rx_port = self.UDP_PORT_RX
        
        tmp_img = np.ones((230, 1200, 3), dtype=np.uint8) * 20
        line_color=(200,200,200)
        string_color=(220,220,220)
        msg_color=(20,200,10)
        neg_color=(20,20,200)
        
        WIDTH, HEIGHT = (500, 1200)
        
        cv2.line(tmp_img, (0,        40), (WIDTH,        40), line_color, 1)
        cv2.line(tmp_img, (0,       140), (WIDTH,       140), line_color, 1)
        cv2.line(tmp_img, (0,HEIGHT-120), (WIDTH,HEIGHT-120), line_color, 1)
        cv2.line(tmp_img, (0,HEIGHT- 90), (WIDTH,HEIGHT- 90), line_color, 1)
        cv2.line(tmp_img, (0,HEIGHT- 60), (WIDTH,HEIGHT- 60), line_color, 1)
        cv2.line(tmp_img, (0,HEIGHT- 30), (WIDTH,HEIGHT- 30), line_color, 1)
        
        cv2.putText(tmp_img, 'Server IP: {}'.format(host_ip),   (  5,25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, string_color, 1)
        cv2.putText(tmp_img, 'UDP IP: {}'.format(udp_ip),       (220,25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, string_color, 1)
        cv2.putText(tmp_img, 'UDP data port: {}'.format(20000), (420,25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, string_color, 1)
        cv2.putText(tmp_img, 'UDP TX port: {}'.format(tx_port), (640,25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, string_color, 1)
        cv2.putText(tmp_img, 'UDP RX port: {}'.format(rx_port), (860,25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, string_color, 1)

        cv2.line(tmp_img, ( 210, 0), ( 210, 40), line_color, 1)
        cv2.line(tmp_img, ( 410, 0), ( 410, 40), line_color, 1)
        cv2.line(tmp_img, ( 630, 0), ( 630, 40), line_color, 1)
        cv2.line(tmp_img, ( 850, 0), ( 850, 40), line_color, 1)
        cv2.line(tmp_img, (1070, 0), (1070, 40), line_color, 1)

        cv2.putText(tmp_img, 'Command list',                   ( 15,  60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, string_color, 1)
        cv2.putText(tmp_img, '1. Launch robot',                ( 85,  85), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        cv2.putText(tmp_img, '2. Shutdown ROS',                (635,  85), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        cv2.putText(tmp_img, '3. System reboot',               ( 85, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        cv2.putText(tmp_img, '4. System shutdown',             (635, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        
        cv2.putText(tmp_img, '[Moving around]',                (370,180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, string_color, 2)
        cv2.putText(tmp_img, '      w        ',                (250,220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, string_color, 2)
        cv2.putText(tmp_img, 'a     s       d',                (250,270), cv2.FONT_HERSHEY_SIMPLEX, 0.8, string_color, 2)
        cv2.putText(tmp_img, '      x        ',                (250,320), cv2.FONT_HERSHEY_SIMPLEX, 0.8, string_color, 2)
        cv2.putText(tmp_img, 'w/x : Accelerate / Decelerate',  (550, 220), cv2.FONT_HERSHET_SIMPLEX, 0.5, string_color, 1)
        cv2.putText(tmp_img, 'a/d : Turn Left / Right',        (550, 260), cv2.FONT_HERSHET_SIMPLEX, 0.5, string_color, 1)
        cv2.putText(tmp_img, 'space, s: Force stop',           (550, 300), cv2.FONT_HERSHET_SIMPLEX, 0.5, string_color, 1)
        cv2.putText(tmp_img, 'q : Quit',                       (550, 350), cv2.FONT_HERSHET_SIMPLEX, 0.5, string_color, 1)

        cv2.putText(tmp_img, "Sent",                           (5, HEIGHT-100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, string_color, 1)
        cv2.putText(tmp_img, "Received",                       (5, HEIGHT- 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, string_color, 1)
        
        cv2.line(tmp_img, (90, HEIGHT-90), (90, HEIGHT-120), line_color, 1)
        cv2.line(tmp_img, (90, HEIGHT-60), (90, HEIGHT- 90), line_color, 1)


        cv2.putText(tmp_img, 'GNSS data thread',     ( 35,HEIGHT-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        cv2.putText(tmp_img, 'NMEA parser thread',   (335,HEIGHT-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        cv2.putText(tmp_img, 'Visualization thread', (635,HEIGHT-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        cv2.putText(tmp_img, 'Console thread',       (935,HEIGHT-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        
        cv2.putText(tmp_img, 'Localization thread',  ( 35,HEIGHT-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        cv2.putText(tmp_img, 'CMD TX thread',        (335,HEIGHT-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        cv2.putText(tmp_img, 'CMD RX thread',        (635,HEIGHT-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)
        cv2.putText(tmp_img, 'Robot connection',     (935,HEIGHT-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, string_color, 1)

        cv2.rectangle(tmp_img, (  6, HEIGHT-54), ( 24,HEIGHT-36), neg_color, -1)
        cv2.rectangle(tmp_img, (306, HEIGHT-54), (324,HEIGHT-36), neg_color, -1)
        cv2.rectangle(tmp_img, (606, HEIGHT-54), (624,HEIGHT-36), neg_color, -1)
        cv2.rectangle(tmp_img, (906, HEIGHT-54), (924,HEIGHT-36), neg_color, -1)
        
        cv2.rectangle(tmp_img, (  6, HEIGHT-24), ( 24,HEIGHT- 6), neg_color, -1)
        cv2.rectangle(tmp_img, (306, HEIGHT-24), (324,HEIGHT- 6), neg_color, -1)
        cv2.rectangle(tmp_img, (606, HEIGHT-24), (624,HEIGHT- 6), neg_color, -1)
        cv2.rectangle(tmp_img, (906, HEIGHT-24), (924,HEIGHT- 6), neg_color, -1)
        
    
        self.console_default_image = tmp_img.copy()
        
    def udp_tx_task(self, TxMSG: Queue, Command: Queue, ThreadReady: Queue, 
                    RemoteMachineReady: Queue, Operation: Queue):
        UDP_IP = self.UDP_IP
        TX_PORT = self.UDP_PORT_TX
        remote_machine_ready = False
        
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            
            ThreadReady.put(UDP_CMD_TX_THREAD)
            
            while True:
                if Operation.qsize() != 0:
                    break

                tmp_q_size = RemoteMachineReady.qsize()

                for _ in range(tmp_q_size):
                    tmp_n = RemoteMachineReady.get()
                    print(tmp_n)
                    if tmp_n == 1:
                        remote_machine_ready = True
                        
                    elif tmp_n == -1:
                        remote_machine_ready = False
                        
                    else:
                        RemoteMachineReady.put(tmp_n)
                        
                if not remote_machine_ready:
                    # Request connection check msg
                    TxMSG.put(self.operation_chk_msg)
                    sock.sendto(self.operation_chk_msg, (UDP_IP, TX_PORT))
                    time.sleep(0.5)

                else:
                    tmp_msg = None
                    if Command.qsize() != 0:
                        tmp_cmd = Command.get()
                        if tmp_cmd == 1:
                            tmp_msg = self.launch_msg
                        elif tmp_cmd == 2:
                            tmp_msg = self.stop_msg
                        elif tmp_cmd == 3:
                            tmp_msg = self.reboot_msg
                        elif tmp_cmd == 4:
                            tmp_msg = self.shutdown_msg
                            
                        if tmp_msg is not None:
                            TxMSG.put(tmp_msg)
                            sock.sendto(tmp_msg, (UDP_IP, TX_PORT))
                            
                    time.sleep(0.1)
            
            sock.close()
            ThreadReady.put(-UDP_CMD_TX_THREAD)
        print('TX task terminated.')
        
    def validate_msg(self, msg: bytes, Response: Queue, RemoteMachineReady: Queue):
        if msg == self.connection_resp:
            print(RemoteMachineReady.qsize())
            if RemoteMachineReady.qsize() == 0:
                RemoteMachineReady.put(1)
                RemoteMachineReady.put(2)
        elif msg == self.disconnection_resp:
            if RemoteMachineReady.qsize() == 0:
                RemoteMachineReady.put(-1)
                RemoteMachineReady.put(-2)
        
        Response.put(msg)
        
    def udp_rx_task(self, Response: Queue, RemoteMachineReady: Queue, 
                    ThreadReady: Queue, Operation: Queue):
        EGO_IP = self.EGO_IP
        UDP_IP = self.UDP_IP
        RX_PORT = self.UDP_PORT_RX
        
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind((UDP_IP, RX_PORT))
            sock.settimeout(0.5)
            
            ThreadReady.put(UDP_CMD_RX_THREAD)
            
            while True:
                try:
                    if Operation.qsize() != 0:
                        break
                    
                    data, addr = sock.recvfrom(1024)
                    
                    if len(data) != 0:
                        if addr[0] != EGO_IP:
                            self.validate_msg(data, Response, RemoteMachineReady)
                
                    time.sleep(0.5)
                except socket.timeout:
                    continue
            
            sock.close()
            ThreadReady.put(-UDP_CMD_RX_THREAD)
        print('RX task terminated.')
        
    def update_console_image(self, TxMSG, Response, thread_ready, 
                             remote_machine_ready):
        img = self.console_default_image.copy()
        
        msg_color=(20,200,10)
        pos_color=(20,200,20)
        neg_color=(20,20,200)
        
        for i, attrib in enumerate([UDP_DATA_THREAD, NMEA_THREAD, VISUALIZER_THREAD, CONSOLE_THREAD,
                                    LOCALIZER_THREAD, UDP_CMD_TX_THREAD, UDP_CMD_RX_THREAD]):
            x = int(i // 4)
            y = int(i  % 4)
            if thread_ready[attrib]:
                cv2.putText(img, (6 + 300*x, HEIGHT - (1-y)*30 - 24), (24 + 300*x, HEIGHT(1-y)*30 - 6), pos_color, -1)
            else:
                cv2.putText(img, (6 + 300*x, HEIGHT - (1-y)*30 - 24), (24 + 300*x, HEIGHT(1-y)*30 - 6), neg_color, -1)
        
        if TxMSG.qsize() != 0:
            self.tx_message = TxMSG.get().decode('ascii')
        cv2.putText(img, '{}'.format(self.tx_message), (110,160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, msg_color, 1)
        if Response.qsize() != 0:
            self.rx_message = Response.get().decode('ascii')
        cv2.putText(img, '{}'.format(self.rx_message), (110,190), cv2.FONT_HERSHEY_SIMPLEX, 0.5, msg_color, 1)
        
        if remote_machine_ready:
            cv2.rectangle(img, (906,HEIGHT-24), (924,HEIGHT-6), pos_color, -1)
        else:
            cv2.rectangle(img, (906,HEIGHT-24), (924,HEIGHT-6), neg_color, -1)
            
        self.console_image = img.copy()
        
    def update_termination_image(self, thread_ready, remote_machine_ready):
        img = self.console_image.copy()
        rect_color = (50,50,50)
        pos_color=(20,200,20)
        neg_color=(20,20,200)
        string_color=(220,220,220)
        
        ret = True
        
        cv2.rectangle(img, (40,40), (WIDTH-40,HEIGHT-40), rect_color, -1)
        cv2.putText(img, 'GNSS data thread :        ', (310, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.7, string_color, 2)
        cv2.putText(img, 'NMEA parser thread :      ', (310,135), cv2.FONT_HERSHEY_SIMPLEX, 0.7, string_color, 2)
        cv2.putText(img, 'Visualization thread :    ', (310,185), cv2.FONT_HERSHEY_SIMPLEX, 0.7, string_color, 2)
        cv2.putText(img, 'Console thread :          ', (310,235), cv2.FONT_HERSHEY_SIMPLEX, 0.7, string_color, 2)
        cv2.putText(img, 'Localization thread :     ', (310,285), cv2.FONT_HERSHEY_SIMPLEX, 0.7, string_color, 2)
        cv2.putText(img, 'CMD Tx thread :           ', (310,335), cv2.FONT_HERSHEY_SIMPLEX, 0.7, string_color, 2)
        cv2.putText(img, 'CMD Rx thread :           ', (310,385), cv2.FONT_HERSHEY_SIMPLEX, 0.7, string_color, 2)
        cv2.putText(img, 'Robot connection:         ', (180,435), cv2.FONT_HERSHEY_SIMPLEX, 0.7, string_color, 2)

        for i, attrib in enumerate(THREAD_LIST):
            if thread_ready[attrib]:
                cv2.putText(img, 'Running', (590, 85 + i*50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, neg_color, 2)
            else:
                cv2.putText(img, 'Running', (590, 85 + i*50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, pos_color, 2)
            
        if remote_machine_ready:
            cv2.putText(img, 'Connected',    (590,385), cv2.FONT_HERSHEY_SIMPLEX, 0.7, pos_color, 2)
        else:
            cv2.putText(img, 'disconnected', (590,385), cv2.FONT_HERSHEY_SIMPLEX, 0.7, neg_color, 2)
        
        self.console_image = img.copy()
        
        return ret
    
    def thread_ready_check(self, ThreadReady: Queue):
        if ThreadReady.qsize() != 0:
            tmp = ThreadReady.get()
            
            if abs(tmp) in THREAD_LIST:
                self.thread_ready[abs(tmp)] = (abs(tmp) == tmp)
                
    def remote_machine_connection_chk(self, RemoteMachineReady: Queue):
        tmp_q_size = RemoteMachineReady.qsize()
        
        for _ in range(tmp_q_size):
            tmp_n = RemoteMachineReady.get()
            if tmp_n == 2:
                remote_machine_ready = True
            elif tmp_n == -2:
                remote_machine_ready = False
            else:
                RemoteMachineReady.put(tmp_n)
                
        return remote_machine_ready
            
    def console_task(self, TxMSG: Queue, Command: Queue, Response: Queue, 
                     RemoteMachineReady: Queue, ThreadReady: Queue, 
                     Operation: Queue):
        thread_ready = [False, False]
        remote_machine_ready = False
        
        self.draw_default_console_image()

        while True:
            if Operation.qsize() != 0:
                break
            
            self.thread_ready_check(ThreadReady)
            if RemoteMachineReady.qsize() != 0:
                remote_machine_ready = self.remote_machine_connection_chk(RemoteMachineReady)
                
            self.update_console_image(TxMSG, Response, self.thread_ready, remote_machine_ready)
            
            cv2.imshow('Console', self.console_image)
            
            key = cv2.waitKey(10) & 0xFF
            
            
            if not self.publish_command_message(self, key, Command, Operation):
                break
            
        # Termination sequence
        start_time = time.time()
        while time.time() - start_time <= 5.0:
            self.thread_ready_check(ThreadReady)
            
            ret = self.update_termination_image(thread_ready, remote_machine_ready)
            
            cv2.imshow('Console', self.console_image)
            
            key = cv2.waitKey(50)
            if ret:
                break
        
        cv2.destroyAllWindows()
        
        
class GNSS:
    def __init__(self, log=False):
        self.LOG_ENABLE             = log
        self.upd_receiver           = None
        self.nmea_parser            = None
        self.visualizer             = None
        self.localizer              = None
        self.controller             = None
        
        self.NmeaBuffer             = Queue()
        self.LocalizationInfoBuffer = Queue()
        self.MarkerBuffer           = Queue()
        self.OriginBuffer           = Queue()
        self.RtkStatus              = Queue()
        
        self.RemoteMachineReady     = Queue()
        self.Command                = Queue()
        self.Response               = Queue()
        self.TxMSG                  = Queue()
        self.Threadready            = Queue()
        self.Operation              = Queue()
        self.LogBuffer              = Queue()
        
        self.bringup_console_thread = None
        self.udp_receiver_thread    = None
        self.nmea_parser_thread     = None
        self.visualizer_thread      = None
        self.localizer_thread       = None
        self.control_tx_thread      = None
        self.control_rx_thread      = None
        self.log_thread             = None
        
        self.log_max_line           = 2000
        self.log_line_idx           = 0
        self.log_chunk              = np.zeros(())
        self.log_chunks             = []
        self.attributes             = ['UTC','LAT','LON','']
        self.log_save_path          = ''
        
        # Shared memories and arguments
        self.bringup_console_thread_args    = (self.TxMSG, self.Command, self.Response,
                                               self.RemoteMachineReady, self.Threadready,
                                               self.Operation, )
        self.udp_receiver_thread_args       = (self.NmeaBuffer, self.Operation, ) # chk
        self.nmea_parser_thread_args        = (self.NmeaBuffer, self.LocalizationInfoBuffer, 
                                               self.RtkStatus, self.Operation, ) # chk
        self.visualizer_thread_args         = (self.MarkerBuffer, self.OriginBuffer,
                                               self.RtkStatus, self.Operation, )
        self.localizer_thread_args          = (self.LocalizationInfoBuffer, self.MarkerBuffer,
                                               self.OriginBuffer, self.Operation, )
        self.control_tx_thread_args         = (self.TxMSG, self.Command, self.Threadready,
                                               self.RemoteMachineReady, self.Operation, )
        self.control_rx_thread_args         = (self.Response, self.RemoteMachineReady,
                                               self.Threadready, self.Operation, )
        
        self.log_thread_args                = (self.LogBuffer, )
        
    def raise_error(self, exception):
        exc_type, exc_obj, exc_tb = sys.exc_info()
        
        if exc_tb is not None:
            line_no = exc_tb.tb_lineno
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        else:
            exc_type = ''
            line_no = ''
            fname = ''
            
        rospy.logerr(exception)
        print(exc_type, fname, line_no)
        print(traceback.format_exc())
    
    def default_configuration(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        self.ip_addr = sock.getsockname()[0]
        sock.close()
        
        seg = self.ip_addr.split('.')
        
        self.UDP_IP = '{}.{}.{}.255'.format(seg[0], seg[1], seg[2])
        self.udp_port = 20000
        
        self.udp_receiver           = UDPReceiver(ego_ip=self.ip_addr, udp_ip=self.UDP_IP)
        self.nmea_parser            = NMEAParser()
        self.visualizer             = Visualizer()
        self.localizer              = Localizer()
        self.controller             = RemoteControl(ego_ip=self.ip_addr, udp_ip=self.UDP_IP)

        self.bringup_console_thread = threading.Thread(target=self.controller.console_task, 
                                                       args=self.bringup_console_thread_args)
        self.udp_receiver_thread    = threading.Thread(target=self.udp_receiver.run,
                                                       args=self.udp_receiver_thread_args)
        self.nmea_parser_thread     = threading.Thread(target=self.nmea_parser.run,
                                                       args=self.nmea_parser_thread_args)
        self.visualizer_thread      = threading.Thread(target=self.visualizer.run,
                                                       args=self.visualizer_thread_args)
        self.localizer_thread       = threading.Thread(target=self.localizer.run,
                                                       args=self.localizer_thread_args)
        self.control_tx_thread      = threading.Thread(target=self.controller.udp_tx_task,
                                                       args=self.control_tx_thread_args)
        self.control_rx_thread      = threading.Thread(target=self.controller.udp_rx_task,
                                                       args=self.control_rx_thread_args)
        
        if self.LOG_ENABLE:
            self.log_thread         = threading.Thread(target=self.log_task, 
                                                       args=self.log_thread_args)
            
    def update_log(self, tmp_log):
        # Pandas column append check required -> NMEA?
        
        self.log_chunk[self.log_line_idx, :] = tmp_log
        self.log_line_idx += 1
        
        if self.log_line_idx == self.log_max_line:
            self.log_chunks.append(pd.DataFrame(self.log_chunk.copy(), columns=self.attributes))
            self.log_line_idx = 0
            self.log_chunk *= 0.0
            
    def log_task(self, LogBuffer: Queue, Operation: Queue):
        try:
            while True:
                if Operation.qsize() != 0:
                    break
                
                tmp_log_len = LogBuffer.qsize()
                
                for _ in range(tmp_log_len):
                    self.update_log(LogBuffer.get())
                
        finally:
            # Save function here
            pass
        
    def run(self):
        self.default_configuration()
        
        try:
            self.bringup_console_thread.start()
            self.udp_receiver_thread.start()
            self.nmea_parser_thread.start()
            self.visualizer_thread.start()
            self.localizer_thread.start()
            self.control_tx_thread.start()
            self.control_rx_thread.start()
        
        except Exception as e:
            print('[SYSTEM] Error: {}'.format(e))
            self.Operation.put(False)
            
        finally:
            if self.udp_receiver_thread.is_alive():
                self.udp_receiver_thread.join()
            if self.nmea_parser_thread.is_alive():
                self.nmea_parser_thread.join()
            if self.visualizer_thread.is_alive():
                self.visualizer_thread.join()
            if self.localizer_thread.is_alive():
                self.localizer_thread.join()
            if self.control_tx_thread.is_alive():
                self.control_tx_thread.join()
            if self.control_rx_thread.is_alive():
                self.control_rx_thread.join()
            time.sleep(1.0)
            if self.bringup_console_thread.is_alive():
                self.bringup_console_thread.join()
        
        
if __name__ == '__main__':
    rospy.init_node('GNSS_ZED_F9P')
    
    node = GNSS()
    
    node.run()