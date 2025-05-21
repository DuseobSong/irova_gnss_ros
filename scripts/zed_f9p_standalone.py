#!/usr/bin/env python3

import os, sys
import time
import serial
import socket
import traceback
import base64
import datetime
import numpy as np
import threading
import pyproj
import rospy
from queue import Queue
from std_msgs.msg import UInt8
from irova_gnss_ros.msg import GNSS

def degmin2deg(degmin: float):
    deg = degmin // 100.0
    min = degmin  % 100.0
    
    return deg + min / 60.0

def deg2rad(deg):
    return deg * np.pi / 180.0 


class NTRIPClient:
    def __init__(self):
        self.buffer_size = 4096
        self.user = base64.b64encode(bytes('ttngtest01:ngii', 'utf-8')).decode('utf-8')
        self.port = 2101
        self.caster = 'RTS2.ngii.go.kr'
        self.mountpoint = 'VRS-RTCM32'
        self.user_agent = 'ntrip_client'
        self.rtcm_request_interval = 0.1
        
        self.connection_error_cnt = 0
        
    def print_error(self):
        exc_type, exc_obj, exc_tb = sys.exc_info()
        if exc_tb is not None:
            line_no = exc_tb.tb_lineno
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, line_no)
        print(traceback.format_exc())
        
    def bytes_chksum(self, barray):
        val = 0
        for b in barray:
            val = val ^ b
        return b

    def get_mountpoint_bytes(self):
        mount_point_string = 'GET /{} HTTP/1.1\r\nUser-Agent: {}\r\nAuthorization: Basic {}\r\n'.format(
            self.mountpoint, self.user_agent, self.user
        )
        mount_point_string += 'Host: %s:%i\r\n'.format(self.caster, self.port)
        mount_point_string += 'Ntrip-Version: Ntrip/2.0\r\n'
        mount_point_string += '\r\n'
        
        return bytes(mount_point_string, 'ascii')
    
    def byte_checksum(self, barray):
        val = 0x00 
        for b in barray:
            val = val ^ b
        return val
    
    def string_checksum(self, string):
        val = 0
        for ch in string:
            val = val ^ ord(ch)
        return val
    
    def get_dummy_gga_bytes(self):
        tmp_UTC = datetime.datetime.now().strftime('%Y%m%d:%H%M%S.%f').split(':')[1]
        tmp_gga_msg = 'GNGGA,{},3550.92296,N,12829.50878,E,1,12,1.35,55.6,M,22.7,M,,'.format(tmp_UTC)
        tmp_gga_msg = '$' + tmp_gga_msg + '*{:02x}\r\n'.format(self.byte_checksum(bytes(tmp_gga_msg, 'ascii')))
        tmp_gga_bytes = bytes(tmp_gga_msg,'ascii')
        return tmp_gga_bytes
    
    def run(self, gga_buffer: Queue, rtcm_buffer: Queue, command_queue: Queue):
        reconnect_try = 0
        op = True
        
        tmp_gga_bytes = None
        
        rospy.loginfo('[NTRIP] Thread started.')
        
        sock = None
        # if not DEBUG:
            # Wait for initial GGA message
        while op:
            try:
                if gga_buffer.qsize() == 0:
                    time.sleep(0.1)
                    if command_queue.qsize() != 0:
                        break
                else:
                    tmp_gga_bytes = gga_buffer.get()
                    break
            except Exception as e:
                rospy.logerr('[NTRIP] Error: {}'.format(e))
                self.print_error()
                command_queue.put(False)
        # else:
        #     tmp_gga_bytes = self.get_dummy_gga_bytes()
            # tmp_gga_bytes = b'$GNGGA,052935.00,3550.92296,N,12829.50878,E,1,12,1.35,55.6,M,22.7,M,,*78\r\n'
        # print('[NTRIP] GGA bytes received: {}'.format(tmp_gga_bytes))
        
        try:
            while True:
                if command_queue.qsize() != 0:
                    break
                found_header = False
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    try:
                        # print(self.caster, self.port)
                        
                        error_indicator = sock.connect_ex((self.caster, self.port))
                        if error_indicator == 0:
                            sock.settimeout(10)
                            sock.sendall(self.get_mountpoint_bytes())
                            
                            rospy.loginfo('[NTRIP] Mount point bytes sent')
                            while not found_header:
                            
                                if command_queue.qsize() != 0:
                                    break
                                caster_response = sock.recv(self.buffer_size)
                                header_lines = caster_response.decode('utf-8').split('\r\n')
                                
                                for line in header_lines:
                                    if line == '':
                                        if not found_header:
                                            found_header=True
                                    else:
                                        pass
                                for line in header_lines:
                                    if line.find("SOURCETABLE") >= 0:
                                        # print(line)
                                        rospy.logerr('[NTRIP] Mount point does not exist.')
                                        op = False
                                        break
                                    elif line.find('401 Unauthorized') >= 0:
                                        # print(line)
                                        rospy.logerr('[NTRIP] Unauthorized request.')
                                        op = False
                                        break
                                    elif line.find('404 Not Found') >= 0:
                                        # print(line)
                                        rospy.logerr('[NTRIP] Mount point does not exist.')
                                        op = False
                                        break
                                    elif line.find('ICY 200 OK') >= 0:
                                        # print(line)
                                        sock.sendall(tmp_gga_bytes)
                                        break
                                    elif line.find('HTTP/1.0 200 OK') >= 0:
                                        # print(line)
                                        sock.sendall(tmp_gga_bytes)
                                        break
                                    elif line.find('HTTP/1.1 200 ok') >= 0:
                                        # print(line)
                                        sock.sendall(tmp_gga_bytes)
                                        break
                                time.sleep(0.01)
                                rospy.loginfo('[NTRIP] Connected to NTRIP caster.')
                            
                                rtcm_bytes = b'initial_data'
                                while rtcm_bytes and op:
                                    if command_queue.qsize() != 0:
                                        break
                                    
                                    rtcm_bytes = sock.recv(self.buffer_size)
                                    
                                    if len(rtcm_bytes) != 0:
                                        if rtcm_buffer.qsize() == 10:
                                            _ = rtcm_buffer.get()
                                        rtcm_buffer.put(rtcm_bytes)
                                        
                                        # print(rtcm_bytes)
                                    
                                    gga_buffer_size = gga_buffer.qsize()
                                    for _ in range(gga_buffer_size):
                                        tmp_gga_bytes = gga_buffer.get()
                                        sock.sendall(tmp_gga_bytes)
                                        
                                    time.sleep(0.01)
                                    
                        else:
                            if reconnect_try == 10:
                                op = False
                                rospy.logwarn('[NTRIP] No connection to NTRIP caster. Closing connection')
                                break
                            else:
                                reconnect_try += 1
                                rospy.logwarn('[NTRIP] Try to reconnect ... ({}/10)'.format(reconnect_try))
                                
                        time.sleep(0.5)
                    except socket.timeout:
                        self.connection_error_cnt += 1
                        rospy.logerr('[NTRIP] NTRIP caster connection timeout. ({}/10)'.format(self.connection_error_cnt))
                        break
                    
                    except socket.error:
                        self.connection_error_cnt += 1
                        rospy.logerr('[NTRIP] Ntrip caster connection error. ({}/10)'.format(self.connection_error_cnt))
                        break
                    
                    if self.connection_error_cnt == 10:
                        rospy.logerr('[NTRIP] NTRIP caster connection instable.')
                        break
                    else:
                        self.connection_error_cnt += 1
                            
        except Exception as e:
            rospy.logerr('[NTRIP] Error: {}'.format(e))
            self.print_error()
            command_queue.put(False)
            
        finally:
            command_queue.put(False)
            rospy.loginfo('[NTRIP] Thread terminated.')


class GNSSSerial:
    def __init__(self):
        self.port = '/dev/zed_f9p'
        self.baudrate = 115200
        self.ser = None
        
        if DEBUG:
            self.prev_lon = 12829.50878
            self.prev_lat = 3550.92296
        
    def byte_checksum(self, barray):
        val = 0x00 
        for b in barray:
            val = val ^ b
        return val
    
    def print_error(self):
        exc_type, exc_obj, exc_tb = sys.exc_info()
        if exc_tb is not None:
            line_no = exc_tb.tb_lineno
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, line_no)
        print(traceback.format_exc())
        
    def connect(self, command_queue: Queue):
        cnt = 0
        while True:
            if command_queue.qsize() != 0:
                break
            
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=0.01
                )
                
                if self.ser.is_open:
                    rospy.loginfo('[SERIAL] Connected to \'{}\''.format(self.port))
                    break
                else:
                    cnt += 1
                    if cnt == 10:
                        rospy.logwarn('[SERIAL] Connection failed. Serial port is not available.')
                        break
                    else:
                        rospy.logwarn('[SERIAL] Connection failed. Retry in 1 second ... [{}/10]'.format(cnt))
                        time.sleep(1.0)
            except Exception as e:
                rospy.logerr('[SERIAL] Error: {}'.format(e))
                self.print_error()
                command_queue.put(False)
                break
        
    def read_rx_bytes(self, nmea_bytes_buffer: Queue):
        rcv = self.ser.readall()
        if len(rcv) > 0:
            nmea_bytes_buffer.put(rcv)
            # print(rcv   )
        time.sleep(0.01)
        
    def write_tx_bytes(self, rtcm_buffer: Queue):
        if rtcm_buffer.qsize() != 0:
            tmp_rtcm_bytes = rtcm_buffer.get()
            self.ser.write(tmp_rtcm_bytes)
            # print(tmp_rtcm_bytes)
        time.sleep(0.01)
    
    def run(self, nmea_bytes_buffer: Queue, rtcm_buffer: Queue, command_queue: Queue):
        op = True
        
        rospy.loginfo('[SERIAL] Thread started.')
        
        self.connect(command_queue)
        
        try:
            while op:
                # Main code here
                # Main 1: read bytes from ZED-F9P
                self.read_rx_bytes(nmea_bytes_buffer)
                
                self.write_tx_bytes(rtcm_buffer)
                
                time.sleep(0.01) 
                if command_queue.qsize() != 0:
                    break
        
        except Exception as e:
            rospy.loginfo('[SERIAL] Error: {}'.format(e))
            self.print_error()
            command_queue.put(False)
        
        finally:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            command_queue.put(False)
            rospy.loginfo('[SERIAL] Thread terminated.')
            

class CoordConverter:
    def __init__(self):
        self.EPSG               = 'epsg5179'
        self.transformer        = None
        
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
        
    def convert_lat_lon_to_x_y(self, latitude, longitude):
        return self.transformer.transform(longitude, latitude)
        
class NMEAParser:
    def __init__(self):
        self.coord_transformer = CoordConverter()
        
        self.rx_buffer      = bytearray()
        
        self.UTC            = None
        self.latitude       = 0.0
        self.longitude      = 0.0
        self.altitude       = 0.0
        self.n_flag         = "N"
        self.e_flag         = "E"
        self.gps_quality    = "Position Fix unavailable"
        self.mode           = 'Unidentified'
        self.control_mode   = "Unidentified"
        self.fix_mode       = "Fix not available"
        self.data_validity  = "Data Not Valid"
        self.hdop           = 0.0
        self.vdop           = 0.0
        self.pdop           = 0.0
        self.course         = 0.0
        self.course_m       = 0.0
        self.speed          = 0.0
        self.speed_kmh      = 0.0
        self.origin_update  = False
        self.origin_e       = 0.0
        self.origin_n       = 0.0
        self.coord_e        = 0.0
        self.coord_n        = 0.0
        
        self.gnss_topic = '/zed-f9p'
        self.gnss_pub = rospy.Publisher(self.gnss_topic, GNSS, queue_size=10)
        
    def default_configuration(self):
        pass
        
    def print_error(self):
        exc_type, exc_obj, exc_tb = sys.exc_info()
        if exc_tb is not None:
            line_no = exc_tb.tb_lineno
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, line_no)
        print(traceback.format_exc())
        
    def byte_checksum(self, barray):
        val = 0x00 
        for b in barray:
            val = val ^ b
        return val
    
    def string_checksum(self, string):
        val = 0
        for ch in string:
            val = val ^ ord(ch)
        return val
    
    def convert_coordinate(self):
        e, n = self.coord_transformer.convert_lat_lon_to_x_y(self.longitude, self.latitude)
        if not self.origin_update:
            self.origin_e = e
            self.origin_n = n
            self.origin_update = True
            
        self.coord_e = e - self.origin_e
        self.coord_n = n - self.origin_n
        
    def parse_nmea_msg(self, NmeaBytesBuffer: Queue, GgaBuffer: Queue):
        ret = []
        
        if NmeaBytesBuffer.qsize() != 0:
            self.rx_buffer += NmeaBytesBuffer.get()
        
        while True:
            start_idx = self.rx_buffer.find(b'$')    
            end_idx = self.rx_buffer.find(b'\r\n')
            
            if start_idx != -1 and end_idx != -1:
                tmp_msg = self.rx_buffer[start_idx: end_idx]
                payload, chksum = tmp_msg.split(b'*')
                chksum_cmpr = self.byte_checksum(payload[1:])
                
                if chksum_cmpr == int(chksum.decode('utf-8'), 16):
                    if payload[3:6] == b'GGA':
                        GgaBuffer.put(tmp_msg + b'\r\n')
                        
                    ret.append(tmp_msg)
                else:
                    rospy.logwarn('[NMEA] Checksum failed: {} ({}, {}, {})'.format(tmp_msg, payload[1:], chksum_cmpr, chksum))
                        
                self.rx_buffer = self.rx_buffer[end_idx + 2:]
                continue
            else:
                break
        return ret
    
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
            self.hdop           = int(float(segments[8]) * 10)
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
            self.pdop           = int(float(segments[4]) * 10)
        if segments[5] != b'':
            self.hdop           = int(float(segments[5]) * 10)
        if segments[6] != b'':
            self.vdop           = int(float(segments[6]) * 10)
    
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
            self.speed          = int(float(segments[7]) * 10)
        if segments[8] != b'':
            self.course         = int(float(segments[8]) * 10)
        if segments[12] != b'':
            self.mode           = self.check_algorithm_mode(segments[12])
    
    def parse_vtg_msg(self, segments: list):
        if segments[1] != b'':
            self.course         = int(float(segments[1]) * 10)
        if segments[3] != b'':
            self.course_m       = int(float(segments[3]) * 10)
        if segments[5] != b'':
            self.speed          = int(float(segments[5]) * 10)
        if segments[7] != b'':
            self.speed_kmh      = int(float(segments[7]) * 10)
        if segments[9] != b'':
            tmp_mode            = self.check_algorithm_mode(segments[9])
            if tmp_mode != self.mode:
                self.rtk_status_update_flag = True
                self.mode = tmp_mode
    
    def convert_utc(self, barray: bytearray):
        string = barray.decode('ascii')
        res = '{}:{}:{}'.format(string[:2], string[2:4], string[4:])
        return res
    
    def check_gps_quality(self, mark):
        if mark == b'0':
            return (0, 'Position fix unavailable')
        elif mark == b'1':
            return (1, 'Valid position fix (SPS mode)')
        elif mark == b'2':
            return (2, 'Valid position fix (DGPS mode)')
        elif mark == b'4':
            return (4, 'RTK fixed ambiguity solution')
        elif mark == b'5':
            return (5, 'RTK floating ambiguity solution')
        elif mark == b'6':
            return (6, 'Dead reckoning mode')
        elif mark == b'7':
            return (7, 'Manual input mode (fixed position)')
        elif mark == b'8':
            return (8, 'Simulation mode')
        elif mark == b'9':
            return (9, 'WAAS (SBAS)')
        
    def check_data_validity(self, mark):
        if mark == b'A':
            return (1, 'Valid')
        elif mark == b'V':
            return (2, 'Data Not Valid')
        else:
            return (0, 'Unidentified')
        
    def check_control_mode(self, mark):
        if mark == b'M':
            return (1, 'Manual')
        elif mark == b'A':
            return (2, 'Automatic')
        else:
            return (0, 'Unidentified')
    
    def check_fix_type(self, mark):
        if mark == b'1':
            return (1, 'Fix not available')
        elif mark == b'2':
            return (2, '2D')
        elif mark == b'3':
            return (3, '3D')
        else:
            return (0, 'Unidentified')
    
    def check_algorithm_mode(self, mark):
        if mark == b'N':
            return (1, 'Data not valid')
        elif mark == b'A':
            return (2, 'Autonomous mode')
        elif mark == b'D':
            return (3, 'Differential mode')
        elif mark == b'E':
            return (4, 'Estimated(Dead-reckoning) mode')
        elif mark == b'F':
            return (5, 'RTK-Floting')
        elif mark == b'R':
            return (6, 'RTK-Fixed')
        elif mark == b'M':
            return (7, 'Manual input')
        elif mark == b'P':
            return (8, 'Precise')
        elif mark == b'S':
            return (9, 'Simulator')
        else:
            return (0, 'Unidentified')
                
    def get_data_from_msg(self, msg_list: list):
        for msg in msg_list:
            header, chksum = msg.split(b'*')
            segments = header.split(b',')
            send_msg = False
            if segments[0].endswith(b"GGA"):
                self.parse_gga_msg(segments)
                send_msg = True
            elif segments[0].endswith(b"GLL"):
                self.parse_gll_msg(segments)
                send_msg = True
            elif segments[0].endswith(b"GSA"):
                self.parse_gsa_msg(segments)
                send_msg = True
            elif segments[0].endswith(b"GSV"):
                continue
                # self.parse_gsv_msg(segments)
                # self.publish_gnss_msg()
            elif segments[0].endswith(b'RMC'):
                self.parse_rmc_msg(segments)
                send_msg = True
            elif segments[0].endswith(b'VTG'):
                self.parse_vtg_msg(segments)
                send_msg = True
            else:
                continue
        
            if send_msg:
                self.publish_gnss_msg()
    
    def publish_gnss_msg(self):
        date = datetime.datetime.now().strftime('%Y%m%d')
        gnss_msg = GNSS()
        gnss_msg.header.stamp   = rospy.Time.now()
        gnss_msg.date           = date
        gnss_msg.UTC            = self.UTC
        gnss_msg.latitude       = self.latitude
        gnss_msg.longitude      = self.longitude
        gnss_msg.altitude       = self.altitude
        gnss_msg.N_flag         = self.n_flag
        gnss_msg.E_flag         = self.e_flag
        gnss_msg.gps_quality    = self.gps_quality[0]
        gnss_msg.mode           = self.mode[0]
        gnss_msg.control_mode   = self.control_mode[0]
        gnss_msg.fix_mode       = self.fix_mode[0]
        gnss_msg.data_validity  = self.data_validuty[0]
        gnss_msg.hdop           = self.hdop
        gnss_msg.vdop           = self.vdop
        gnss_msg.pdop           = self.pdop
        gnss_msg.course         = self.course
        gnss_msg.course_m       = self.course_m
        gnss_msg.speed          = self.speed
        gnss_msg.speed_kmh      = self.speed_kmh
        gnss_msg.Origin_E       = self.origin_e
        gnss_msg.Origin_N       = self.origin_n
        gnss_msg.Coord_E        = self.coord_e
        gnss_msg.Coord_N        = self.coord_n
        
        self.gnss_pub.publish(gnss_msg)
    
    def run(self, NmeaBytesBuffer: Queue, GgaBuffer: Queue, Operation: Queue):
        op = True
        rospy.loginfo('[NMEA] Thread started.')
        
        try:
            while op:
                if Operation.qsize() != 0:
                    break
                
                msg_list = self.parse_nmea_msg(NmeaBytesBuffer, GgaBuffer)        
                if len(msg_list) != 0:
                    self.get_data_from_msg()
                time.sleep(0.01)
                
                
        except Exception as e:
            rospy.logerr('[NMEA] Error: {}'.format(e)) 
            self.print_error()
            Operation.put(False)
        
        finally:
            Operation.put(False)
        rospy.loginfo('[NMEA] Thread terminated.')
        

class ZED_F9P:
    def __init__(self):
        self.node_name = 'ZED-F9P'
        
        self.ntrip_client = NTRIPClient()
        self.gnss_serial = GNSSSerial()
        self.nmea_parser = NMEAParser()
        
        self.NmeaBuffer = Queue()
        self.GgaBuffer = Queue()
        self.NmeaBytesBuffer = Queue()
        self.RtcmBuffer = Queue()
        self.Operation = Queue()
        
        self.ntrip_client_thread_args = (self.GgaBuffer,
                                         self.RtcmBuffer, 
                                         self.Operation,
                                         )
        self.gnss_serial_thread_args = (self.NmeaBytesBuffer,
                                        self.RtcmBuffer,
                                        self.Operation,
                                        )
        
        self.nmea_parser_thread_args = (self.NmeaBytesBuffer,
                                        self.GgaBuffer,
                                        self.Operation)
        
        self.ntrip_client_thread = None
        self.gnss_serial_thread = None
        self.nmea_parser_thread = None
        
        self.system_shutdown_topic = '/system_shutdown'
        self.system_shutdown_sub = rospy.Subscriber(self.system_shutdown_topic, UInt8, self.system_shutdown_callback)
        
    def system_shutdown_callback(self, msg):
        if msg.data == 92:
            self.Operation.put(False)
            rospy.loginfo('[{}] System shutdown'.format(self.node_name))
            
    def run(self):
        try:
            self.ntrip_client_thread = threading.Thread(target=self.ntrip_client.run,
                                                        args=self.ntrip_client_thread_args)
            self.gnss_serial_thread = threading.Thread(target=self.gnss_serial.run,
                                                       args=self.gnss_serial_thread_args)
            self.nmea_parser_thread = threading.Thread(target=self.nmea_parser.run,
                                                       args=self.nmea_parser_thread_args)
            
            self.ntrip_client_thread.start()
            self.gnss_serial_thread.start()
            self.nmea_parser_thread.start()
        
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
        
            if exc_tb is not None:
                line_no     = exc_tb.tb_lineno
                fname       = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            else:
                exc_type    = ''
                line_no     = ''
                fname       = ''
                
            print('[{}] Error: {}'.format(self.node_name, e))
            print(exc_type, fname, line_no)
            print(traceback.format_exc())
            
        finally:
            if self.ntrip_client_thread is not None and self.ntrip_client_thread.is_alive():
                self.ntrip_client_thread.join()
            if self.gnss_serial_thread is not None and self.gnss_serial_thread.is_alive():
                self.gnss_serial_thread.join()
            if self.nmea_parser_thread is not None and self.nmea_parser_thread.is_alive():
                self.nmea_parser_thread.join()
                

if __name__ == '__main__':
    rospy.init_node('ZED-F9P', anonymous=False)
    node = ZED_F9P()
    
    node.run()
        
                