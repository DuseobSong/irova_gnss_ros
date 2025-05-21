#!/usr/bin/env python3

import os, sys
import time
import serial
import socket
import traceback
import base64
import datetime
from queue import Queue
# import ssl
import numpy as np

import threading

import rospy

from std_msgs.msg import UInt8

DUMMY = False
DEBUG = True


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
        
    def get_dummy_gga_bytes(self):
        tmp_UTC = datetime.datetime.now().strftime('%Y%m%d:%H%M%S.%f').split(':')[1]
        self.prev_lon += np.random.normal(0, 1) * 1e-3 * 2
        self.prev_lat += np.random.normal(0, 1) * 1e-3 * 3
        tmp_gga_msg = 'GNGGA,{},{:.4f},N,{:.4f},E,1,12,1.35,55.6,M,22.7,M,,'.format(tmp_UTC, self.prev_lat, self.prev_lon)
        tmp_gga_msg = '$' + tmp_gga_msg + '*{:02x}\r\n'.format(self.byte_checksum(bytes(tmp_gga_msg, 'ascii')))
        tmp_gga_bytes = bytes(tmp_gga_msg,'ascii')
        return tmp_gga_bytes
        
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
        if not DEBUG:
            rcv = self.ser.readall()
            if len(rcv) > 0:
                nmea_bytes_buffer.put(rcv)
                # print(rcv   )
            time.sleep(0.01)
        else:
            tmp_gga_bytes = self.get_dummy_gga_bytes()
            nmea_bytes_buffer.put(tmp_gga_bytes)
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
            
            
class NMEAParser:
    def __init__(self):
        self.rx_buffer = bytearray()
        
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
        
    def parse_nmea_msg(self, nmea_bytes_buffer: Queue, nmea_buffer: Queue, gga_buffer: Queue):
        if nmea_bytes_buffer.qsize() != 0:
            self.rx_buffer += nmea_bytes_buffer.get()
        
        while True:
            start_idx = self.rx_buffer.find(b'$')    
            end_idx = self.rx_buffer.find(b'\r\n')
            
            if start_idx != -1 and end_idx != -1:
                tmp_msg = self.rx_buffer[start_idx: end_idx]
                payload, chksum = tmp_msg.split(b'*')
                chksum_cmpr = self.byte_checksum(payload[1:])
                
                if chksum_cmpr == int(chksum.decode('utf-8'), 16):
                    nmea_buffer.put(tmp_msg + b'\r\n')
                    if payload[3:6] == b'GGA':
                        gga_buffer.put(tmp_msg + b'\r\n')
                else:
                    rospy.logwarn('[NMEA] Checksum failed: {} ({}, {}, {})'.format(tmp_msg, payload[1:], chksum_cmpr, chksum))
                        
                self.rx_buffer = self.rx_buffer[end_idx + 2:]
                continue
            else:
                break
            
    def run(self, nmea_bytes_buffer: Queue, nmea_buffer: Queue, gga_buffer: Queue, command_queue: Queue):
        op = True
        rospy.loginfo('[NMEA] Thread started.')
        
        try:
            while op:
                self.parse_nmea_msg(nmea_bytes_buffer, nmea_buffer, gga_buffer)        
                time.sleep(0.01)
                
                if command_queue.qsize() != 0:
                    break
        except Exception as e:
            rospy.logerr('[NMEA] Error: {}'.format(e)) 
            self.print_error()
            command_queue.put(False)
        
        finally:
            command_queue.put(False)
        rospy.loginfo('[NMEA] Thread terminated.')
        

class Broadcaster:
    def __init__(self):
        self.EGO_IP = None
        self.UDP_IP = None
        self.UDP_DATA_PORT = None
        
        self.configuration()
        
    def configuration(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(('8.8.8.8', 80))
            self.EGO_IP = sock.getsockname()[0]
        seg = self.EGO_IP.split('.')
        
        self.UDP_IP = '{}.{}.{}.255'.format(seg[0], seg[1], seg[2])
        self.UDP_DATA_PORT = 20000
    
    def print_error(self):
        exc_type, exc_obj, exc_tb = sys.exc_info()
        if exc_tb is not None:
            line_no = exc_tb.tb_lineno
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, line_no)
        print(traceback.format_exc())
        
    def run(self, nmea_buffer: Queue, command_queue: Queue):
        op = True
        rospy.loginfo('[BROADCAST] Thread started.')
        
        try:
            UDP_IP = self.UDP_IP
            UDP_PORT = self.UDP_DATA_PORT
            # print(UDP_IP, UDP_PORT)
            
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                
                while op:
                    try:
                        while nmea_buffer.qsize() != 0:
                            tmp_data = nmea_buffer.get()
                            message = b'[ZED-F9P]:' + tmp_data
                            sock.sendto(message, (UDP_IP, UDP_PORT))
                            # print(message)
                        
                        time.sleep(0.01)
                        if command_queue.qsize() != 0:
                            break
                        
                    except Exception as e:
                        rospy.logerr('[BROADCAST] Error: {}'.format(e))
                        self.print_error()
                        command_queue.put(False)
        except Exception as e:
            rospy.logerr('[BROADCAST] Error: {}'.format(e))
            self.print_error()
            command_queue.put(False)
        
        finally:
            command_queue.put(False)
            rospy.loginfo('[BROADCAST] Thread terminated.')


class ZED_F9P:
    def __init__(self):
        self.node_name              = 'ZED-F9P'
        
        self.ntrip_client           = NTRIPClient()
        self.gnss_serial            = GNSSSerial() 
        self.nmea_parser            = NMEAParser()
        self.broadcaster            = Broadcaster()
        
        self.nmea_buffer            = Queue()
        self.gga_buffer             = Queue()
        self.nmea_bytes_buffer      = Queue()
        self.rtcm_buffer            = Queue()
        self.command_queue          = Queue()
        
        self.ntrip_client_thread    = None
        self.gnss_serial_thread     = None
        self.nmea_parser_thread     = None
        self.broadcaster_thread     = None
        
        self.system_shutdown_topic  = '/system_shutdown'
        self.system_shutdown_sub    = rospy.Subscriber(self.system_shutdown_topic, UInt8, self.system_shutdown_callback)
    
    def system_shutdown_callback(self, msg: UInt8):
        if msg.data == 92:
            self.command_queue.put(False)
            rospy.loginfo('[{}] System shutdown'.format(self.node_name))
    
    def run(self):
        try:
            self.ntrip_client_thread = threading.Thread(target=self.ntrip_client.run, 
                                                        args=(self.gga_buffer,
                                                              self.rtcm_buffer,
                                                              self.command_queue,
                                                              ))
            self.gnss_serial_thread = threading.Thread(target=self.gnss_serial.run, 
                                                       args=(self.nmea_bytes_buffer,
                                                             self.rtcm_buffer,
                                                             self.command_queue,
                                                             ))
            self.nmea_parser_thread = threading.Thread(target=self.nmea_parser.run, 
                                                       args=(self.nmea_bytes_buffer,
                                                             self.nmea_buffer,
                                                             self.gga_buffer,
                                                             self.command_queue,
                                                             ))
            self.broadcaster_thread = threading.Thread(target=self.broadcaster.run, 
                                                       args=(self.nmea_buffer, 
                                                             self.command_queue,
                                                             ))
            
            self.ntrip_client_thread.start()
            self.gnss_serial_thread.start()
            self.nmea_parser_thread.start()
            self.broadcaster_thread.start()
            
        finally:
            if self.ntrip_client_thread is not None and self.ntrip_client_thread.is_alive():
                self.ntrip_client_thread.join()
            if self.gnss_serial_thread is not None and self.gnss_serial_thread.is_alive():
                self.gnss_serial_thread.join()
            if self.nmea_parser_thread is not None and self.nmea_parser_thread.is_alive():
                self.nmea_parser_thread.join()
            if self.broadcaster_thread is not None and self.broadcaster_thread.is_alive():
                self.broadcaster_thread.join()
                
                
if __name__ == '__main__':
    rospy.init_node('ZED-F9P', anonymous=False)
    
    DEBUG = False
    
    node = ZED_F9P()
    
    node.run()