import os, sys 
import socket
import json
import datetime
import base64
import time
import ssl
from queue import Queue

VERSION = 0.3
USERAGENT = "NTRIP_CLIENT"
FACTOR = 2
MAX_RECONNECT = 1
MAX_RECONNECT_TIME = 1200
SLEEP_TIME = 1


class NtripClient(object):
    def __init__(self, 
                 buffer_size, 
                 user, 
                 port, 
                 caster, 
                 mountpoint, 
                 user_agent, 
                 ssl=False, 
                 host=False, 
                 UDP_Port=None,
                 V2=False,
                 max_reconnect=0,
                 rtcm_request_interval=0.1,
                 latitude=None,
                 longitude=None,
                 altitude=None
                 ):
        self.buffer                 = buffer_size
        self.user                   = user
        self.port                   = port
        self.caster                 = caster
        self.mountpoint             = mountpoint
        self.user_agent             = user_agent
        
        self.ssl                    = ssl
        self.host                   = host
        self.UDP_Port               = UDP_Port
        self.V2                     = V2
        self.max_connect_time       = 0
        self.socket                 = None
        self.max_reconnect          = max_reconnect
        self.rtcm_request_interval  = rtcm_request_interval
        
        self.connection_error_cnt = 0
        
        
        # self.rx_buffer = bytearray()
        # if self.save_log:
        #     self.buffer_to_save = bytearray()
        self.msg_queue = []
        
        if self.UDP_Port:
            self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.UDP_socket.bind(('192.168.0.255'), 0)
            self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        else:
            self.UDP_socket = None
            
        self.utc_time           = None
        self.latitude           = latitude
        self.longitude          = longitude
        self.altitude           = altitude
        self.n_flag             = "N"
        self.e_flag             = "E"
        
        self.gga_bytes          = None

    def check_operation(self, cmd_queue: Queue):
        ret = True
        if cmd_queue.qsize() != 0:
            ret= False
            
        return ret
    
    def ntrip_chksum(self, string):
        val = 0 
        for c in string:
            val = val ^ ord(c)
            
        return val
    
    def ntrip_byte_chksum(self, barray):
        val = 0 
        for b in barray:
            val = val ^ b
            
        return val
        
    def get_mount_point_bytes(self):
        mount_point_string = 'GET /{} HTTP/1.1\r\nUser-Agent: {}\r\nAuthorization: Basic {}\r\n'.format(self.mountpoint, self.user_agent, self.user)
        
        if self.host or self.V2:
            host_string = 'Host: {}:{}\r\n'.format(self.caster, self.port)
            mount_point_string += host_string
        if self.V2:
            mount_point_string += 'Ntrip-Version: Ntrip/2.0\r\n'
        
        mount_point_string += '\r\n'
        
        return bytes(mount_point_string, 'ascii')
    
    def get_GGA_bytes(self, nmea_gga_buffer: Queue, nmea_buffer: Queue):
        if nmea_gga_buffer.qsize() != 0:
            self.gga_bytes = bytes(nmea_gga_buffer.get(), 'ascii')
            return True
        elif nmea_buffer.qsize() != 0:
            tmp_packet = nmea_buffer.get()
            header, tail = tmp_packet.split('*')
            head, payload = header.split(',')
            
            if head.endswith('GLL'):
                self.latitude   = payload[0]
                self.n_flag     = payload[1]
                self.longitude  = payload[2]
                self.e_flag     = payload[3]
                self.utc_time   = payload[4]
            elif head.endswith('RMC'):
                self.utc_time   = payload[0]
                self.latitude   = payload[1]
                self.n_flag     = payload[2]
                self.longitude  = payload[3]
                self.e_flag     = payload[4]
            
            gga_string = 'GPGGA,{},{},{},{},{},1,05,0.19,{},M,,,,'.format(
                self.utc_time,
                self.latitude,
                self.n_flag,
                self.longitude,
                self.e_flag,
                self.altitude
            )
        
            gga_string = "$" + gga_string + "*" + "{:02X}".format(self.ntrip_chksum(gga_string)) + '\r\n'
            self.gga_bytes = bytes(gga_string, 'ascii')
            return True
        else:
            return False
        
    def split_rtcm_msg(self, barray: bytearray):
        lines = []
        
        while True:
            start_idx = None
            end_idx = None
            msg_found = False
            
            if len(barray) < 4:
                break
            
            # for i in range
            
    def run(self, nmea_buffer: Queue, nmea_gga_buffer: Queue, cmd_queue: Queue):
        reconnect_try = 0
        connect_time = 0
        
        # Wait for initial nmea
        while self.operation:
            if self.get_GGA_bytes(nmea_gga_buffer, nmea_buffer):
                break
            time.sleep(0.1)
                
        try:
            op = True
            
            while reconnect_try <= self.max_reconnect and op:
                self.check_operation(cmd_queue)
                found_header = False
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                
                if self.ssl:
                    self.socket = ssl.wrap_socket(self.socket)
                    
                error_indicator = self.socket.connect_ex((self.caster, self.port))
                
                if error_indicator == 0:
                    connect_time = datetime.datetime.now()
                    
                    self.socket.settimeout(10)
                    self.socket.sendall(self.get_mount_point_bytes())
                    
                    while not found_header and op:
                        self.check_operation(cmd_queue)
                        caster_response = self.socket.recv(self.buffer)
                        header_lines = caster_response.decode('utf-8').split('\r\n')
                        
                        for line in header_lines:
                            if line == '':
                                if not found_header:
                                    found_header = True
                            else:
                                pass
                            
                        for line in header_lines:
                            if line.find("SOURCETABLE") >= 0:
                                print('[NTRIP] Mount point does not exist.')
                                op = False
                                break
                            elif line.find('401 Unauthorized') >= 0:
                                print('[NTRIP] Unauthorized request.')
                                op = False
                                break
                            elif line.find('404 Not Found') >= 0:
                                print('[NTRIP] Mount point does not exist.')
                                op = False
                                break
                            elif line.find('ICY 200 OK') >= 0:
                                self.socket.sendall(self.gga_bytes)
                                break
                            elif line.find("HTTP/1.0 200 OK") >= 0:
                                self.socket.sendall(self.gga_bytes)
                                break
                            elif line.find('HTTP/1.1 200 ok') >= 0:
                                self.socket.sendall(self.gga_bytes)
                                break
                                
                    data = 'initial data'
                    while data and op:
                        self.check_operation(cmd_queue)
                        
                        try:
                            if reconnect_try != 0:
                                reconnect_try = 0
                                
                            rtcm_bytes = self.socket.recv(self.buffer)
                            
                            if len(rtcm_bytes) != 0:
                                self.rtcm_buffer += rtcm_bytes
                            
                            if self.get_GGA_bytes(nmea_gga_buffer, nmea_buffer):
                                if last_request_time is None:
                                    self.socket.sendall(self.gga_bytes)
                                    last_request_time = time.time()
                                else:
                                    if time.time() - last_request_time >= self.rtcm_request_interval:                                    
                                        self.socket.sendall(self.get_GGA_bytes())
                            
                            time.sleep(0.01)
                            
                        except socket.timeout:
                            self.connection_error_cnt += 1    
                            print('[NTRIP] NTRIP caster connection timeout.')
                            break
                        
                        except socket.error:
                            self.connection_error_cnt += 1
                            print('[NTRIP] NTRIP caster connection error.')
                            break
                            
                    if self.connection_error_cnt > 10:
                        print('[NTRIP] ERROR - connection instable.')
                        break
                    else:
                        print('[NTRIP] Try to reconnect ...')
                        
                else:
                    if reconnect_try == 10:
                        self.operation = False
                        print('[NTRIP] No connection to NTRIP caster. Closing connection.')
                        break
                    else:
                        reconnect_try += 1
                        
                time.sleep(0.1)
                
        except Exception as e:
            print('[NTRIP] Error: {}'.format(e))
        
        finally:
            try:
                self.socket.close()
                self.socket = None
            except:
                pass
            