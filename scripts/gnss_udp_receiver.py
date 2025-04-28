#!/usr/bin/env python3

import socket
import os

import folium
from queue import Queue
import time
import numpy as np
import pyproj
import threading

def degmin2deg(degmin: float):
    deg = degmin // 100.0
    min = degmin  % 100.0
    
    return deg + min / 60.0

def deg2rad(deg):
    return deg * np.pi / 180.0


class UDPReceiver:
    def __init__(self):
        self.ip_addr = socket.gethostbyname(socket.gethostname())
        self.UDP_IP = "{}.{}.{}.255".format(self.ip_addr[0], self.ip_addr[1], self.ip_addr[2])
        self.UDP_PORT = "20000"
        
        self.sock = None
        
    def check_operation(self, command_queue: Queue):
        if command_queue.qsize() != 0:
            return False
        else:
            return True
        
    def run(self, command_queue: Queue, nmea_buffer: Queue):
        op = True
        
        print('[UDP] Thread started.')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        
        while op:
            try:
                op = self.check_operation(command_queue)
                if not op:
                    break
                data, addr = self.sock.recvfrom(1024)
                
                if len(data) > 0: # Queue size never exceed 20
                    if nmea_buffer.qsize() == 20:
                        _ = nmea_buffer.get()
                    nmea_buffer.put(data)
                time.sleep(0.05)
            except KeyboardInterrupt:
                command_queue.put(False)
                op = False
                break
            except Exception as e:
                print('[UDP] Error: {}'.format(e))
                command_queue.put(False)
                op = False
                break
        
        if self.sock is not None:
            try:
                self.sock.close()
            except:
                pass
        print('[UDP] Thread terminated.')
        
class NMEAParser:
    def __init__(self, localization_update_interval=0.05, status_display_interval=1.0):
        self.nmea_byte_buffer           = bytearray()
        
        self.UTC                        = None
        self.latitude                   = None
        self.longitude                  = None
        self.altitude                   = None
        self.n_flag                     = 'N'
        self.e_flag                     = 'E'
        self.gps_quality                = None
        self.mode                       = None
        self.control_mode               = None
        self.fix_mode                   = None
        self.data_validity              = None
        self.hdop                       = None
        self.vdop                       = None
        self.pdop                       = None
        self.course                     = None
        self.course_m                   = None
        self.speed                      = None
        self.speed_kmh                  = None
        
        self.last_display_time          = 0.0
        self.status_display_interval    = status_display_interval
        
        self.update_interval            = localization_update_interval
        self.last_update_time           = 0.0
        
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
    
    def check_operation(self, command_queue: Queue):
        if command_queue.qsize() != 0:
            return False
        else:
            return True
        
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
        self.UTC            = self.convert_utc(segments[1])
        self.latitude       = degmin2deg(float(segments[2]))
        self.n_flag         = segments[3].decode('ascii')
        self.longitude      = degmin2deg(float(segments[4]))
        self.e_flag         = segments[5].decode('ascii')
        self.gps_quality    = self.check_gps_quality(segments[6])
        self.hdop           = float(segments[8])
        self.altitude       = float(segments[9])
    
    def parse_gll_msg(self, segments: list):
        self.latitude       = degmin2deg(float(segments[1]))
        self.n_flag         = segments[2].decode('ascii')
        self.longitude      = degmin2deg(float(segments[3]))
        self.e_flag         = segments[4].decode('ascii')
        self.UTC            = self.convert_utc(segments[5])
        self.data_validity  = self.check_data_validity(segments[6])
    
    def parse_gsa_msg(self, segments: list):
        self.control_mode   = self.check_control_mode(segments[1])
        self.fix_mode       = self.check_fix_type(segments[2])
        self.pdop           = float(segments[4])
        self.hdop           = float(segments[5])
        self.vdop           = float(segments[6])
    
    def parse_gsv_msg(self, segments: list):
        pass
    
    def parse_rmc_msg(self, segments: list):
        self.UTC            = self.convert_utc(segments[1])
        self.data_validity  = self.check_data_validity(segments[2])
        self.latitude       = degmin2deg(float(segments[3]))
        self.n_flag         = segments[4].decode('ascii')
        self.longitude      = degmin2deg(float(segments[5]))
        self.e_flag         = segments[5].decode('ascii')
        self.speed          = float(segments[7])
        self.course         = float(segments[8])
        self.mode           = self.check_algorithm_mode(segments[10])
    
    def parse_vtg_msg(self, segments: list):
        self.course         = float(segments[1])
        self.course_m       = float(segments[2])
        self.speed          = float(segments[3])
        self.speed_kmh      = float(segments[4])
        self.mode           = self.check_algorithm_mode(segments[5])
        
    def check_gps_quality(self, mark):
        if mark == b'0':
            return 'Position fix unavailable'
        elif mark == b'1':
            return 'Valid position fix (SPS mode)'
        elif mark == b'2':
            return 'Valid position fix (DGPS mode)'
        
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
        else:
            return 'Unidentified'
        
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
            print('Longitude:      {:12.6f} deg ({})'.format(self.longitude, self.n_flag))
            print('Latitude:       {:12.6f} deg ({})'.format(self.latitude, self.n_flag))
            print('Altitude:       {:.2f} m'.format(self.altitude))
            print('Fix mode:       {} ({})'.format(self.fix_mode, self.gps_quality))
            print('Mode:           {} ({})'.format(self.mode, self.control_mode))
            print('Course angle:   {} (magnetic: {})'.format(self.course, self.course_m))
            print('Speed:          {} knots ({} km/h)'.format(self.speed, self.speed_kmh))
            print('HDOP/VDOP/PDOP: {} / {} / {}'.format(self.hdop, self.vdop, self.pdop))
            print('Data validity:  {}'.format(self.data_validity))
            print('===================================================================')
            
            self.last_display_time = time.time()
            
    def update_localization_info(self, localization_info_buffer: Queue):
        if self.latitude is None or self.longitude is None:
            return False
        else:
            if time.time() - self.last_update_time >= self.update_interval:
                record = (self.UTC, self.longitude, self.e_flag, self.latitude, self.n_flag, self.fix_mode, self.altitude)
                if localization_info_buffer.qsize() == 1: # Always use the newest data
                    _ = localization_info_buffer.get()
                localization_info_buffer.put(record)
    
    def run(self, 
            nmea_buffer: Queue, 
            localization_info_buffer: Queue, 
            command_queue: Queue):
        op = True
        print('[NMEA] Parsing thread started.')
        
        while op:
            op = self.check_operation(command_queue)
            if not op:
                break
            
            tmp_q_size = nmea_buffer.qsize()
            for _ in range(tmp_q_size):
                tmp_data = nmea_buffer.get()
                messages = self.search_nmea_message_packet(tmp_data)
                
                for m in messages:
                    self.parse_message_packet(m)
                    
            self.update_localization_info(localization_info_buffer)
            time.sleep(0.05)
            
        print('[NMEA] Parsing thread terminated.')


class Visualizer:
    def __init__(self):
        pass
    
    
class Localizer:
    def __init__(self):
        self.utc        = None
        self.longitude  = None
        self.e_flag     = 'E'
        self.latitude   = None
        self.n_flag     = 'N'
        self.altitude   = None
        self.fix_mode   = None
        
        self.prev_e     = None
        self.prev_n     = None
        self.prev_th    = None
        self.e          = None
        self.n          = None
        self.th         = None
        self.ve         = None
        self.vn         = None
        self.omega      = None
        self.set_e      = None
        self.est_n      = None
        self.est_th     = None
        
        self.prev_t     = None
        self.cur_t      = None
        self.delta_t    = None
        self.delta_e    = None
        self.delta_n    = None
        self.delta_th   = None
    
    def check_operation(self, command_queue: Queue):
        if command_queue.qsize() != 0:
            return False
        else:
            return True
        
    def kalman_filter(self, localization_info_record):
        pass
    
    def run(self, localization_info_buffer: Queue, command_queue: Queue):
        op = True
        
        print('[LOCALIZER] Thread started.')
        while op:
            op = self.check_operation(command_queue)
            if not op:
                break
            
            tmp_q_size = localization_info_buffer.qsize()
            for _ in range(tmp_q_size):
                tmp_data = localization_info_buffer.get()
                self.kalman_filter(tmp_data)
                
        
        print('[LOCALIZER] Thread terminated.')
    
class Node:
    def __init__(self):
        self.udp_receiver               = UDPReceiver() # udp
        self.nmea_parser                = NMEAParser()  # nmea
        self.visualizer                 = Visualizer()  # vis
        self.localizer                  = Localizer()   # loc
        
        self.command_queue              = Queue() 
        self.nmea_buffer                = Queue() # udp -> nmea
        self.localization_info_buffer   = Queue() # nmea -> loc
        self.marker_buffer              = Queue() # loc -> vis
        
        self.udp_receiver_thread        = None
        self.nmea_parser_thread         = None
        self.visualizer_thread          = None
        self.localizer_thread           = None
        
    def run(self):
        self.udp_receiver_thread = threading.Thread(target=self.udp_receiver.run, args=(
            self.nmea_buffer, self.command_queue
        ))
        self.nmea_parser_thread  = threading.Thread(target=self.nmea_parser.run,  args=(
            self.nmea_buffer, self.localization_info_buffer, self.command_queue
        ))
        self.visualizer_thread   = threading.Thread(target=self.visualizer.run,   args=(
            self.marker_buffer, self.command_queue
        ))
        self.localizer_thread    = threading.Thread(target=self.localizer.run,    args=(
            self.localization_info_buffer, self.marker_buffer, self.command_queue
        ))
        
        try:
            self.udp_receiver_thread.start()
            self.nmea_parser_thread.start()
            self.visualizer_thread.start()
            self.localizer_thread.start()
        except KeyboardInterrupt:
            self.command_queue.put(False)
        finally:
            if self.udp_receiver_thread is not None and self.udp_receiver_thread.is_alive():
                self.udp_receiver_thread.join()
            if self.nmea_parser_thread is not None and self.nmea_parser_thread.is_alive():
                self.nmea_parser_thread.join()
            if self.visualizer_thread is not None and self.visualizer_thread.is_alive():
                self.visualizer_thread.join()
            if self.localizer_thread is not None and self.localizer_thread.is_alive():
                self.localizer_thread.join()


if __name__ == '__main__':
    node = Node()
    node.run()