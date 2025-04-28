#!/usr/bin/env python3

import os, sys
import time
import serial
import socket
import traceback
import base64
import datetime
from queue import Queue

import threading


class NTRIPClient:
    def __init__(self):
        self.buffer_size = 4096
        self.user = base64.b64encode('ttngtest7700@gmail.com')
        self.port = 2101
        self.caster = 'RTS2.ngii.go.kr'
        self.mountpoint = 'VRS-RTCM31'
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
        
    def check_operation(self, command_queue: Queue):
        if command_queue.qsize() != 0:
            return False
        else:
            return True
        
    def bytes_chksum(self, barray):
        val = 0
        for b in barray:
            val = val ^ b
        return b

    def get_mountpoint_bytes(self):
        mount_point_string = 'GET /{} HTTP/1.1\r\nUser-Agent: {}\r\nAuthorization: Basic {}\r\n'.format(
            self.mountpoint, self.user_agent, self.user
        )
        
        mount_point_string += '\r\n'
        
        return bytes(mount_point_string, 'ascii')
    
    def run(self, gga_buffer: Queue, rtcm_buffer: Queue, command_queue: Queue):
        reconnect_try = 0
        op = True
        
        tmp_gga_bytes = None
        
        print('[NTRIP] Thread started.')
        
        # Wait for initial GGA message
        while op:
            try:
                if gga_buffer.qsize() == 0:
                    time.sleep(0.1)
                    self.check_operation(command_queue)
                else:
                    tmp_gga_bytes = gga_buffer.get()
                    break
            except KeyboardInterrupt:
                command_queue.put(False)
            except Exception as e:
                print('[NTRIP] Error: {}'.format(e))
                self.print_error()
                command_queue.put(False)
        
        try:
            while op:
                op = self.check_operation(command_queue)
                if not op:
                    break
                found_header = False
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                
                error_indicator = sock.connect_ex((self.caster, self.port))
                
                if error_indicator == 0:
                    sock.settimeout(10)
                    sock.sendall(self.get_mountpoint_bytes())
                    
                    while not found_header and op:
                        try:
                            op = self.check_operation(command_queue)
                            if not op:
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
                                    sock.sendall(tmp_gga_bytes)
                                    break
                                elif line.find('HTTP/1.0 200 OK') >= 0:
                                    sock.sendall(tmp_gga_bytes)
                                    break
                                elif line.find('HTTP/1.1 200 ok') >= 0:
                                    sock.sendall(tmp_gga_bytes)
                                    break
                            time.sleep(0.01)
                        except KeyboardInterrupt:
                            command_queue.put(False)
                        
                    rtcm_bytes = b'initial_data'
                    while rtcm_bytes and op:
                        op = self.check_operation(command_queue)
                        if not op:
                            break
                        try:
                            rtcm_bytes = sock.recv(self.buffer_size)
                            
                            if len(rtcm_bytes) != 0:
                                rtcm_buffer.put(rtcm_bytes)
                            
                            if gga_buffer.qsize() != 0:
                                tmp_gga_bytes = gga_buffer.get()
                                sock.sendall(tmp_gga_bytes)
                            time.sleep(0.01)
                        except KeyboardInterrupt:
                            command_queue.put(False)
                        except socket.timeout:
                            self.connection_error_cnt += 1
                            print('[NTRIP] NTRIP caster connection timeout. ({}/10)'.format(self.connection_error_cnt))
                            break
                        except socket.error:
                            self.connection_error_cnt += 1
                            print('[NTRIP] Ntrip caster connection error. ({}/10)'.format(self.connection_error_cnt))
                            break
                    
                    if self.connection_error_cnt == 10:
                        print('[NTRIP] NTRIP caster connection instable.')
                        break
                    else:
                        self.connection_error_cnt += 1
                        
                else:
                    if reconnect_try == 10:
                        op = False
                        print('[NTRIP] No connection to NTRIP caster. Closing connection')
                        break
                    else:
                        reconnect_try += 1
                        print('[NTRIP] Try to reconnect ... ({}/10)'.format(reconnect_try))
                        
                time.sleep(0.01)
        except KeyboardInterrupt:
            command_queue.put(False)
        
        except Exception as e:
            print('[NTRIP] Error: {}'.format(e))
            self.print_error()
            command_queue.put(False)
            
        finally:
            try:
                sock.close()
                sock = None
            except:
                pass
        print('[NTRIP] Thread terminated.')
    
class GNSSSerial:
    def __init__(self):
        self.port = '/dev/UBLOX-F9P'
        self.baudrate = 115200
        self.ser = None
        
    def print_error(self):
        exc_type, exc_obj, exc_tb = sys.exc_info()
        if exc_tb is not None:
            line_no = exc_tb.tb_lineno
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, line_no)
        print(traceback.format_exc())
        
    def connect(self, command_queue: Queue):
        op = True
        cnt = 0
        while op:
            op = self.check_operation(command_queue)
            
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
                    print('[SERIAL] Connected to \'{}\''.format(self.port))
                    break
                else:
                    cnt += 1
                    if cnt == 10:
                        print('[SERIAL] Connection failed. Serial port is not available.')
                        break
                    else:
                        print('[SERIAL] Connection failed. Retry in 1 second ... [{}/10]'.format(cnt))
                        time.sleep(1.0)
            except KeyboardInterrupt:
                command_queue.put(False)
                break
            except Exception as e:
                print('[SERIAL] Error: {}'.format(e))
                self.print_error()
                command_queue.put(False)
    
    def check_operation(self, command_queue: Queue):
        if command_queue.qsize() != 0:
            return False
        else:
            return True
        
    def read_rx_bytes(self, nmea_bytes_buffer: Queue):
        rcv = self.ser.read_all()
        if len(rcv) > 0:
            nmea_bytes_buffer.put(rcv)
        time.sleep(0.01)
        
    def write_tx_bytes(self, rtcm_buffer: Queue):
        if rtcm_buffer.qsize() != 0:
            tmp_rtcm_bytes = rtcm_buffer.get()
            self.ser.write(tmp_rtcm_bytes)
        time.sleep(0.01)
    
    def run(self, nmea_bytes_buffer: Queue, rtcm_buffer: Queue, command_queue: Queue):
        op = True
        
        print('[SERIAL] Thread started.')
        
        self.connect(command_queue)
        
        try:
            while op:
                # Main code here
                # Main 1: read bytes from ZED-F9P
                self.read_rx_bytes(nmea_bytes_buffer)
                
                self.write_tx_bytes(rtcm_buffer)
                
                time.sleep(0.01) 
                op = self.check_operation(command_queue)
        
        except KeyboardInterrupt:
            command_queue.put(False)
        
        except Exception as e:
            print('[SERIAL] Error: {}'.format(e))
            self.print_error()
            command_queue.put(False)
        
        finally:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
                
            print('[SERIAL] Thread terminated.')
            
            
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
    
    def check_operation(self, command_queue: Queue):
        if command_queue.qsize() != 0:
            return False
        else:
            return True
        
    def parse_nmea_msg(self, nmea_bytes_buffer: Queue, nmea_buffer: Queue, gga_buffer: Queue):
        if nmea_bytes_buffer.qsize() != 0:
            self.rx_buffer += nmea_bytes_buffer.get()
        
        while True:
            start_idx = self.rx_buffer.find(b'$')    
            end_idx = self.rx_buffer.find(b'\r\n')
            
            if start_idx != -1 and end_idx != -1:
                tmp_msg = self.rx_buffer[start_idx: end_idx]
                payload, chksum = tmp_msg.split('*')
                chksum_cmpr = self.byte_checksum(payload[1:])
                
                if chksum_cmpr == chksum:
                    nmea_buffer.put(tmp_msg + b'\r\n')
                    if payload[3:6] == b'GGA':
                        gga_buffer.put(tmp_msg + b'\r\n')
                else:
                    print('[NMEA] Checksum filed: {} ({})'.format(tmp_msg, chksum_cmpr))
                        
                self.rx_buffer = self.rx_buffer[end_idx + 2:]
                continue
            else:
                break
            
    def run(self, nmea_bytes_buffer: Queue, nmea_buffer: Queue, gga_buffer: Queue, command_queue: Queue):
        op = True
        print('[NMEA] Thread started.')
        
        try:
            while op:
                self.parse_nmea_msg(nmea_bytes_buffer, nmea_buffer, gga_buffer)        
                time.sleep(0.01)
                
                op = self.check_operation(command_queue)
        except KeyboardInterrupt:
            command_queue.put(False)
        
        except Exception as e:
            print('[NMEA] Error: {}'.format(e)) 
            self.print_error()
            command_queue.put(False)
        
        print('[NMEA] Thread terminated.')
        

class Broadcaster:
    def __init__(self):
        pass
    
    def print_error(self):
        exc_type, exc_obj, exc_tb = sys.exc_info()
        if exc_tb is not None:
            line_no = exc_tb.tb_lineno
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, line_no)
        print(traceback.format_exc())
        
    def check_operation(self, command_queue: Queue):
        if command_queue.qsize() != 0:
            return False
        else:
            return True
        
    def run(self, nmea_buffer: Queue, command_queue: Queue):
        op = True
        print('[BROADCAST] Thread started.')
        
        try:
            UDP_IP = "192.168.0.255"
            UDP_PORT = 20000
            
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            
            while op:
                try:
                    while nmea_buffer.qsize() != 0:
                        tmp_data = nmea_buffer.get()
                        message = b'[ZED-F9P]:' + tmp_data
                        sock.sendto(message, (UDP_IP, UDP_PORT))
                    
                    time.sleep(0.01)
                    op = self.check_operation(command_queue)
                except Exception as e:
                    print('[BROADCAST] Error: {}'.format(e))
                    self.print_error()
                    
                except KeyboardInterrupt:
                    command_queue.put(False)
                    break
            
        except KeyboardInterrupt:
            command_queue.put(False)
            
        except Exception as e:
            print('[BROADCAST] Error: {}'.format(e))
            self.print_error()
            command_queue.put(False)
        
        finally:
            sock.close()
            print('[BROADCAST] Thread terminated.')
    
    
class ZED_F9P:
    def __init__(self):
        self.ntrip_client = NTRIPClient()
        self.gnss_serial = GNSSSerial() 
        self.nmea_parser = NMEAParser()
        self.broadcaster = Broadcaster()
        
        self.nmea_buffer = Queue()
        self.gga_buffer = Queue()
        self.nmea_bytes_buffer = Queue()
        self.rtcm_buffer = Queue()
        self.command_queue = Queue()
        
        self.ntrip_client_thread = None
        self.gnss_serial_thread = None
        self.nmea_parser_thread = None
        self.broadcaster_thread = None
    
    def run(self):
        try:
            self.ntrip_client_thread = threading.Thread(self.ntrip_client.run, 
                                                        args=(self.gga_buffer,
                                                              self.rtcm_buffer,
                                                              self.command_queue))
            self.gnss_serial_thread = threading.Thread(self.gnss_serial.run, 
                                                       args=(self.nmea_bytes_buffer,
                                                             self.rtcm_buffer,
                                                             self.command_queue))
            self.nmea_parser_thread = threading.Thread(self.nmea_parser.run, 
                                                       args=(self.nmea_bytes_buffer,
                                                             self.nmea_buffer,
                                                             self.gga_buffer,
                                                             self.command_queue))
            self.broadcaster_thread = threading.Thread(self.broadcaster.run, 
                                                       args=(self.nmea_buffer, 
                                                             self.command_queue))
            
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
    node = ZED_F9P()
    
    node.run()