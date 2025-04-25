import os, sys
import time
import serial
import json
from queue import Queue


class GNSSSerial:
    def __init__(self, 
                 nmea_port, 
                 rtcm_port,
                 nmea_baudrate=115200,
                 rtcm_baudrate=115200, 
                 parity=serial.PARITY_NONE,
                 bytesize=serial.EIGHTBITS,
                 stopbits=serial.STOPBITS_ONE,
                 timeout=0.05,
                 reconnect_cnt=0):
        self.nmea_port      = nmea_port
        self.nmea_baudrate  = nmea_baudrate
        self.rtcm_port      = rtcm_port
        self.rtcm_baudrate  = rtcm_baudrate
        self.bytesize       = bytesize
        self.stopbits       = stopbits
        self.parity         = parity
        self.timeout        = timeout
        self.reconnect_cnt  = reconnect_cnt
        
        self.nmea_ser       = None
        self.rtcm_ser       = None
        
        self.tx_buffer      = bytearray() # RTCM byte buffer to receive from NTRIP caster
        
    def check_operation(self, cmd_queue: Queue):
        ret = True
        if cmd_queue.qsize() != 0:
            ret = False
        
        return ret
        
    def connect_nmea_serial(self):
        cnt = 0
        ret = False
        
        while True:
            self.nmea_ser = serial.Serial(port=self.nmea_port,
                                          baudrate=self.nmea_baudrate,
                                          parity=self.parity,
                                          stopbits= self.stopbits,
                                          bytesize=self.bytesize,
                                          timeout=self.timeout)
            if self.nmea_ser.is_open():
                print('[Serial] Serial (NMEA) connected. ({}, {} bps)'.format(self.nmea_port, self.nmea_baudrate))
                ret = True
                break
            else:
                if cnt < self.reconnect_cnt:
                    print('[Serial] Serial (NMEA) connection failed. Try to reconnect in 1 second. ({} / {})'.format(cnt+1, self.reconnect_cnt+1))
                    self.nmea_ser = None
                    cnt += 1
                    time.sleep(1.0)
                    continue
                else:
                    self.nmea_ser = None
                    print('[Serial] Serial (NMEA) connection failed.')
                    break
        return ret
    
    def connect_rtcm_serial(self):
        cnt = 0
        ret = False
        
        while True:
            self.rtcm_ser = serial.Serial(port=self.rtcm_port,
                                          baudrate=self.rtcm_baudrate,
                                          parity=self.parity,
                                          stopbits=self.stopbits,
                                          bytesize=self.bytesize,
                                          timeout=self.timeout)
            if self.rtcm_ser.is_open():
                print('[Serial] Serial (RTCM) connected. ({}, {} bps)'.format(self.rtcm_port, self.rtcm_baudrate))
                ret = True
                break
            else:
                if cnt < self.reconnect_cnt:
                    print('[Serial] Serial (RTCM) connection failed. Try to reconnect in 1 second. ({} / {})'.format(cnt+1, self.reconnect_cnt+1))
                    self.rtcm_ser = None
                    cnt += 1
                    time.sleep(1.0)
                    continue
                else:
                    self.rtcm_ser = None
                    print('[Serial] Serial (RTCM) connection failed.')
                    break
        return ret
    
    def write_packet_on_tx_buffer(self, rtcm_buffer: Queue):
        initial_length = len(self.tx_buffer)
        
        if rtcm_buffer.qsize() == 0:
            return initial_length
        
        while True:
            tmp_packet = rtcm_buffer.get()
            if tmp_packet is None:
                break
            self.tx_buffer.append(tmp_packet)
            if rtcm_buffer.qsize() == 0:
                break
        
        return len(self.tx_buffer) - initial_length
    
    def save_bytes_on_rx_buffer(self, rx_buffer: Queue, rx_buffer_log: Queue):
        tmp_rx_buffer = self.nmea_ser.read_all()
        if len(tmp_rx_buffer) > 0:
            rx_buffer.put(tmp_rx_buffer)
            if rx_buffer_log is not None:
                rx_buffer_log.put(tmp_rx_buffer)
        ret = len(self.rx_buffer)
        
        return ret
    
    def send_rtcm_bytes(self):
        if len(self.tx_buffer) != 0:
            self.rtcm_ser.write(bytes(self.tx_buffer))
            self.tx_buffer = bytearray()
            return True
        else:
            return False
        
    def read_nmea_bytes(self):
        self.rx_buffer += self.nmea_ser.read_all()
    
    def nmea_run(self, rx_buffer: Queue, rx_log_buffer: Queue, cmd_queue: Queue):
        try:
            self.connect_nmea_serial()
            
            op = True
            
            while op:
                # Receive bytes from GNSS-RTK module
                ret = self.save_bytes_on_rx_buffer(rx_buffer, rx_log_buffer)
                # If any data received, then pass this RX buffer
                
                time.sleep(0.01)
                op = self.check_operation(cmd_queue)
                
        except Exception as e:
            print('[Serial] Exception: {}'.format(e))
            
        finally:
            self.read_nmea_bytes()
            try:
                if self.nmea_ser.is_open():
                    self.nmea_ser.close()
            except:
                pass
            
    def rtcm_run(self, rtcm_buffer: Queue, cmd_queue: Queue):
        try:
            self.connect_rtcm_serial()
            
            op = True
            
            while op:
                # Retrieve RTCM v3.2 bytes from RTCM buffer
                self.write_packet_on_tx_buffer(rtcm_buffer)
                
                # If any message received pass this to GNSS-RTK module
                if len(self.tx_buffer) > 0:
                    ret = self.send_rtcm_bytes()
                time.sleep(0.01)
                self.check_operation(cmd_queue)
                
        except Exception as e:
            print('[Serial] (RTCM) Exception: {}'.format(e))
            
        finally:
            try:
                if self.rtcm_ser.is_open():
                    self.rtcm_ser.close()
            except:
                pass
            

