import os, sys
import time
import serial
import json
from queue import Queue

import rospy
from rospkg import RosPack

rp = RosPack()
ROOT_PKG_DIR = rp.get_path('irova_gnss_ros')


class GNSSSerial:
    def __init__(self, parameters):
        self.parameters     = parameters
        self.nmea_port      = None
        self.nmea_baudrate  = None
        self.rtcm_port      = None
        self.rtcm_baudrate  = None
        self.bytesize       = serial.EIGHTBITS
        self.stopbits       = serial.STOPBITS_ONE
        self.parity         = serial.PARITY_NONE
        self.timeout        = 0.05
        self.reconnect_cnt  = 0
        
        self.nmea_ser       = None
        self.rtcm_ser       = None
        
        self.tx_buffer      = bytearray() # RTCM byte buffer to receive from NTRIP caster
        
        self.set_parameters()
        
    def set_parameters(self):
        self.nmea_port = self.parameters["NMEA_PORT"]
        self.rtcm_port = self.parameters["RTCM_PORT"]
        self.nmea_baurdate = self.parameters["NMEA_BAUDRATE"]
        self.rtcm_baudrate = self.parameters["RTCM_BAUDRATE"]
    
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
            rospy.loginfo('[GNSS] NMEA thread started.')
            self.connect_nmea_serial()
            
            op = True
            
            while op and not rospy.is_shutdown():
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
            rospy.loginfo('[GNSS] NMEA thread terminated.')
            
    def rtcm_run(self, rtcm_buffer: Queue, cmd_queue: Queue):
        
        try:
            rospy.loginfo('[GNSS] RTCM thread started.')
            self.connect_rtcm_serial()
            
            op = True
            
            while op and not rospy.is_shutdown():
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
            rospy.loginfo('[GNSS] RTCM thread terminated.')
            