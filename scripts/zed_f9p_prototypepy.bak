import os, sys
import rospy
from std_msgs.msg import Bool, UInt8, Float64
from irova_gnss_ros.msg import *

import traceback

import numpy as np
import pandas as pd
from pyproj import Transformer, Proj
from rospkg import RosPack

import time
import threading

import json
from queue import Queue

from GNSSSerial import GNSSSerial
from RTCMParser import RTCMParser
from NMEAParser import NMEAParser
from GNSSLocalization import GNSSLocalization
from DataLogger import DataLogger
from NTRIPClient import NtripClient

'''
Root package : irova_gnss_ros
Serial node: 
'''

rp = RosPack()
ROOT_PKG = 'irova_gnss_ros'
ROOT_PKG_DIR = rp.get_path(ROOT_PKG)
 

class GNSS:
    def __init__(self, log=False):
        self.rate                               = rospy.Rate(0.1)
        
        self.nmea_parser                        = None # NMEAParser()
        self.rtcm_parser                        = None # RTCMParser()
        self.gnss_serial                        = None # GNSSSerial()
        self.ntrip_client                       = None # NtripClient()
        self.data_logger                        = None # DataLogger()
        self.gnss_localization                  = None # GNSSLocalization()
        
        self.rx_buffer                          = Queue()
        self.nmea_gga_buffer                    = Queue()
        self.nmea_gga_ntrip_buffer              = Queue()
        self.nmea_buffer                        = Queue()
        self.nmea_ntrip_buffer                  = Queue()
        self.rtcm_buffer                        = Queue()
        self.gnss_info_buffer                   = Queue()
        self.localization_info_buffer           = Queue()
        self.n_terminated_thread                = Queue()
        self.command_queue                      = Queue()
        
        self.log_enable                         = log
        
        self.system_param                       = None
        
        self.system_operation                   = Queue()
        self.operation                          = True
        
        self.operation_sub                      = rospy.Subscriber("/gnss/operation", Bool, 
                                                                   self.operation_callback)
        
        if self.log_enable:
            self.rx_log_buffer                  = Queue()
            self.nmea_gga_log_buffer            = Queue()
            self.nmea_log_buffer                = Queue()
            self.rtcm_log_buffer                = Queue()
            self.gnss_info_log_buffer           = Queue()
            self.localization_info_log_buffer   = Queue()
        else:
            self.rx_log_buffer                  = None
            self.nmea_gga_log_buffer            = None
            self.nmea_log_buffer                = None
            self.rtcm_log_buffer                = None
            self.gnss_info_log_buffer           = None
            self.localization_info_log_buffer   = None
        
        self.lock                               = threading.Lock()
        
        self.nmea_parser_thread                 = None
        self.rtcm_serial_thread                 = None
        self.nmea_serial_thread                 = None
        self.ntrip_client_thread                = None
        self.gnss_localization_thread           = None
        self.data_logger_thread                 = None
            
        self.load_system_paramter()
        self.set_modules()
        
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
            
    def load_system_paramter(self):
        try:
            with open(ROOT_PKG_DIR + '/config/system_config.json') as f:
                self.system_param = json.loads(f.read())
            return True
        except Exception as e:
            self.raise_error(e)
            return False
    
    def set_modules(self):
        self.nmea_parser                = NMEAParser(self.system_param["NMEA_PARSER"])
        self.rtcm_parser                = RTCMParser()
        self.gnss_serial                = GNSSSerial(self.system_param["GNSS_SERIAL"])
        self.ntrip_client               = NtripClient(self.system_param["NTRIP_CLIENT"])
        self.data_logger                = DataLogger(self.system_param["DATA_LOGGER"])
        self.gnss_localization          = GNSSLocalization(self.system_param["GNSS_LOCALIZATION"])
    
        self.nmea_serial_thread         = threading.Thread(target=self.gnss_serial.nmea_run,    args=(self.rx_buffer,
                                                                                                      self.rx_log_buffer,
                                                                                                      self.command_queue))
        self.rtcm_serial_thread         = threading.Thread(target=self.gnss_serial.rtcm_run,    args=(self.rtcm_buffer,
                                                                                                      self.rtcm_log_buffer,
                                                                                                      self.command_queue))
        self.ntrip_client_thread        = threading.Thread(target=self.ntrip_client.run,        args=(self.nmea_ntrip_buffer,
                                                                                                      self.nmea_gga_ntrip_buffer,
                                                                                                      self.rtcm_buffer,
                                                                                                      self.rtcm_log_buffer,
                                                                                                      self.command_queue))
        self.nmea_parser_thread         = threading.Thread(target=self.nmea_parser.run,         args=(self.rx_buffer,
                                                                                                      self.nmea_gga_buffer,
                                                                                                      self.nmea_gga_ntrip_buffer,
                                                                                                      self.nmea_gga_log_buffer,
                                                                                                      self.nmea_buffer,
                                                                                                      self.nmea_ntrip_buffer,
                                                                                                      self.nmea_log_buffer,
                                                                                                      self.gnss_info_buffer,
                                                                                                      self.gnss_info_log_buffer,
                                                                                                      self.command_queue))
        self.gnss_localization_thread   = threading.Thread(target=self.gnss_localization.run,   args=(self.gnss_info_buffer,
                                                                                                      self.localization_info_buffer,
                                                                                                      self.localization_info_log_buffer,
                                                                                                      self.command_queue))
        if self.log_enable:
            self.data_logger_thread     = threading.Thread(target=self.data_logger.run,         args=(self.rx_log_buffer,
                                                                                                      self.nmea_gga_log_buffer,
                                                                                                      self.nmea_buffer,
                                                                                                      self.localization_info_log_buffer,
                                                                                                      self.rtcm_log_buffer,
                                                                                                      self.gnss_info_log_buffer,
                                                                                                      self.command_queue))
        
    
    def operation_callback(self, msg: Bool):
        if msg.data == False:
            self.kill_all_threads()
    
    def kill_all_threads(self):
        self.system_operation.put(False)
        
        time.sleep(0.1)
        
        if self.nmea_parser_thread.is_alive():
            self.nmea_parser_thread.join()
        if self.rtcm_serial_thread.is_alive():
            self.rtcm_serial_thread.join()
        if self.nmea_serial_thread.is_alive():
            self.nmea_serial_thread.join()
        if self.ntrip_client_thread.is_alive():
            self.ntrip_client_thread.join()
        if self.gnss_localization_thread.is_alive():
            self.gnss_localization_thread.join()
        if self.log_enable:
            if self.data_logger_thread.is_alive():
                self.data_logger_thread.join()
    
    def check_threads_alive(self):
        if self.rtcm_serial_thread.is_alive():
            return False
        if self.nmea_serial_thread.is_alive():
            return False
        elif self.ntrip_client_thread.is_alive():
            return False
        elif self.gnss_localization_thread.is_alive():
            return False
        elif self.nmea_parser_thread.is_alive():
            return False
        elif self.log_enable:
            if self.data_logger_thread.is_alive():
                return False
            else:
                return True
        else:
            return True
    
    def run(self):
        self.load_system_paramter()
        try:
            self.rtcm_serial_thread.start()
            self.nmea_serial_thread.start()
            self.ntrip_client_thread.start()
            self.gnss_localization_thread.start()
            self.nmea_parser_thread.start()
            if self.log_enable:
                self.data_logger_thread.start()
            
            while self.operation and not rospy.is_shutdown():
                try:
                    self.rate.sleep()
                except KeyboardInterrupt:
                    self.kill_all_threads()
                    break
                except Exception as e:
                    self.kill_all_threads()
                    break
        finally:    
            cnt = 0
            while cnt != 10:
                if self.check_threads_alive():
                    break
                else:
                    self.kill_all_threads()
                time.sleep(1)
                
            print('[SYSTEM] All threads are terminated.')
        
        
if __name__ == '__main__':
    


    pass