import os, sys
import time
import json
import pandas as pd
import numpy as np
import datetime

from queue import Queue

import rospy
from rospkg import RosPack

rp = RosPack()
ROOT_PKG_DIR = rp.get_path('irova_gnss_ros')

class DataLogger:
    def __init__(self, parameters):
        self.parameters                         = parameters
        self.update_interval                    = None
        self.log_path                           = ''
        self.data_to_save                       = None
        
        self.max_line                           = None
        self.max_bytes                          = None
        
        self.rx_buffer_log_path                 = None
        self.nmea_gga_buffer_log_path           = None
        self.nmea_buffer_log_path               = None
        self.localization_info_log_path         = None
        self.rtcm_buffer_log_path               = None
        self.gnss_info_buffer_log_path          = None
        
        self.localization_info_attributes       = None
        self.gnss_info_attributes               = None
        
        self.dummy_localization_info_chunk      = None
        self.dummy_gnss_info_chunk              = None
        
        self.tmp_rx_log                         = None
        self.tmp_nmea_gga_log                   = None
        self.tmp_nmea_log                       = None
        self.tmp_localization_info_log          = None
        self.tmp_rtcm_log                       = None
        self.tmp_gnss_info_log                  = None
        
        self.rx_log_chunk_cnt                   = -1
        self.nmea_gga_log_chunk_cnt             = -1
        self.nmea_log_chunk_cnt                 = -1
        self.localization_info_log_chunk_cnt    = -1
        self.rtcm_log_chunk_cnt                 = -1
        self.gnss_info_log_chunk_cnt            = -1
        
        self.tmp_rx_log_byte_length             = 0
        self.tmp_nmea_gga_log_line_cnt          = 0
        self.tmp_nmea_log_line_cnt              = 0
        self.tmp_localization_info_log_line_cnt = 0
        self.tmp_rtcm_log_byte_length           = 0
        self.tmp_gnss_info_log_line_cnt         = 0
        
        self.set_parameters()
    
    def set_parameters(self):
        self.log_path           = ROOT_PKG_DIR + self.parameters["LOG_PATH"]
        self.data_to_sace       = self.parameters["DATA_TO_SAVE"]
        self.max_line           = self.parameters["MAX_LINE"]
        self.max_bytes          = self.parameters["MAX_BYTES"]
        
        if os.path.exists(self.log_path):
            os.mkdir(self.log_path)
    
    def set_operation(self, cmd_queue: Queue):
        if cmd_queue.qsize() != 0:
            return False
        else:
            return True
            
    def create_dummy_dataframe_chunk(self):
        dummy = {}
        for attrib in self.gnss_info_attributes:
            if attrib == "N_INDICATOR":
                dummy[attrib] = ['N' for _ in range(self.max_line)]
            elif attrib == "E_INDICATOR":
                dummy[attrib] = ['E' for _ in range(self.max_line)]
            elif attrib == "UTC":
                dummy[attrib] = ['000000.000' for _ in range(self.max_line)]
            else:
                dummy[attrib] = [0.0 for _ in range(self.max_line)]
                
        self.dummy_gnss_info_chunk = pd.DataFrame(dummy)
        
        dummy = {}
        for attrib in self.localization_info_attributes:
            if attrib == "N_INDICATOR":
                dummy[attrib] = ['N' for _ in range(self.max_line)]
            elif attrib == "E_INDICATOR":
                dummy[attrib] = ['E' for _ in range(self.max_line)]
            elif attrib == "UTC":
                dummy[attrib] = ['000000.000' for _ in range(self.max_line)]
            else:
                dummy[attrib] = [0.0 for _ in range(self.max_line)]
        self.dummy_localization_info_chunk = pd.DataFrame(dummy)
    
    def initialize_data_log_chunk(self):
        self.initialize_rx_log_chunk()
        self.initialize_nmea_gga_log_chunk()
        self.initialize_nmea_log_chunk()
        self.initialize_rtcm_log_chunk()
        self.initialize_localization_info_log_chunk()
        self.initialize_gnss_info_log_chunk()
        
    def initialize_rx_log_chunk(self):
        self.tmp_rx_log                     = bytes()
        self.rx_log_chunk_cnt               += 1
        self.tmp_rx_log_byte_length  = 0
        
    def initialize_nmea_gga_log_chunk(self):
        self.tmp_nmea_gga_log               = ''
        self.nmea_gga_log_chunk_cnt         += 1
        self.tmp_nmea_gga_log_line_cnt      = 0
    
    def initialize_nmea_log_chunk(self):
        self.tmp_nmea_log                   = ''
        self.nmea_log_chunk_cnt             += 1
        self.tmp_nmea_log_line_cnt          = 0
    
    def initialize_localization_info_log_chunk(self):
        self.tmp_localization_info_log          = self.dummy_localization_info_chunk
        self.localization_info_log_chunk_cnt    += 1
        self.tmp_localization_info_log_line_cnt = 0
            
    def initialize_rtcm_log_chunk(self):
        self.tmp_rtcm_log               = bytes()
        self.rtcm_log_chunk_cnt         += 1
        self.tmp_rtcm_log_byte_length   = 0
    
    def initialize_gnss_info_log_chunk(self):
        self.tmp_gnss_info_log          = self.dummy_gnss_info_chunk
        self.gnss_info_log_chunk_cnt    += 1
        self.tmp_gnss_info_log_line_cnt = 0
        
    def save_rx_log_chunk(self):
        d,t = datetime.datetime.now().strftime("%Y%m%d:%H%M%S").strip(':')
        
        file_name = '/{}_rx_log_{}_{}.bin'.format(d, self.rx_log_chunk_cnt, t)
        with open(self.log_path + file_name, 'wb') as f:
            f.write(self.tmp_rx_log)
    
    def save_nmea_gga_log_chunk(self):
        d,t = datetime.datetime.now().strftime("%Y%m%d:%H%M%S").strip(":")
        
        file_name = '/{}_nmea_gga_log_{}_{}.txt'.format(d, self.nmea_gga_log_chunk_cnt, t)
        with open(self.log_path + file_name, 'w') as f:
            f.write(self.tmp_nmea_gga_log)
            
    def save_nmea_log_chunk(self):
        d,t = datetime.datetime.now().strftime("%Y%m%d:%H%M%S").strip(':')
        
        file_name = '/{}_nmea_log_{}_{}.txt'.format(d, self.nmea_log_chunk_cnt, t)
        with open(self.log_path + file_name, 'w') as f:
            f.write(self.tmp_nmea_log)
    
    def save_rtcm_log_chunk(self):
        d,t = datetime.datetime.now().strftime("%Y%m%d:%H%M%S").strip(":")
        
        file_name = '/{}_rtcm_log_{}_{}.bin'.format(d, self.rtcm_log_chunk_cnt, t)
        with open(self.log_path + file_name, 'wb') as f:
            f.write(self.tmp_rtcm_log)
            
    def save_localization_info_log_chunk(self):
        d,t  = datetime.datetime.now().strftime('%Y%m%s:%H%M%S').strip(':')
        
        file_name = '/{}_localization_info_log_{}_{}.csv'.format(d, 
                                                                 self.localization_info_log_chunk_cnt, 
                                                                 t)
        self.tmp_localization_info_log.iloc[:self.tmp_localization_info_log_line_cnt].to_csv(self.log_path + file_name, 
                                              columns=self.localization_info_attributes,
                                              index=False)
    
    def save_gnss_info_log_chunk(self):
        d,t = datetime.datetime.now().strftime('%Y%m%s:%H%M%S').split(':')
        file_name = '/{}_gnss_info_log_{}_{}.csv'.format(d, self.gnss_info_log_chunk_cnt, t)
        
        self.tmp_gnss_info_log.iloc[:self.tmp_gnss_info_log_line_cnt].to_csv(self.log_path + file_name,
                                      columns=self.gnss_info_attributes,
                                      index=False)
    
    def update_rx_log(self, rx_log_buffer: Queue):
        while rx_log_buffer.qsize() != 0:
            tmp_data = rx_log_buffer.get()
            self.tmp_rx_log += tmp_data
            self.tmp_rx_log_byte_length += len(tmp_data)
            
            if self.tmp_rx_log_byte_length >= self.max_bytes:
                self.save_rx_log_chunk()
                self.initialize_rx_log_chunk()
    
    def update_nmea_gga_log(self, nmea_gga_log_buffer: Queue):
        while nmea_gga_log_buffer.qsize() != 0:
            tmp_data = nmea_gga_log_buffer.get()
            tmp_data = tmp_data.decode('ascii')
            self.tmp_nmea_gga_log += '\r\n{}'.format(tmp_data)
            self.tmp_nmea_gga_log_line_cnt += 1
            
            if self.tmp_nmea_gga_log_line_cnt == self.max_line:
                self.save_nmea_gga_log_chunk()
                self.initialize_nmea_gga_log_chunk()
    
    def update_nmea_log(self, nmea_log_buffer: Queue):
        while nmea_log_buffer.qsize() != 0:
            tmp_data = nmea_log_buffer.get()
            tmp_data = tmp_data.decode('ascii')
            self.tmp_nmea_log += '\r\n{}'.format(tmp_data)
            self.tmp_nmea_log_line_cnt += 1
            
            if self.tmp_nmea_log_line_cnt == self.max_line:
                self.save_nmea_log_chunk()
                self.initialize_nmea_log_chunk()
    
    def update_rtcm_log(self, rtcm_log_buffer: Queue):
        while rtcm_log_buffer.qsize() != 0:
            tmp_data = rtcm_log_buffer.get()
            self.tmp_rtcm_log += tmp_data
            self.tmp_rtcm_log_byte_length += len(tmp_data)
            
            if self.tmp_rtcm_log_byte_length >= self.max_bytes:
                self.save_rtcm_log_chunk()
                self.initialize_rtcm_log_chunk()
                
    def update_gnss_info_log(self, gnss_info_log_buffer: Queue):
        while gnss_info_log_buffer.qsize() != 0:
            tmp_data = gnss_info_log_buffer.get()
            self.tmp_gnss_info_log.iloc[self.tmp_gnss_info_log_line_cnt] = \
                tmp_data.items()
                
            if self.tmp_gnss_info_log_line_cnt == self.max_line:
                self.save_gnss_info_log_chunk()
                self.initialize_gnss_info_log_chunk()
            
    def update_localization_info_log(self, localization_info_log_buffer: Queue):
        while localization_info_log_buffer.qsize() != 0:
            tmp_data = localization_info_log_buffer.get()
            self.tmp_localization_info_log.iloc[self.tmp_localization_info_log_line_cnt] = \
                tmp_data.items()
            
            if self.tmp_localization_info_log_line_cnt == self.max_line:
                self.save_localization_info_log_chunk()
                self.initialize_localization_info_log_chunk()
                
    def run(self, rx_log_buffer: Queue, nmea_gga_log_buffer: Queue, nmea_log_buffer: Queue,
            rtcm_log_buffer: Queue, localization_info_log_buffer: Queue, 
            gnss_info_log_buffer: Queue, cmd_queue: Queue):
        
        rospy.loginfo('[GNSS] Data logger thread started.')
        
        self.create_dummy_dataframe_chunk()
        
        self.initialize_data_log_chunk()
        
        op = True
        try:
            while op and not rospy.is_shutdown():
                self.update_rx_log(rx_log_buffer)
                self.update_nmea_gga_log(nmea_gga_log_buffer)
                self.update_nmea_log(nmea_log_buffer)
                self.update_rtcm_log(rtcm_log_buffer)
                self.update_gnss_info_log(gnss_info_log_buffer)
                self.update_localization_info_log(localization_info_log_buffer)
                
                time.sleep(1)
                op = self.set_operation(cmd_queue)
                    
        finally:
            if self.tmp_rx_log_byte_length != 0:
                self.save_rx_log_chunk()
            if self.tmp_nmea_gga_log_line_cnt != 0:
                self.save_nmea_gga_log_chunk()
            if self.tmp_nmea_log_line_cnt != 0:
                self.save_nmea_log_chunk()
            if self.tmp_rtcm_log_byte_length != 0:
                self.save_rtcm_log_chunk()
            if self.tmp_localization_info_log_line_cnt != 0:
                self.save_rtcm_log_chunk()
            if self.tmp_gnss_info_log_line_cnt != 0:
                self.save_gnss_info_log_chunk()
                
            rospy.loginfo('[GNSS] Data logger thread terminated.')
    
    