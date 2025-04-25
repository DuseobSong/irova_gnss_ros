import os, sys
import json
import time
from queue import Queue
import rospy

from rospkg import RosPack

# Directional indicator
E = 0
W = 1
N = 2
S = 3

# GPS quality indicator
FIX_UNAVAILABLE = 1
FIX_VALID_SPS = 2
FIX_VALID_DGPS = 2

# Mode
MANUAL = 'M'
AUTO = 'A'

# Fix type
FIX_2D = 2
FIX_3D = 3

# Status
WARNING = 'V'
DATA_VALID = 'A'

# RMC mode
DATA_INVALID = 'N'
AUTO = 'A'
DIFF = 'D'
ESTIMATE = 'E'

# NMEA header
START_CH = 0x23
END_CH_1 = 0x0D
END_CH_2 = 0x0A

rp = RosPack()
ROOT_PKG_DIR = rp.get_path('irova_gnss_ros')


class NMEAParser:
    def __init__(self, parameters):
        self.parameters                 = parameters
        self.nmea_info_dir              = ROOT_PKG_DIR + '/config/NMEA.json'
        self.nmea_info                  = None
        
        self.rx_buffer                  = bytearray()
        self.nmea_queue                 = []
        self.gga_queue                  = []
        
        self.UTC                        = None
        self.latitude                   = None
        self.longitude                  = None
        self.altitude                   = None
        self.n_flag                     = None
        self.e_flag                     = None
        
        self.gps_quality                = None
        self.mode                       = None
        self.fix_type                   = None
        self.status                     = None
        
        self.operation                  = True
        self.gnss_info_update_flag   = False
        
    def load_nmea_info(self):
        try:
            with open(self.nmea_info_dir) as f:
                self.nmea_info = json.loads(f.read())
            return True
        except Exception as e:
            rospy.logerr('[NMEA] Parser loading failed: {}'.format(e))
            return False
        
    def checksum(self, string):
        val = 0
        for c in string:
            val = val ^ ord(c)
        return val
    
    def byte_chkcksum(self, barray):
        val = 0
        for b in barray:
            val = val ^ b
        return val
    
    def set_operation(self, cmd_queue: Queue):
        if cmd_queue.qsize() != 0:
            return False
        else: 
            return True
        
    def retrieve_rx_bytes(self, rx_buffer: Queue):
        initial_length = len(self.rx_buffer)
        
        while True:
            if rx_buffer.empty():
                break
            tmp_bytes = rx_buffer.get()
            if len(tmp_bytes) > 0:
                self.rx_buffer.append(tmp_bytes)
            if rx_buffer.qsize() == 0:
                break
        
        return len(self.rx_buffer) - initial_length
    
    def search_nmea_message_packet(self, barray: bytearray):
        msg_found = False
        rest = None
        
        messages = []
        
        while True:
            start_idx = barray.find(b'$')
            end_idx = barray.find(b'\r\n')
            
            if start_idx != -1 and end_idx != -1:
                messages.append(barray[start_idx: end_idx])
                barray = barray[end_idx + 2:]
                msg_found = True
                continue
            else:
                break
                
        rest = barray
        
        # start_idx = -1
        # end_idx = -1
        # last_end_idx = -1
        
        # for i, b in enumerate(barray):
        #     if b == START_CH:
        #         start_idx = i
        #     elif i != 0:
        #         if barray[i-1] == END_CH_1 and barray[i] == END_CH_2:
        #             # checksum function here
        #             if start_idx != -1:
        #                 end_idx = i
        #                 last_end_idx = end_idx
            
        #     if start_idx != -1 and end_idx != -1:
        #         messages.append(barray[start_idx, end_idx+1])
        #         msg_found = True
        #         start_idx = -1
        #         end_idx = -1                

        # if not msg_found:
        #     if start_idx != -1:
        #         rest = barray[start_idx:]
        #     else:
        #         rest = barray
        # else:
        #     rest = barray[last_end_idx+1:]

        return msg_found, messages, rest # bool, list(bytearray), bytearray
    
    def split_message_packet(self, barray: bytearray):
        header, tail = barray.split(b'*')
        ret = False
        head = None
        payload = None
        
        if tail[:2] == self.byte_chkcksum(header[1:]):
            ret = True
            
        if ret:
            msg_fields = header.decode('ascii').split(',')

            head = msg_fields[0]
            payload = msg_fields[1:]
            
        return ret, head, payload
    
    def check_UTC_update(self, tmp_UTC):
        '''
        UTC format: hhmmss.sss
        '''
        ret = True
        prev_UTC    = self.UTC
        prev_H      =  prev_UTC // 1e4
        prev_M      = (prev_UTC %  1e4) // 1e2
        prev_S      = (prev_UTC %  1e2) // 1e0
        prev_MS     = (prev_UTC % 1e0)
        
        tmp_H       =  tmp_UTC // 1e4
        tmp_M       = (tmp_UTC %  1e4) // 1e2
        tmp_S       = (tmp_UTC %  1e2) // 1e0
        tmp_MS      = (tmp_UTC % 1e0)
        
        print(prev_H, tmp_H)
        print(prev_M, tmp_M)
        print(prev_S, tmp_S)
        print(prev_MS, tmp_MS)
        
        if prev_H != tmp_H:
            if prev_H > tmp_H and prev_H != 23 and tmp_H != 0:
                ret = False
        else:
            if prev_M != tmp_M:
                if prev_M > tmp_M and prev_M != 59 and tmp_M != 0:
                    ret = False
            else:
                if prev_S != tmp_S:
                    if prev_S > tmp_S and prev_S != 59 and tmp_S != 0:
                        ret = False
                else:
                    if prev_MS > tmp_MS:
                        ret = False
                        
        return ret        
       
    def parse_gga_message(self, payload: list):
        tmp_UTC             = float(payload[0])
        
        if self.UTC is None or self.check_UTC_update(tmp_UTC):
            self.UTC            = tmp_UTC
            self.latitude       = float(payload[1])
            self.n_indicator    = payload[2]
            self.lon            = float(payload[3])
            self.e_indicator    = payload[3]
            self.gps_quality    = payload[4]
            self.altitude       = float(payload[7])
            
            self.gnss_info_update_flag = True
        
    def parse_gll_message(self, payload: list):
        tmp_UTC            = float(payload[4])
        
        if self.UTC is None or self.check_UTC_update(tmp_UTC):
            tmp_UTC             = tmp_UTC
            self.latitude       = float(payload[0])
            self.n_indicator    = payload[1]
            self.longitude      = float(payload[2])
            self.e_indicator    = payload[3]
            
            self.gnss_info_update_flag = True
        
    def parse_gsa_message(self, payload: list):
        self.fix_type = payload[1]
        
    def parse_rmc_message(self, payload: list):
        tmp_UTC             = float(payload[0])
        
        if self.UTC is None or self.check_UTC_update(tmp_UTC):
            self.UTC            = tmp_UTC
            self.status         = payload[1]
            self.latitude       = float(payload[2])
            self.n_indicator    = payload[3]
            self.longitude      = float(payload[4])
            self.e_indicator    = payload[5]

            self.gnss_info_update_flag = True
        
    def parse_NMEA_message(self, nmea_gga_buffer: Queue, nmea_buffer: Queue,
                           nmea_gga_log_buffer: Queue, nmea_log_buffer: Queue):
        msg_found, messages, rest = self.search_nmea_message_packet(self.rx_buffer) 
        self.rx_buffer = rest
        if msg_found:
            for msg in messages:
                valid_msg, msg_type, payload = self.split_message_packet(msg)
                if valid_msg:
                    if msg_type.endswith("GGA"):
                        nmea_gga_buffer.put(msg)    # msg: bytearray
                        if nmea_gga_log_buffer is not None:
                            nmea_gga_log_buffer.put(msg)
                        nmea_buffer.put(msg)        # msg: bytearray
                        if nmea_log_buffer is not None:
                            nmea_log_buffer.put(msg)
                        self.parse_gga_message(payload)
                    elif msg_type.endswith('GLL'):
                        nmea_buffer.put(msg)
                        if nmea_log_buffer is not None:
                            nmea_log_buffer.put(msg)
                        self.parse_gll_message(payload)
                    elif msg_type.endswith('GSA'):
                        nmea_buffer.put(msg)
                        if nmea_log_buffer is not None:
                            nmea_log_buffer.put(msg)
                        self.parse_gsa_message(payload)
                    elif msg_type.endswith('RMC'):
                        nmea_buffer.put(msg)
                        if nmea_log_buffer is not None:
                            nmea_log_buffer.put(msg)    
                        self.parse_rmc_message(payload)
                    else:
                        pass
                
    def update_gnss_info(self, gnss_info_buffer: Queue, gnss_info_log_buffer: Queue):
        record = {"UTC": self.UTC, 
                  "LONGITUDE": self.longitude, 
                  "N_INDICATOR": self.n_indicator, 
                  "LATITUDE": self.latitude, 
                  "E_INDICATOR": self.e_indicator,
                  "ALTITUDE": self.altitude}
        
        gnss_info_buffer.put(record)
        if gnss_info_log_buffer is not None:
            gnss_info_log_buffer.put(record)
        self.gnss_info_update_flag = False
        
    def run(self, rx_buffer: Queue, 
            nmea_gga_buffer: Queue, 
            nmea_gga_log_buffer: Queue,
            nmea_buffer: Queue, 
            nmea_log_buffer: Queue,
            gnss_info_buffer: Queue,
            gnss_info_log_buffer: Queue,
            cmd_queue: Queue):
        
        op = True
        rospy.loginfo("[GNSS] NMEA parser thread started.")
        try:
            while op and not rospy.is_shutdown():
                if self.retrieve_rx_bytes(rx_buffer) > 10:
                    self.parse_NMEA_message(nmea_gga_buffer, nmea_buffer, nmea_gga_log_buffer, nmea_log_buffer)
                
                if self.gnss_info_update_flag:
                    self.update_gnss_info(gnss_info_buffer, gnss_info_log_buffer)
                
                time.sleep(0.01)
                
                op = self.set_operation(cmd_queue)
        finally:
            rospy.loginfo("[GNSS] NMEA parser thread terminated.")