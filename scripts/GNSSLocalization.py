import os, sys
import pyproj
import time
import json
import numpy as np
from queue import Queue


class GNSSLocalization:
    def __init__(self, epsg, epsg_proj4_path):
        self.epsg = epsg
        self.epsg_proj4_path = epsg_proj4_path
        self.epsg_string_from = None
        self.epsg_string_to = None
        
        self.UTC = None
        self.n_indicator = None
        self.e_indicator = None
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.origin_latitude = 0.0
        self.origin_longitude = 0.0
        self.prev_e = 0.0
        self.prev_n = 0.0
        self.e = 0.0
        self.n = 0.0
        self.prev_e = 0.0
        self.prev_n = 0.0
        self.pred_e = 0.0
        self.pred_n = 0.0
        self.orientation = 0.0 # Radian, Velocity vector direction
        self.ve = 0.0
        self.vn = 0.0
        self.ae = 0.0
        self.an = 0.0
        self.cur_timestamp = 0.0
        self.prev_timestamp = 0.0
        self.dt = 0.0
        
        self.proj_from = None
        self.proj_to = None
        
        # Kalman filter
        self.A = np.array([[1, 1, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 1],
                           [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0],
                           [0, 0, 1, 0]])
        self.Q = 0.01 * np.eye(4)
        self.R = np.eye(4) *0.0625
        self.P = 4 * np.eye(4) # initial state covariance matrix
        self.X = np.array([0,0,0,0]).T
        
    def dm2deg(self, dm):
        return dm // 1e2 + (dm % 1e2) / 60.0
    
    def load_epsg(self):
        with open(self.epsg_proj4_path) as f:
            epsg_data = json.loads(f.read())
        if self.epsg in epsg_data["AVAILABLE_EPSG"]:
            self.epsg_string_from = epsg_data["EPSG_LIST"]["epsg4326"]["format"]
            self.epsg_string_to = epsg_data["EPSG_LIST"]["epsg{}".format(self.epsg)]["format"]
            self.proj_from = pyproj.Proj(self.epsg_string_from)
            self.proj_to = pyproj.Proj(self.epsg_string_to)
            return True
        else:
            print('[LOCALIZATION] Error - Invalid EPSG : {}'.format(self.epsg))
            return False
        
    def set_operation(self, cmd_queue: Queue):
        ret = True
        if cmd_queue.qsize() != 0:
            ret = False
        return ret
    
    def set_timestamp(self, utc):
        self.prev_timestamp = self.cur_timestamp
        self.cur_timestamp = utc
        
        prev_h = self.prev_timestamp // 1e4
        prev_m = (self.prev_timestamp % 1e4) // 1e2
        prev_s = (self.prev_timestamp % 1e2) // 1e0
        prev_ms = self.prev_timestamp % 1e0
        
        cur_h = self.cur_timestamp // 1e4
        cur_m = (self.cur_timestamp % 1e4) // 1e2
        cur_s = (self.cur_timestamp % 1e2) // 1e0
        cur_ms = self.cur_timestamp % 1e0
        
        dd  = 0
        dh  = cur_h - prev_h
        dm  = cur_m - prev_m
        ds  = cur_s - prev_s
        dms = cur_ms - prev_ms
        
        if dms < 0:
            dms = dms % 1.0
            ds -= 1
        if ds < 0:
            ds = ds % 60
            ds -= 1
        if dm < 0:
            dm = dm % 60
            dh -= 1
        if dh < 0:
            dd += 1
            dh = dh % 24
            
        self.dt = dd * 24 * 60 * 60 + dh * 60 * 60 + dm * 60 + ds + dms
        
    def get_localization_info(self, gnss_info_buffer: Queue):
        if gnss_info_buffer.qsize() != 0:
            while gnss_info_buffer.qsize() != 0:
                data = gnss_info_buffer.get()
            
            self.UTC            = data[0]
            self.longitude      = data[1]
            self.n_indicator    = data[2]
            self.latitude       = data[3]
            self.e_indicator    = data[4]
            self.altitude       = data[5]
            
            return True
        else:
            return False
            
    def convert_coordinate(self):
        e, n = pyproj.transform(self.proj_from, self.proj_to, self.longitude, self.latitude)
        self.prev_e = self.e
        self.prev_n = self.n
        self.e      = e
        self.n      = n
        self.compute_velocity()
        
    def compute_velocity(self):
        if self.prev_timestamp != 0.0:
            dt      = self.cur_timestamp - self.prev_timestamp
            de      = self.e - self.prev_e
            dn      = self.n - self.prev_n
            self.prev_ve = self.ve
            self.prev_vn = self.vn
            self.ve = de / dt
            self.vn = dn / dt
            
    def kalman_filter(self):
        '''
        Input:
            - e
            - n
            - ve
            - vn
            
        Output:
            - pred_e
            - pred_n
        '''
        X = np.array([self.prev_e, self.prev_ve, self.prev_n, self.prev_vn]).T # state
        Z = np.array([self.e, self.ve, self.n, self.vn]).T # measurement
        
        Xp = self.A.dot(X)
        Pp = self.A.dot(self.P).dot(self.A.T) + self.Q
        
        K = Pp.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R))
        X = Xp + K.dot(Z - self.H.dot(Xp))
        self.P = Pp - K.dot(self.H).dot(Pp)
        
        self.pred_e = X[0]
        self.pred_n = X[1]
        
    def update_localization_info(self, localization_info_buffer: Queue, localization_info_log_buffer: Queue):
        record = (self.UTC, self.latitude, self.n_indicator, self.longitude, self.e_indicator, 
                  self.altitude, self.ve, self.vn, self.pred_e, self.pred_n)
        localization_info_buffer.put(record)
        localization_info_log_buffer.put(record)
        
    def run(self, 
            gnss_info_buffer: Queue,
            localization_info_buffer: Queue,
            localization_info_log_buffer: Queue,
            cmd_queue: Queue):
        op = True
        
        while op:
            if self.get_localization_info(gnss_info_buffer):
                self.convert_coordinate()
                
                self.kalman_filter()
                
                self.update_localization_info(localization_info_buffer, localization_info_log_buffer)
            
            time.sleep(0.05)
            op = self.set_operation(cmd_queue)
        