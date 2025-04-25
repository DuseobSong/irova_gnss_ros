import os, sys
import pyproj
import time
import json
import numpy as np
from queue import Queue

import rospy
from rospkg import RosPack

rp = RosPack()
ROOT_PKG_DIR = rp.get_path('irova_gnss_ros')

def deg2rad(deg):
    return deg * np.pi / 180.0


class GNSSLocalization:
    def __init__(self, parameters):
        self.parameters         = parameters
        
        self.epsg               = None
        self.epsg_proj4_path    = ROOT_PKG_DIR + '/config/EPSG.json'
        self.epsg_string_from   = None
        self.epsg_string_to     = None
        
        self.UTC                = None
        self.n_indicator        = None
        self.e_indicator        = None
        self.latitude           = 0.0
        self.longitude          = 0.0
        self.altitude           = 0.0
        self.origin_latitude    = None
        self.origin_longitude   = None
        self.prev_e             = None
        self.prev_n             = None
        self.e                  = None
        self.n                  = None
        self.orientation        = 0.0 # Radian, Velocity vector direction
        self.pred_e             = 0.0
        self.pred_n             = 0.0
        self.pred_orientation   = 0.0
        self.ve                 = 0.0
        self.vn                 = 0.0
        self.ae                 = 0.0
        self.an                 = 0.0
        self.cur_timestamp      = 0.0
        self.prev_timestamp     = 0.0
        self.dt                 = 0.0
        
        self.proj_from          = None
        self.proj_to            = None
        
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
        
        self.set_parameters()
        
    def dm2deg(self, dm):
        return dm // 1e2 + (dm % 1e2) / 60.0
    
    def set_operation(self, cmd_queue: Queue):
        if cmd_queue.qsize() != 0:
            return False
        else:
            return True
    
    def set_parameters(self):
        self.epsg = self.parameters["EPSG"]
        if 'ORIGIN_LATITUDE' in self.parameters.keys():
            self.origin_latitude = self.parameters["ORIGIN_LATITUDE"]
            self.origin_longitude = self.parameters["ORIGIN_LONGITUDE"]
            
        if self.load_epsg():
            return True
        else:
            return False
    
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
        
    def get_gnss_info(self, gnss_info_buffer: Queue):
        if gnss_info_buffer.qsize() != 0:
            while gnss_info_buffer.qsize() != 0:
                record = gnss_info_buffer.get()
            
            self.UTC            = record["UTC"]
            self.longitude      = record["LONGITUDE"]
            self.n_indicator    = record["N_INDICATOR"]
            self.latitude       = record["LATITUDE"]
            self.e_indicator    = record["E_INDICATOR"]
            self.altitude       = record["ALTITUDE"]
            
            if self.origin_latitude is None:
                self.origin_latitude = self.latitude
                self.origin_longitude = self.longitude
            
            return True
        else:
            return False
            
    def convert_coordinate(self):
        e, n = pyproj.transform(self.proj_from, self.proj_to, self.longitude, self.latitude)
        self.kalman_filter(filt="LKF")
        
        self.orietnation = self.compute_orientation(e, n)
        self.pred_orientation = self.compute_orientation(self.pred_e, self.pred_n)
        
        self.compute_velocity()
        self.prev_e = self.e
        self.prev_n = self.n
        self.e      = e
        self.n      = n
        
    def compute_velocity(self):
        if self.prev_timestamp != 0.0:
            delta_t      = self.cur_timestamp - self.prev_timestamp
            delta_e      = self.e - self.prev_e
            delta_n      = self.n - self.prev_n
            self.prev_ve = self.ve
            self.prev_vn = self.vn
            self.ve      = delta_e / delta_t
            self.vn      = delta_n / delta_t
            
    def compute_orientation(self, e, n):
        ret = 0.0
        
        if self.prev_e is None:
            ret = 0.0
        else:
            delta_e = e - self.prev_e
            delta_n = n - self.prev_n
                
            # Direct north as 0 degree
            if delta_e == 0 and delta_n == 0:
                ret = deg2rad(0.0)
            elif delta_e == 0:
                if delta_n > 0:
                    ret = deg2rad(0.0)
                elif delta_n <0:
                    ret = deg2rad(-180.0)
            elif delta_n == 0:
                if delta_e > 0:
                    ret = deg2rad(270.0)
                elif delta_e < 0:
                    ret = deg2rad(90.0)
            else:
                ret = np.arctan2(delta_n, delta_e)
        return ret
            
    def kalman_filter(self, filt="LKF"):
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
        if filt == "LKF": # Linear Kalman Filter
            X = np.array([self.prev_e, self.prev_ve, self.prev_n, self.prev_vn]).T # state
            Z = np.array([self.e, self.ve, self.n, self.vn]).T # measurement
            
            Xp = self.A.dot(X)
            Pp = self.A.dot(self.P).dot(self.A.T) + self.Q
            
            K = Pp.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R))
            X = Xp + K.dot(Z - self.H.dot(Xp))
            self.P = Pp - K.dot(self.H).dot(Pp)
            
            self.pred_e = X[0]
            self.pred_n = X[2]
            
        elif filt == "EKF": # Extended Kalman Filter
            pass
        
        elif filt == "UKF": # Uncented Kalman Filter
            pass
        
        
    def update_localization_info(self, localization_info_buffer: Queue, localization_info_log_buffer: Queue):
        record = {
            "UTC": self.UTC,
            "LATITUDE": self.latitude,
            "N_INDICATOR": self.n_indicator,
            "LONGITUDE": self.longitude,
            "E_INDICATOR": self.e_indicator,
            "ALTITUDE": self.altitude,
            "V_E": self.ve,
            "V_N": self.vn,
            "MEA_E": self.e,
            "MEA_N": self.n,
            "MEA_TH": self.orientation,
            "KAL_E": self.pred_e,
            "KAL_N": self.pred_n,
            "KAL_TH": self.pred_orientation
            }
        
        localization_info_buffer.put(record)
        if localization_info_log_buffer is not None:
            localization_info_log_buffer.put(record)
        
    def run(self, 
            gnss_info_buffer: Queue,
            localization_info_buffer: Queue,
            localization_info_log_buffer: Queue,
            cmd_queue: Queue):
        op = True
        rospy.loginfo('[GNSS] GNSS-Serial thread started.')
        
        try:
            while op and not rospy.is_shutdown():
                if self.get_gnss_info(gnss_info_buffer):
                    self.convert_coordinate()
                    
                    self.update_localization_info(localization_info_buffer, localization_info_log_buffer)
                
                time.sleep(0.05)
                
                op = self.set_operation(cmd_queue)
        finally:
            rospy.loginfo('[GNSS] GNSS-Serial thread terminated.')
        