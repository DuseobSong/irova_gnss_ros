import os, sys, time
import serial
import threading
import numpy as np
import pandas as pd
import datetime
import select
import struct
import rospkg


PI = 3.14159265359
# column index
DATE    = 0
TIME    = 1
LAT     = 2
LON     = 3
MODE    = 4
CNO     = 5
ALT     = 6
ROLL    = 7
PITCH   = 8
YAW     = 9
GPS_VAL = 10
RSSI    = 11
BAT     = 12
ACC_N   = 13
ACC_E   = 14
ACC_D   = 15
VEL_N   = 16
VEL_E   = 17
VEL_D   = 18

serial_struct = struct.Struct("<BHI16sHBBI3Bb3Bf2H3B2qiB9f4B")

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
    
def deg2rad(deg):
    return deg * PI / 180.0

def rad2deg(rad):
    return rad * 180.0 / PI
    
rp = rospkg.RosPack()
root_dir = rp.get_path('gnss_ros')
    
    
class Node:
    def __init__(self, device, port, baudrate, display_rawdata=False):
        self.device             = device
        self.port               = port
        self.baudrate           = baudrate
        self.display_rawdata    = display_rawdata
        
        self.key_list   = []
        
        self.ser        = None
        
        self.rx_buffer  = bytearray()
        
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
            
        self.cmd        = None
        
        self.operation  = True
        
        # Data
        self.serial_number = None
        self.attrs      = ['DATE', 'TIME', 'MODE', 'CNO', 'LAT', 'LON', 'ALT', 'ROLL', 'PITCH', 'YAW', 'GPS_VAL',
                           'RSSI', 'BAT', 'ACC_N', 'ACC_E', 'ACC_D', 'VEL_N', 'VEL_E', 'VEL_D']
        self.date       = 0.0
        self.time       = 0.0
        self.latitude   = 0.0
        self.longitude  = 0.0
        self.altitude   = 0.0
        self.roll       = 0.0
        self.pitch      = 0.0
        self.yaw        = 0.0
        self.gps_val    = 0.0
        self.rssi       = 0.0
        self.bat        = 0.0
        self.acc_n      = 0.0
        self.acc_e      = 0.0
        self.acc_d      = 0.0
        self.vel_n      = 0.0
        self.vel_e      = 0.0
        self.vel_d      = 0.0
        
        self.cno        = 0.0
        self.mode       = 0.0
        
        self.max_lines  = 1000
        self.line_no    = 0
        self.db         = np.zeros((self.max_lines, len(self.attrs)), dtype=float)
        self.dbs        = []
        
        self.cnt        = 1
        
        self.lock       = threading.Lock()
        
        self.input_thread   = threading.Thread(target=self.input_task,  args=())
        self.uart_thread    = threading.Thread(target=self.uart_task,   args=())
        
        
    def connect(self):
        retry_cnt = 0
        ret = False
        
        while True:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001
            )
            
            time.sleep(0.05)
            
            if self.ser.isOpen():
                ret = True
                break
            else:
                try:
                    self.ser.close()
                except:
                    pass
                retry_cnt += 1
                
                if retry_cnt == 10:
                    break
            
        return ret
    
    def display(self):
        if self.device == 'FITO':
            rcv = self.read_until_FITO()
            # print(rcv)
            self.display_data(rcv)
        elif self.device == 'PPS':
            lines = []
            tmp_rcv = self.ser.read_all()
            lines = tmp_rcv.split(b'\r\n')
            
            # split lines here
            # start_idx = 0
            # for i, b in enumerate(tmp_rcv):
            #     if b == b'\n':
            #         lines.append(tmp_rcv[start_idx:i+1])
            #         start_idx = i + 1
            
            for l in lines:
                if len(l) != 0:
                    print(l)
                    
    def parse(self):
        self.rx_buffer += self.ser.readall()
        while len(self.rx_buffer) > 114:
            
            start_idx = -1
            search_range = len(self.rx_buffer) - 110
            for i in range(search_range):
                if self.rx_buffer[i:i+4] == b'FITO':
                    start_idx = i+4
                    break
            if start_idx != -1:
                tmp_msg = self.rx_buffer[start_idx: start_idx + 110]
                self.rx_buffer = self.rx_buffer[start_idx+110:]
                # print(len(tmp_msg))
                unpacked = serial_struct.unpack(tmp_msg)
                if self.serial_number is None:
                    self.serial_number = unpacked[3].decode()
                # self.serial_number = unpacked[3]
                year = unpacked[4] * 1.0 
                month = unpacked[5] * 1.0
                day = unpacked[6] * 1.0
                self.date = year * 10**4 + month * 100 + day
                self.time = unpacked[7]*1.0
                self.mode = unpacked[9] * 1.0
                self.cno = unpacked[10] * 1.0
                
                self.rssi = unpacked[11] * 1.0
                self.bat = unpacked[12] * 1.0
                self.gps_val = unpacked[14] * 1.0
                self.latitude = unpacked[21] * 1e-10
                self.longitude = unpacked[22] * 1e-10
                self.altitude = unpacked[23] * 1e-3
                self.roll = unpacked[25]
                self.pitch = unpacked[26]
                self.yaw = unpacked[27]
                self.acc_n = unpacked[28]
                self.acc_e = unpacked[29]
                self.acc_d = unpacked[30]
                self.vel_n = unpacked[31]
                self.vel_e = unpacked[32]
                self.vel_d = unpacked[33]
            else:
                self.rx_buffer = self.rx_buffer[search_range:]
            
            if self.cnt == 10:
                print('[FITO] {}.{}: {}, {}, {}, {}, {}, {}, {}\r'.format(self.date, self.time, self.cno, self.latitude, self.longitude, self.altitude, self.roll, self.pitch, self.yaw))
                self.cnt = 1
            else:
                self.cnt += 1
        
    def read_until_FITO(self):
        line = b''
        while True:
            char = self.ser.read(1)
            line += char
            if line.endswith(b'FITO'):
                break
        return self.ser.read(110)
        
    def display_data(self, rcv):
        if len(rcv) < 110:
            return
        unpacked = serial_struct.unpack(rcv)
            
        if self.serial_number is None:
            self.serial_number = unpacked[3].decode()
        year = unpacked[4] * 1.0 
        month = unpacked[5] * 1.0
        day = unpacked[6] * 1.0
        self.date = year * 10**4 + month * 100 + day
        self.time = unpacked[7]*1.0
        self.mode = unpacked[9] * 1.0
        self.cno = unpacked[10] * 1.0
        
        self.rssi = unpacked[11] * 1.0
        self.bat = unpacked[12] * 1.0
        self.gps_val = unpacked[14] * 1.0
        self.latitude = unpacked[21] * 1e-10
        self.longitude = unpacked[22] * 1e-10
        self.altitude = unpacked[23] * 1e-3
        self.roll = unpacked[25]
        self.pitch = unpacked[26]
        self.yaw = unpacked[27]
        self.acc_n = unpacked[28]
        self.acc_e = unpacked[29]
        self.acc_d = unpacked[30]
        self.vel_n = unpacked[31]
        self.vel_e = unpacked[32]
        self.vel_d = unpacked[33]
        
        self.db[self.line_no, :] = [self.date, self.time, self.mode, self.cno, self.latitude, self.longitude, self.altitude, 
                                    self.roll, self.pitch, self.yaw, self.gps_val, self.rssi, self.bat,
                                    self.acc_n, self.acc_e, self.acc_d, self.vel_n, self.vel_e, self.vel_d]
        
        if self.cnt == 10:
            print('[FITO] {}.{}: {}, {}, {}, {}, {}, {}, {}'.format(self.date, self.time, self.cno, self.latitude, self.longitude, self.altitude, self.roll, self.pitch, self.yaw))
            self.cnt = 1
        else:
            self.cnt += 1
            
    def parse_message(self):
        # split lines
        if self.device == 'FITO':
            self.rx_buffer += self.ser.readall()
            
            while len(self.rx_buffer) > 114:
                start_idx = -1
                search_range = len(self.rx_buffer) - 110
                
                for i in range(search_range):
                    if self.rx_buffer[i:i+4] == b'FITO':
                        start_idx = i+4
                        break
                    
                if start_idx != -1:
                    rcv = self.rx_buffer[start_idx: start_idx+110]
                    self.rx_buffer = self.rx_buffer[start_idx+110:]
             
                    unpacked = serial_struct.unpack(rcv)
                    
                    if self.serial_number is None:
                        self.serial_number = unpacked[3].decode()
                    year = unpacked[4] * 1.0 
                    month = unpacked[5] * 1.0
                    day = unpacked[6] * 1.0
                    self.date = year * 10**4 + month * 100 + day
                    self.time = unpacked[7]*1.0
                    self.mode = unpacked[9] * 1.0
                    self.cno = unpacked[10] * 1.0
                    
                    self.rssi = unpacked[11] * 1.0
                    self.bat = unpacked[12] * 1.0
                    self.gps_val = unpacked[14] * 1.0
                    self.latitude = unpacked[21] * 1e-10
                    self.longitude = unpacked[22] * 1e-10
                    self.altitude = unpacked[23] * 1e-3
                    self.roll = unpacked[25]
                    self.pitch = unpacked[26]
                    self.yaw = unpacked[27]
                    self.acc_n = unpacked[28]
                    self.acc_e = unpacked[29]
                    self.acc_d = unpacked[30]
                    self.vel_n = unpacked[31]
                    self.vel_e = unpacked[32]
                    self.vel_d = unpacked[33]
                    
                    self.db[self.line_no, :] = [self.date, self.time, self.mode, self.cno, self.latitude, self.longitude, self.altitude, 
                                                self.roll, self.pitch, self.yaw, self.gps_val, self.rssi, self.bat,
                                                self.acc_n, self.acc_e, self.acc_d, self.vel_n, self.vel_e, self.vel_d]
                    
                    if self.cnt == 100:
                        print('[FITO] {}.{}: {}, {}, {}, {}, {}, {}, {}, {}\r'.format(self.date, self.time, self.mode, self.cno, self.latitude, self.longitude, self.altitude, self.roll, self.pitch, self.yaw))
                        self.cnt = 1
                    else:
                        self.cnt += 1
                    self.line_no += 1
                    if self.line_no == self.max_lines:
                        tmp_df = pd.DataFrame(self.db, columns=self.attrs)
                        self.dbs.append(tmp_df)
                        self.line_no = 0
                else:
                    self.rx_buffer = self.rx_buffer[search_range:]
                
        elif self.device == 'PPS':
            if self.serial_number is None:
                self.serial_number = 'RTK-B39-0001'
            rcv = self.rx_buffer + self.ser.read_all()
            lines = rcv.split(b'\r\n')
            self.rx_buffer = lines[-1]
            lines = lines[:-1]
            
            for l in lines:
                if l[3:6] == b'GGA':
                    print(l)
                    l               = l.split(b'*')[0]
                    elements        = l.split(b',')
                    if elements[1] != b'':
                        self.time       = float(elements[1])
                    else:
                        self.time = 0.0
                    if elements[2] != b'':
                        self.latitude   = float(elements[2])
                    else:
                        self.latitude = 0.0
                    if elements[3] != b'':
                        self.longitude  = float(elements[4])
                    else:
                        self.longitude = 0.0
                    if elements[6] != b'':
                        self.gps_val    = float(elements[6])
                    else:
                        self.gps_val = 0.0
                    if elements[9] != b'':
                        self.altitude   = float(elements[9])
                    else:
                        self.altitude = 0.0
                        
                elif l[3:6] == b'RMC':
                    print(l)
                    l               = l.split(b'*')[0]
                    elements        = l.split(b',')
                    if elements[1] != b'':
                        self.time       = float(elements[1])
                    else:
                        self.time = 0.0
                    if elements[3] != b'':
                        self.latitude   = float(elements[3])
                    else:
                        self.latitude = 0.0
                    if elements[5] != b'':
                        self.longitude  = float(elements[5])
                    else:
                        self.longitude = 0.0
                    if elements[9] != b'':
                        self.date       = float(elements[9])
                    else:
                        self.date = 0.0
                    if elements[10] != b'':
                        self.gps_val    = float(elements[10])
                    else:
                        self.gps_val = 0.0
                    
                if self.time + self.date + self.latitude + self.longitude + self.gps_val + self.altitude == 0.0:
                    pass
                else:
                    # save in db here
                    self.db[self.line_no, :] = [self.date, self.time, self.mode, self.cno, self.latitude, self.longitude, self.altitude, 
                                                self.roll, self.pitch, self.yaw, self.gps_val, self.rssi, self.bat, 
                                                self.acc_n, self.acc_e, self.acc_d, self.vel_n, self.vel_e, self.vel_d]
                    self.line_no += 1
                
                # check if sub db is full
                if self.line_no == self.max_lines:
                    tmp_df = pd.DataFrame(self.db, columns=self.attrs)
                    self.dbs.append(tmp_df)
                    self.line_no = 0
    
    def save_data(self):
        date = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        if self.line_no != 0:
            tmp_df = pd.DataFrame(self.db[:self.line_no, :], columns=self.attrs)
            self.dbs.append(tmp_df)
        print('save data... ({} DBs)'.format(len(self.dbs)))
        for i, db in enumerate(self.dbs):
            print('progress... ({}/{})'.format(i+1, len(self.dbs)))
            db.to_csv(root_dir + '/scripts/{}/{}_{}_{}.csv'.format(self.device, self.serial_number, date, i),
                      index=False)
    
    def uart_task(self):
        if self.connect():
            print('[SYSTEM] Serial connected.')
            
            op = True
            
            print('[SERIAL] thread start.')
            start_time = time.time()
            
            while op:
                self.lock.acquire()
                op = self.operation
                self.lock.release()
                
                # self.rx_buffer += self.ser.read_all()
                if self.display_rawdata:
                    # self.display()
                    self.parse()
                else:
                    self.parse_message()
                    
                if time.time() - start_time >= 60 * 5: # 5 min.
                    self.lock.acquire()
                    self.operation = False
                    self.lock.release()
                    break
                time.sleep(0.05)
            self.ser.close()
            if not self.display_rawdata:
                self.save_data()
        else:
            print('[SYSTEM] Failed to open serial port.')
            self.lock.acquire()
            self.operation = False
            self.lock.release()
        
    def get_key(self):
        if os.name == 'nt':
            timeout = 0.01
            startTime = time.time()
            
            while True:
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif time.time() - startTime > timeout:
                    return ''
        else:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
                
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        
    def input_task(self):
        op = True
        print('[INPUT] thread start.')
        self.settings = termios.tcgetattr(sys.stdin)
        
        while op:
            self.lock.acquire()
            op = self.operation
            self.lock.release()
            
            key = self.get_key()
            
            if key == 'q':
                self.lock.acquire()
                self.operation = False
                self.lock.release()
                
                break
            else:
                pass
            
            time.sleep(0.001)
            
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
    def run(self):
        try:
            self.input_thread.start()
            self.uart_thread.start()
        finally:
            if self.input_thread.is_alive():
                self.input_thread.join()
            if self.uart_thread.is_alive():
                self.uart_thread.join()
                

if __name__ == '__main__':
    DEVICE      = 'FITO' # FITO: FtiTogether, PPS: RTK2U
    # DEVICE      = 'PPS'
    if DEVICE == 'FITO':
        PORT    = '/dev/fitogether_rtk_gt'
    elif DEVICE == 'PPS':
        PORT    = 'COM4'
    BAUDRATE    = 115200
    RAWDATA     = False
    
    node = Node(
        device=DEVICE,
        port=PORT,
        baudrate=BAUDRATE,
        display_rawdata=RAWDATA
        )
    
    node.run()
        
