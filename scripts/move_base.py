#!/usr/bin/env python3

import rospy
import os, sys
import traceback

from geometry_msgs.msg import Twist

import threading
import socket
import numpy as np
from queue import Queue
import time

def deg2rad(deg):
    return deg * np.pi / 180.0


class MoveBase:
    def __init__(self):
        self.rate                   = rospy.Rate(20)
        self.Operation              = Queue()
        self.ReturnMsg              = Queue()
        self.Command                = Queue()
        
        self.cmd_vel_lin            = 0.0
        self.cmd_vel_ang            = 0.0
        
        self.cur_vel_lin            = 0.0
        self.cur_vel_ang            = 0.0
        
        self.vel_lin_max            = 0.7
        self.vel_lin_min            = -0.3
        
        self.vel_ang_max            = deg2rad(50.0)
        self.vel_ang_min            = deg2rad(-50.0)
        
        self.vel_lin_step           = 0.02
        self.vel_ang_step           = deg2rad(5.0)
        
        self.UDP_IP                 = None
        self.UDP_PORT_TX            = 25002
        self.UDP_PORT_RX            = 25001
        self.cmd_vel_pub            = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.accel_msg              = b'ROS remote command: accelerate'
        self.decel_msg              = b'ROS remote command: decelerate'
        self.turn_left_msg          = b'ROS remote command: turn left'
        self.turn_right_msg         = b'ROS remote command: turn right'
        self.halt_msg               = b'ROS remote command: halt'
        
        self.accel_resp             = b'ROS robot remote: Accelerate'
        self.decel_resp             = b'ROS robot remote: Decelerate'
        self.turn_left_resp         = b'ROS robot remote: turn left'
        self.turn_right_resp        = b'ROS robot remote: turn right'
        self.halt_resp              = b'ROS robot remote: halt'
        
        self.udp_tx_thread_args     = (self.ReturnMsg, 
                                       self.Operation,
                                       )
        self.udp_rx_thread_args     = (self.Command,  
                                       self.Operation,
                                       )
        self.control_thread_args    = (self.Command, 
                                       self.ReturnMsg, 
                                       self.Operation,
                                       )
        
        self.UDP_tx_thread          = None
        self.UDP_rx_thread          = None
        self.control_thread         = None
        
    def raise_error(self, node_name, exception):
        exc_type, exc_obj, exc_tb = sys.exc_info()
        
        if exc_tb is not None:
            line_no = exc_tb.tb_lineno
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        else:
            exc_type = ''
            line_no = ''
            fname = ''
            
        rospy.logerr('[{}] Error: ' + exception)
        print(exc_type, fname, line_no)
        print(traceback.format_exc())
        
    def set_configuration(self):
        self.user   = os.environ["USER"]
        self.EGO_IP = os.popen('hostname -I | tr -d " "').read().split('\n')[0]
        EGO_IP      = self.EGO_IP.split('.')
        self.UDP_IP = '{}.{}.{}.255'.format(EGO_IP[0], EGO_IP[1], EGO_IP[2])
    
    def constrain(self, input_vel, lower_b, upper_b):
        if input_vel < lower_b:
            input_vel = lower_b
        elif input_vel > upper_b:
            input_vel = upper_b
        else:
            input_vel = input_vel
            
        return input_vel
    
    def check_linear_limit_velocity(self, v):
        return self.constrain(v, self.vel_lin_min, self.vel_lin_max)
    
    def check_angular_limit_velocity(self, w):
        return self.constrain(w, self.vel_ang_min, self.vel_ang_max)
    
    def publish_cmd_vel_msg(self):
        msg             = Twist()
        msg.linear.x    = self.cmd_vel_lin
        msg.angular.z   = self.cmd_vel_ang
        self.cmd_vel_pub.publish(msg)
        
    def move(self, tmp_cmd, ReturnMsg: Queue):
        ret_msg = None
        if tmp_cmd == self.accel_msg:
            self.cmd_vel_lin    = self.check_linear_limit_velocity(self.cmd_vel_lin + self.vel_lin_step)
            ret_msg             = self.accel_resp + bytes(':({},{})'.format(self.cmd_vel_lin, self.cmd_vel_ang), 'ascii')
        elif tmp_cmd == self.decel_msg:
            self.cmd_vel_lin    = self.check_linear_limit_velocity(self.cmd_vel_lin - self.vel_lin_step)
            ret_msg             = self.decel_resp + bytes(':({},{})'.format(self.cmd_vel_lin, self.cmd_vel_ang), 'ascii')
        elif tmp_cmd == self.turn_left_msg:
            self.cmd_vel_ang    = self.check_angular_limit_velocity(self.cmd_vel_ang + self.vel_ang_step)
            ret_msg             = self.turn_right_resp + bytes(':({},{})'.format(self.cmd_vel_lin, self.cmd_vel_ang), 'ascii')
        elif tmp_cmd == self.turn_right_msg:
            self.cmd_vel_ang    = self.check_angular_limit_velocity(self.cmd_vel_ang - self.vel_ang_step)
            ret_msg             = self.turn_left_resp + bytes(':({},{})'.format(self.cmd_vel_lin, self.cmd_vel_ang), 'ascii')
        elif tmp_cmd == self.halt_msg:
            self.cmd_vel_lin    = 0.0
            self.cmd_vel_ang    = 0.0
            ret_msg             = self.halt_resp + bytes(':({},{})'.format(self.cmd_vel_lin, self.cmd_vel_ang), 'ascii')
        self.publish_cmd_vel_msg()
        rospy.loginfo('[MOVE_BASE_CTR] {}, {}'.format(self.cmd_vel_lin, self.cmd_vel_ang))
        if ret_msg is not None:
            ReturnMsg.put(ret_msg)
        
    def control_task(self, Command: Queue, ReturnMsg: Queue, Operation: Queue):
        rospy.loginfo('[MOVE BASE] thread started.')
        try:
            while not rospy.is_shutdown():
                if Operation.qsize() != 0:
                    break
                l = Command.qsize()
                for _ in range(l):
                    tmp_cmd = Command.get()
                    self.move(tmp_cmd, ReturnMsg)
                    self.rate.sleep()
                self.publish_cmd_vel_msg()
                self.rate.sleep()
            
        except Exception as e:
            self.raise_error('MOVE BASE', e)
            # rospy.logerr('[MOVE BASE] Error: {}'.format(e))
            Operation.put(False)
        
        finally:
            self.cmd_vel_lin = 0.0
            self.cmd_vel_ang = 0.0
            self.publish_cmd_vel_msg()
            rospy.loginfo('[Move base] Thread terminated.')
    
    def UDP_tx_task(self, 
                    ReturnMsg: Queue, 
                    Operation: Queue):
        rospy.loginfo('[MOVE_BASE_UTX] Thread started.')
        UDP_IP  = self.UDP_IP
        TX_PORT = self.UDP_PORT_TX
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                
                while not rospy.is_shutdown():
                    if Operation.qsize() != 0:
                        break
                    
                    l = ReturnMsg.qsize()
                    for _ in range(l):
                        tmp_msg = ReturnMsg.get()
                        sock.sendto(tmp_msg, (UDP_IP, TX_PORT))
                        rospy.loginfo('[MOVE_BASE_UTX] {}'.format(tmp_msg.decode('ascii')))
                        print(tmp_msg)
                    time.sleep(0.5)
                    
        except Exception as e:
            self.raise_error('MOVE_BASE_UTX', e)
            # rospy.logerr('[MOVE_BASE_UTX] Error: {}'.format(e))
            Operation.put(False)
            
        finally:
            rospy.loginfo('[MOVE_BASE_UTX] Thread terminated.')
            
    def UDP_rx_task(self, 
                    Command: Queue, 
                    Operation: Queue):
        rospy.loginfo('[MOVE_BASE_URX] thread started')
        
        EGO_IP  = self.EGO_IP
        UDP_IP  = self.UDP_IP
        RX_PORT = self.UDP_PORT_RX
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                # sock.bind((UDP_IP, RX_PORT))
                sock.bind(("", RX_PORT))
                sock.settimeout(0.5)
                
                while not rospy.is_shutdown():
                    try:
                        if Operation.qsize() != 0:
                            break
                        data, addr = sock.recvfrom(1024)
                        
                        if len(data) != 0:
                            print(data)
                            print(data.decode('ascii'))
                            rospy.loginfo('{}, {}'.format(addr, data.decode('ascii')))
                            if addr[0] != EGO_IP:
                                Command.put(data)
                                
                        time.sleep(0.05)
                    except socket.timeout:
                        continue
        except Exception as e:
            self.raise_error('MOVE_BASE_URX', e)
            # rospy.logerr('[MOVE_BASE_URX] Error: {}'.format(e))
            Operation.put(False)
            
        finally:
            rospy.loginfo('[MOVE_BSAE_URX] Thread terminated.')
                    
    def run(self):
        try:
            self.set_configuration()
            self.control_thread = threading.Thread(target=self.control_task,    args=self.control_thread_args)
            self.udp_tx_thread  = threading.Thread(target=self.UDP_tx_task,     args=self.udp_tx_thread_args)
            self.udp_rx_thread  = threading.Thread(target=self.UDP_rx_task,     args=self.udp_rx_thread_args)
            
            self.control_thread.start()
            self.udp_tx_thread.start()
            self.udp_rx_thread.start()
                    
        except Exception as e:
            self.Operation.put(False)
            if self.control_thread.is_alive():
                self.control_thread.join()
            if self.udp_tx_thread.is_alive():
                self.udp_tx_thread.join()
            if self.udp_rx_thread.is_alive():
                self.udp_rx_thread.join()
                
        finally:
            if self.control_thread.is_alive():
                self.control_thread.join()
            if self.udp_tx_thread.is_alive():
                self.udp_tx_thread.join()
            if self.udp_rx_thread.is_alive():
                self.udp_rx_thread.join()
        
        
if __name__ == '__main__':
    rospy.init_node('move_base', anonymous=False)
    
    node = MoveBase()
    
    node.run()