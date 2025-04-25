#!/usr/bin/env python3

import rospy
import os, sys, time, select

import numpy as np

from std_msgs.msg import UInt8
# import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def deg2rad(deg):
    return deg * np.pi / 180.0

SCALE = 0.7
VMAX = 0.1
WMAX = 20




class Node:
    def __init__(self, odom_enable=False):
        self.ODOM_ENABLE = odom_enable
        self.rate = rospy.Rate(100)
        self.waypoint = {
            'x':    [       SCALE,          SCALE,         SCALE,          SCALE,            0.0,            0.0,            0.0,          0.0],
            'y':    [         0.0,            0.0,         SCALE,          SCALE,          SCALE,          SCALE,            0.0,          0.0],
            'th':   [deg2rad(0.0),  deg2rad(90.0), deg2rad(90.0), deg2rad(180.0), deg2rad(180.0), deg2rad(270.0), deg2rad(270.0), deg2rad(0.0)],
        }
                    #     fw                 rt              fw            rt            fw              rt              fw              rt
        
        self.tar_chkpt = 0
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.target_x = None
        self.target_y = None
        self.target_th = None
        
        self.v_max = VMAX
        self.w_max = deg2rad(WMAX)
        
        self.a = self.v_max / 20.0
        self.o = self.w_max / 20.0
        
        self.auto_drive_flag = False
        self.goal_arrived_flag = False
        self.drive_flag = [0, 0, 0, 0, 1] # a, d, l, r, s
        self.quit_flag = False
        
        self.cur_v = 0.0
        self.cur_w = 0.0
        
        self.prev_t = time.time()
        
        self.operation = True
        
        self.cmd_sub = rospy.Subscriber('/cmd', UInt8, self.cmd_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
    def odom_callback(self, msg: Odometry):
        if self.ODOM_ENABLE:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.th = msg.pose.pose.orientation.z
        
    def cmd_callback(self, msg: UInt8):
        cmd = msg.data
        print('debug: {}'.format(msg.data))
        
        if cmd == 1:
            self.accelerate()
        elif cmd == 2:
            self.decelerate()
        elif cmd == 3:
            self.rotate_left()
        elif cmd == 4:
            self.rotate_right()
        elif cmd == 9:
            self.stop()
        elif cmd == 11:
            self.auto_drive()
        elif cmd == 99:
            self.quit()
    
    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.cur_v
        msg.angular.z = self.cur_w
        
        self.cmd_vel_pub.publish(msg)
    
    def calc_odom(self):
        cur_t = time.time()
        dt = cur_t - self.prev_t
        self.prev_t = cur_t
        self.th += dt * self.cur_w
        self.x += (dt * self.cur_v) * np.cos(self.th)
        self.y += (dt * self.cur_v) * np.sin(self.th)
    
    def v_profile(self, dir=1):
        if self.cur_w != 0.0:
            self.cur_w = 0.0
            
        if dir == 1:
            if self.v_max <= self.cur_v + self.a:
                self.cur_v = self.v_max
            else:
                self.cur_v += self.a
        elif dir == -1:
            if -self.v_max >= self.cur_v - self.a:
                self.cur_v = -self.v_max
            else:
                self.cur_v -= self.a
                
    def w_profile(self, dir=1):
        if self.cur_v != 0.0:
            self.cur_v = 0.0
        if dir == 1:
            if self.w_max <= self.cur_w + self.o:
                self.cur_w = self.w_max
            else:
                self.cur_w += self.o
        elif dir == -1:
            if -self.w_max >= self.cur_w - self.o:
                self.cur_w = -self.w_max
            else:
                self.cur_w -= self.o
                
    def stop_profile(self):
        if self.cur_v > 0.0:
            if self.cur_v - self.a <= 0.0:
                self.cur_v = 0.0
            else:
                self.cur_v -= self.a
        elif self.cur_v < 0.0:
            if self.cur_v + self.a >= 0.0:
                self.cur_v = 0.0
            else:
                self.cur_v += self.a
        
        if self.cur_w > 0.0:
            if self.cur_w - self.o <= 0.0:
                self.cur_w = 0.0
            else:
                self.cur_w -= self.o
        elif self.cur_w < 0.0:
            if self.cur_w + self.o >= 0.0:
                self.cur_w = 0.0
            else:
                self.cur_w += self.o
            
    def accelerate(self):
        self.drive_flag = [1, 0, 0, 0, 0]
    
    def decelerate(self):
        self.drive_flag = [0, 1, 0, 0, 0]
    
    def rotate_left(self):
        self.drive_flag = [0, 0, 1, 0, 0]
    
    def rotate_right(self):
        self.drive_flag = [0, 0, 0, 1, 0]
    
    def stop(self):
        if self.auto_drive_flag:
            self.auto_drive_flag = 0
            self.tar_chkpt = 0
        self.drive_flag = [0, 0, 0, 0, 1]
    
    def auto_drive(self):
        if self.auto_drive_flag:
            self.auto_drive_flag = False
            self.tar_chkpt = 0
        else:
            self.auto_drive_flag = True
            self.drive_flag = [0, 0, 0, 0, 1]
    
    def goal_chk(self):
        '''
        if self.tar_chkpt is even -> straight
        else if self.tar_chkpt id odd -> rotate ccw
        '''
        
        if self.tar_chkpt % 2 == 0:
            print('{:.5f}, {:.5f}, {}, {:.5f}, {:.5f}, {}'.format(self.x, 
                                                                          self.target_x, 
                                                                          abs(self.x - self.target_x) <= 0.02, 
                                                                          self.y, 
                                                                          self.target_y, 
                                                                          abs(self.y - self.target_y) <= 0.02))
            
            if abs(self.x - self.target_x) <= 0.01 and abs(self.y - self.target_y) <= 0.01:
                self.goal_arrived_flag = True
                
        elif self.tar_chkpt % 2 == 1:
            print('{:.5f}, {:.5f}, {:.5f}, '.format(self.target_th, self.th, abs(self.target_th - self.th), abs(self.target_th - self.th) < self.o))
            if abs(self.target_th - self.th) % (np.pi * 2) < self.o * 0.5:
                self.th = self.target_th
                self.goal_arrived_flag = True
                
    def quit(self):
        self.stop()
        self.quit_flag = True
        
    def run(self):
        while not rospy.is_shutdown() and self.operation:
            if not self.ODOM_ENABLE:
                self.calc_odom()
                self.th = self.th % (2 * np.pi)
                # if self.th >= np.pi * 2 + self.o * 0.5:
                #     self.th = self.th % (2 * np.pi)
                # elif self.th <= -np.pi * 2 - self.o * 0.5:
                #     self.th = self.th % (2 * np.pi)
            
            if self.quit_flag:
                if self.cur_v == 0.0 and self.cur_w == 0.0:
                    break
                else:
                    self.stop_profile()
            else:
                if self.auto_drive_flag:
                    if self.target_x is None:
                        self.tar_chkpt = 0
                        self.target_x = self.waypoint['x'][self.tar_chkpt]
                        self.target_y = self.waypoint['y'][self.tar_chkpt]
                        self.target_th = self.waypoint['th'][self.tar_chkpt]
                        
                        self.v_profile(1)
                        
                    else:
                        self.goal_chk()
                        if self.goal_arrived_flag:
                            if self.cur_v == 0.0 and self.cur_w == 0.0:
                                self.tar_chkpt = (self.tar_chkpt + 1) % 8
                                self.target_x = self.waypoint['x'][self.tar_chkpt]
                                self.target_y = self.waypoint['y'][self.tar_chkpt]
                                self.target_th = self.waypoint['th'][self.tar_chkpt]
                                self.goal_arrived_flag = False
                                if self.tar_chkpt % 2 == 0:
                                    self.v_profile(1)
                                else:
                                    self.w_profile(1)
                            else:
                                self.stop_profile()
                        else:
                            if self.tar_chkpt % 2 == 0:
                                self.v_profile(1)
                            else:
                                self.w_profile(1)
                else:
                    if self.drive_flag[0] == 1:
                        self.v_profile(1)
                    elif self.drive_flag[1] == 1:
                        self.v_profile(-1)
                    elif self.drive_flag[2] == 1:
                        self.w_profile(1)
                    elif self.drive_flag[3] == 1:
                        self.w_profile(-1)
                    elif self.drive_flag[4] == 1:
                        self.stop_profile()
            self.publish_cmd_vel()
            self.rate.sleep()
            

if __name__ == '__main__':
    rospy.init_node('couse_tracking', anonymous=True)
    
    node = Node()
    
    node.run()
    