#!/urs/bin/env python3

import rospy
import os, sys, select, time

from std_msgs.msg import UInt8

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

KEYLIST = ['w', 's', 'a', 'd', 'p', ' ', '=', 'y', 'x']


class Node:
    def __init__(self):
        self.rate = rospy.Rate(100)
        
        self.settings = None
        
        self.cmd = ''
        self.pub = rospy.Publisher('/cmd', UInt8, queue_size=10)
        
    def getKey(self):
        if os.name == 'nt':
            timeout=0.1
            start_time = time.time()
            
            while(1):
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                    
                elif time.time() - start_time >= timeout:
                    return ''
                
        else:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
    
    def run(self):
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
            
        while not rospy.is_shutdown():
            key = self.getKey()
            msg = None
            
            if key in KEYLIST:
                print('[DEBUG]: {}'.format(key))
                if key == 'w':
                    msg = UInt8()
                    msg.data = 1
                    self.pub.publish(msg)
                elif key == 'x':
                    msg = UInt8()
                    msg.data = 2
                    self.pub.publish(msg)
                elif key == 'a':
                    msg = UInt8()
                    msg.data = 3
                    self.pub.publish(msg)
                elif key == 'd':
                    msg = UInt8()
                    msg.data = 4
                    self.pub.publish(msg)
                elif key in [' ', 's', 'p']:
                    msg = UInt8()
                    msg.data = 9
                    self.pub.publish(msg)
                elif key == 'y':
                    msg = UInt8()
                    msg.data = 11
                    self.pub.publish(msg)
                elif key == '=':
                    msg = UInt8()
                    msg.data = 99
                    self.pub.publish(msg)
                    break
                else:
                    pass
                
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('teleop', anonymous=True)
    
    node = Node()
    
    node.run()