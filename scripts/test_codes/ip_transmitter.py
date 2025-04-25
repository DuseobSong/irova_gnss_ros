import os, sys
import datetime
import smtplib
import time
import re

class Transmitter:
    def __init__(self):
        self.from_addr = "rtktest7700@gmail.com"
        self.password = "gibu rjoe vosw imth"
        self.to_addr = "duseob.song@irova.kr"
        self.smtp_addr = "smtp.gmail.com"
    
    def validate_ip_addr(self, addr):
        reg="^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]{1,3}$"
        ret = re.match(reg, addr)
        return ret
        
    def run(self):
        try:
            while True:
                stream = os.popen("iw wlan0 link | grep -w Connected")
                
                if len(stream.read()) != 0:
                    stream = os.popen('iw wlan0 link | grep -w SSID')
                    ssid = stream.read()[1:-1]
                    stream = os.popen("ifconfig | grep -w inet | tail -1 | awk '{print $2}'")
                    ip_addr = stream.read()[:-1]
                    if self.validate_ip_addr(ip_addr):
                        date = datetime.datetime.now().strftime('%Y.%m.%d %H:%M:%S')
                        connection = smtplib.SMTP(self.smtp_addr) 
                        connection.starttls() # Transport Layer Security
                        connection.login(user=self.from_addr, password=self.password)
                        connection.sendmail(
                            from_addr=self.from_addr,
                            to_addrs=self.to_addr,
                            msg="Subject:IROVA TURTLEBOT IP\n\n\
                                {}\n\
                                TURTLEBOT IP: {}\n\
                                DATE: {}\n".format(ssid, ip_addr, date)
                        )
                        break
                    else:
                        time.sleep(60.0)
                else:
                    time.sleep(60.0)
        except:
            pass
        finally:
            try:
                connection.close()
            except:
                pass
        

if __name__ == '__main__':
    node = Transmitter()
    
    node.run()