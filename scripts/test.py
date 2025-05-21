import socket

while True:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(("192.168.0.255", 25001))
        
        sock.settimeout(0.5)
        
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                
                if len(data) != 0:
                    print(addr, data)
                    
            except socket.timeout:
                pass
            
            except KeyboardInterrupt:
                break
            
        
                