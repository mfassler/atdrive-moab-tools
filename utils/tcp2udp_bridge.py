"""
TCP to UDP bridge, so that u-center can configure the GPS module over 
the network to/from the Moab
"""

import socket
import select


s_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
s_udp.bind(('0.0.0.0', 27113))
MOAB_ADDR = ('192.168.0.69', 27113)

s_tcp = socket.create_server(('127.0.0.1', 27113))

s_server = {s_udp, s_tcp}
connections = set()


while True:
    inputs, outputs, errors = select.select(s_server | connections, [], [])
    toRemove = set()
    for oneInput in inputs:
        if oneInput == s_udp:
            pkt, addr = s_udp.recvfrom(2048)
            for aConn in connections:
                try:
                    aConn.send(pkt)
                except:
                    aConn.close()
                    toRemove.add(aConn)

        elif oneInput == s_tcp:
            print('new TCP connection...')
            conn, addr = s_tcp.accept()
            connections.add(conn)


        else:
            if oneInput in connections:
                try:
                    pkt = oneInput.recv(2048)
                except:
                    aConn.close()
                    toRemove.add(aConn)
                else:
                    if len(pkt):
                        print(' from a tcp connection:', pkt)
                        s_udp.sendto(pkt, MOAB_ADDR)


    for item in toRemove:
        connections.remove(item)
        print(len(connections), 'connections')


