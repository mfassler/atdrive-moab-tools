#!/usr/bin/env pvpython
'''
Visualize some vectors in ParaView.  Requires ParaView with pvpython.
'''

from __future__ import print_function

from paraview.simple import *

import select
import socket
import struct
import numpy as np

cone1 = Cone(Radius=0.1, Center=[0,0,0])
Show()
cone2 = Cone(Radius=0.1, Center=[0,0,0])
Show()
cone3 = Cone(Radius=0.1, Center=[0,0,0])
Show()
Render()
ResetCamera()
sources = GetSources()
renderView1 = GetActiveViewOrCreate('RenderView')


cone1Display = GetDisplayProperties(cone1, view=renderView1)
cone2Display = GetDisplayProperties(cone2, view=renderView1)
cone3Display = GetDisplayProperties(cone3, view=renderView1)
cone1Display.DiffuseColor = [1.0, 0, 0]
cone2Display.DiffuseColor = [1.0, 1.0, 0]
cone3Display.DiffuseColor = [0, 0, 1.0]


print("cone1:", cone1)
print("sources:", sources)
print("renderView1:", renderView1)

DATA_PORT = 12311
data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data_sock.bind(("127.0.0.1", DATA_PORT))


def updateVectors(rotMatrix):
    cone1.Direction = np.dot(rotMatrix, np.array([1,0,0]))
    cone2.Direction = np.dot(rotMatrix, np.array([0,1,0]))
    cone3.Direction = np.dot(rotMatrix, np.array([0,0,1]))
    renderView1.Update()
    RenderAllViews()


def rxLastPacket(sock):
    '''Throw away all packets except the most recent one (in case the GUI is too slow)'''
    data = None
    sock.setblocking(0)
    cont = True
    while cont:
        try:
            tmpData, addr = sock.recvfrom(256)
        except Exception as ee:
            #print(ee)
            cont = False
        else:
            if tmpData:
                if data is not None:
                    pass
                    #print('throwing away a packet (GUI is too slow)')
                data = tmpData
            else:
                cont=False
    sock.setblocking(1)
    return data


while True:
    inputs, outputs, errors = select.select([data_sock], [], [])
    for oneInput in inputs:
        if oneInput == data_sock:
            data = rxLastPacket(data_sock)
            if data is not None:
                if len(data) == 72:
                    rotMatrix = np.frombuffer(data).reshape((3,3))
                    updateVectors(rotMatrix)
                else:
                    print('failed to parse UDP packet')


