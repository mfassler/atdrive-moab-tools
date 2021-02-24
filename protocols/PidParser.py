import struct

class PidParser:
    def __init__(self):
        self.sbus_steering = None
        self.sbus_throttle = None
        self.output = None
        self.K_p = None
        self.e = None
        self.K_i = None
        self.I = None
        self.target_speed = None
        self.actual_speed = None

    def parse_packet(self, pkt):
        self.sbus_steering, self.sbus_throttle, self.output, \
        self.K_p, self.e, self.K_i, self.I, \
        self.target_speed, self.actual_speed = struct.unpack('HHfdddddd', pkt)


