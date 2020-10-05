
import struct


'''
this is the packet structure in C:

    struct multi_data {

        // 64 bits:
        uint16_t version;
        int16_t compass_XYZ[3];  // external compass

        // 3 * 64 bits:
        char bnoData[22];  // internal IMU
        int16_t _padding2;  // the compiler seems to like 64-bit boundaries

        // 64 bits:
        float temperature; // degrees celsius, no need for high accuracy

        // Pressure:  typical sensor value is ~100000, with accuracy of +/- 12.0,
        // (don't forget to convert between Pa and hPa), so this is well
        // within the accuracy of float32
        float pressure;

        // 64 bits:
        uint16_t sbus_a;
        uint16_t sbus_b;
        uint8_t moab_mode;
        uint8_t _padding3;  // 64-bit boundary
        uint16_t adc0;

        // Everything ABOVE here is the official, "version 1" of this protocol
        // Everything BELOW here is extra, and might change in the future

        // 64 bits:
        // TODO:  do we really need float64 for these numbers?
        double shaft_pps;

    } mData;


'''

class ImuPacket:
    def __init__(self):
        self.shaft_pps = 0
        self.shaft_a_pps = 0
        self.shaft_b_pps = 0
        self.moab_mode = -1
        self.sbus_a = 1024
        self.sbus_b = 1024
        self.moab_mode = -1
        self.adc0 = 0

    def parse(self, pkt):
        version, = struct.unpack('<h', pkt[:2])

        if version == 1:
            self.version, self.magX, self.magY, self.magZ, \
                self.qw, self.qx, self.qy, self.qz, \
                self.lax, self.lay, self.laz, self.gx, self.gy, self.gz, \
                self.imu_temp, self.calib_stat, \
                self._padding2, \
                self.temperature, self.pressure, \
                self.sbus_a, self.sbus_b, self.moab_mode, self._padding3, self.adc0 \
                    = struct.unpack('<HhhhhhhhhhhhhhbBhffHHBBh', pkt[:48])

            self._extra = pkt[48:]
            if len(self._extra) == 8:
                self.shaft_pps, = struct.unpack('<d', self._extra[:8])
            elif len(self._extra) == 16:
                self.shaft_a_pps, = struct.unpack('<d', self._extra[:8])
                self.shaft_b_pps, = struct.unpack('<d', self._extra[8:16])
                self.shaft_pps = 0.5 * (self.shaft_a_pps + self.shaft_b_pps)
            else:
                print('unknown "extra" field')

        else:
            print("unknown IMU packet version:", version)





