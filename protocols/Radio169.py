
import struct
from .Bot_State import Bot_State


class Radio169:
    def __init__(self):

        # special buttons:
        self.btn_start = False
        self.btn_back  = False
        self.btn_logi  = False
        self.btn_wtf   = False

        # The four buttons on the front-side:
        self.btn_LB    = False
        self.btn_LT    = False
        self.btn_RB    = False
        self.btn_RT    = False

        # The four individual buttons on the right:
        self.btn_Y     = False
        self.btn_A     = False
        self.btn_B     = False
        self.btn_X     = False

        # The 4-way button switchpad on the left:
        self.btn_up    = False
        self.btn_down  = False
        self.btn_right = False
        self.btn_left  = False

        # The 2 analog joysticks:
        self.leftjoy_lr = 0.0
        self.leftjoy_ud = 0.0
        self.rightjoy_lr = 0.0
        self.rightjoy_ud = 0.0

        self._prev_A = False
        self._prev_B = False
        self._prev_Y = False

    def parse_payload(self, payload):
        assert len(payload) == 8

        # Typically payload[0] is 0x80
        if payload[0] != 0x80:
            print(" ************* Radio169: BAD PAYLOAD")
            return

        # special buttons:
        self.btn_start = bool(payload[1] & 0x10)
        self.btn_back  = bool(payload[1] & 0x20)
        self.btn_logi  = bool(payload[1] & 0x40)
        self.btn_wtf   = bool(payload[1] & 0x80)  # unused, I think

        # The four buttons on the front-side:
        self.btn_LB    = bool(payload[1] & 0x01)
        self.btn_LT    = bool(payload[1] & 0x02)
        self.btn_RB    = bool(payload[1] & 0x04)
        self.btn_RT    = bool(payload[1] & 0x08)

        # The four individual buttons on the right:
        self.btn_Y     = bool(payload[2] & 0x10)  # orange/yellow
        self.btn_A     = bool(payload[2] & 0x20)  # green
        self.btn_B     = bool(payload[2] & 0x40)  # red
        self.btn_X     = bool(payload[2] & 0x80)  # blue

        # The 4-way button switchpad on the left:
        self.btn_up    = bool(payload[2] & 0x01)
        self.btn_down  = bool(payload[2] & 0x02)
        self.btn_right = bool(payload[2] & 0x04)
        self.btn_left  = bool(payload[2] & 0x08)

        # The 2 analog joysticks:
        L_lr, L_ud, R_lr, R_ud = struct.unpack('bbbb', payload[3:7])

        self.leftjoy_lr = (L_lr - 64) / 64.0
        self.leftjoy_ud = (L_ud - 64) / 64.0
        self.rightjoy_lr = (R_lr - 64) / 64.0
        self.rightjoy_ud = (R_ud - 64) / 64.0

        req_mode = None
        if self.btn_back:  # Program kill
            print('R169: btn_back   kill auto-pilot')
            req_mode = Bot_State.NONE

        if self.btn_A and not self._prev_A:
            print('R169:  btn_A   go out')
            req_mode = Bot_State.GO_OUT

        if self.btn_Y and not self._prev_Y:
            print('R169:  btn_Y   go home')
            req_mode = Bot_State.GO_HOME

        if self.btn_X and not self._prev_X:
            print('R169:  btn_X   follow mode')
            req_mode = Bot_State.FOLLOW

        if self.btn_B and not self._prev_B:
            print('R169:  btn_B   touch button')
            req_mode = "BUTTON"

        self._prev_A = self.btn_A
        self._prev_B = self.btn_B
        self._prev_X = self.btn_X
        self._prev_Y = self.btn_Y

        return req_mode


