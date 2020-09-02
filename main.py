#!/usr/bin/env python
# -*- coding: utf-8 -*
import time
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil


from vehicle_additional import get_bearing, get_distance_metres, get_location_metres

import argparse
parser = argparse.ArgumentParser(description='双机编队')
parser.add_argument('-v1',
                    help="Vehicle1 connection target string.")
parser.add_argument('-v2',
                    help="Vehicle2 connection target string.")
args = parser.parse_args()


def constrain_value(a, b, c):
    if a > c:
        a = c
    if a < b:
        a = b
    return a


def cur_usec():
    """Return current time in usecs"""
    t = time.time()
    # dt = datetime.now()
    # t = dt.minute * 60 + dt.second + dt.microsecond / (1e6)
    return t


class MeasureTime(object):
    def __init__(self):
        self.prevtime = cur_usec()
        self.previnterval = 0
        self.numcount = 0
        self.reset()

    def reset(self):
        self.maxinterval = 0
        self.mininterval = 10000

    def log(self):
        # print "Interval", self.previnterval
        # print "MaxInterval", self.maxinterval
        # print "MinInterval", self.mininterval
        sys.stdout.write('MaxInterval: %s\tMinInterval: %s\tInterval: %s\r' % (
            self.maxinterval, self.mininterval, self.previnterval))
        sys.stdout.flush()

    def update(self):
        now = cur_usec()
        self.numcount = self.numcount + 1
        self.previnterval = now - self.prevtime
        self.prevtime = now
        if self.numcount > 1:  # ignore first value where self.prevtime not reliable.
            self.maxinterval = max(self.previnterval, self.maxinterval)
            self.mininterval = min(self.mininterval, self.previnterval)
            # self.log()


acktime = MeasureTime()


# 初始值
K_P = 0.3
K_I = 0.001
AMAX_I = 2
K_D = 0
data_I = 0

Rsd_dis = 100
drift = 0


# Connect to the Vehicle
print("正在链接")
vehicle1 = connect(args.v1, wait_ready=True)
vehicle2 = connect(args.v2, wait_ready=True)


def set_airspeed(self, aspd):
    thr = -1
    if aspd < 10:
        thr = 15
    elif aspd < 15:
        thr = 30
    elif aspd < 18:
        thr = 40
    elif aspd < 20:
        thr = 50
    elif aspd < 22:
        thr = 65
    elif aspd < 25:
        thr = 80
    else:
        thr = 95



    speed_type = 0  # air speed
    msg = self.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # command
        0,  # confirmation
        speed_type,  # param 1
        aspd,  # speed in metres/second
        thr, 0, 0, 0, 0  # param 3 - 7
    )
    self.send_mavlink(msg)




while True:
    pos = vehicle1.location.global_relative_frame
    pos = get_location_metres(pos, vehicle1.heading,
                              Rsd_dis+drift, is_bearing=True)

    vehicle2.commands.clear()
    vehicle2.commands.wait_ready()
    vehicle2.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, pos.lat, pos.lon, pos.alt+5))
    vehicle2.commands.upload()
    vehicle2.commands.wait_ready()
    vehicle2.mode = VehicleMode("AUTO")
    vehicle2.commands.next = 1

    acktime.update()

    dis = get_distance_metres(
        vehicle2.location.global_relative_frame, pos) - Rsd_dis

    data_P = K_P * dis
    data_I = data_I + K_I * dis * acktime.previnterval

    dt_speed = data_P + constrain_value(data_I, -AMAX_I, AMAX_I)
    
    
    #设置目标速度
    set_airspeed(vehicle2,constrain_value(vehicle1.airspeed+dt_speed, 10, 30))
    print(dis, vehicle2.airspeed-vehicle1.airspeed, data_P, data_I)

    time.sleep(0.2)
