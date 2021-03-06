#!/usr/bin/env python
# -*- coding: utf-8 -*
import time
import sys
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from vehicle_additional import get_bearing, get_distance_metres, get_location_metres

from pymavlink import mavutil
from pid import PID
from wingman import Wingman

import argparse
parser = argparse.ArgumentParser(description='双机编队')
parser.add_argument('-v1',
                    help="Vehicle1 connection target string.",nargs='?', const="127.0.0.1:12341", type=str)
parser.add_argument('-v2',
                    help="Vehicle2 connection target string.",nargs='?', const="127.0.0.1:12342", type=str)
args = parser.parse_args()

print(args)
C = 60
drift = -10


# Connect to the Vehicle
print("正在链接")
vehicle1 = connect(args.v1)
vehicle2 = connect(args.v2,vehicle_class=Wingman)
print("链接成功")

pid = PID(0.2,0.01,0.1)
pid.target = 0

import time
while True:
    pos = vehicle1.location.global_relative_frame

    R = vehicle1.groundspeed ** 2 / (9.8* math.tan(vehicle1.attitude.roll))
    beta = C / R

    bearing = vehicle1.heading + beta /2 *180/math.pi

    pos = get_location_metres(pos, bearing,
                              2*R*math.sin(beta/2)+drift, is_bearing=True)

    
    vehicle2.follow(pos)

    dis = get_distance_metres(
        vehicle2.location.global_relative_frame, pos) - C
    dt_speed = pid(-dis)
    if dt_speed < 0:
        dt_speed *= 1.5
    vehicle2.set_thr_aspd = vehicle1.airspeed+dt_speed
    
    #设置目标速度
    print("速度差距", vehicle2.airspeed-vehicle1.airspeed)

    time.sleep(0.2)
