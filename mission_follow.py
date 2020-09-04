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
                    help="Vehicle1 connection target string.", nargs='?', const="127.0.0.1:12341", type=str)
parser.add_argument('-v2',
                    help="Vehicle2 connection target string.", nargs='?', const="127.0.0.1:12342", type=str)

parser.add_argument('-vu1',
                     help="Vehicle1 connection target string.")
parser.add_argument('-vu2',
                    help="Vehicle2 connection target string.")

args = parser.parse_args()

if args.vu1 and args.vu2:
    v1 = connect(args.vu1)
    v2 = connect(args.vu2, vehicle_class=Wingman)
else:
    v1 = connect(args.v1)
    v2 = connect(args.v2, vehicle_class=Wingman)


C = 100
drift = 0

pid = PID(0.2, 0.01, 0.1)
pid.target = 0


while True:
    pos = get_location_metres(
        v1.location.global_relative_frame, v1.heading, C + drift, is_bearing=True)

    dt_speed = pid(-1 * (get_distance_metres(
        pos, v2.location.global_relative_frame) - C))
    
    if dt_speed < 0:
        dt_speed *= 1.5
    v2.set_thr_aspd = v1.airspeed+dt_speed

    print("速度差距", v2.airspeed-v1.airspeed, dt_speed)
    time.sleep(0.2)