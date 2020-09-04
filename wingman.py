from dronekit import Vehicle, Command, VehicleMode
from pymavlink import mavutil


class Wingman(Vehicle):
    def __init__(self, *args):
        super(Wingman, self).__init__(*args)
        self.max_speed = 35
        self.min_speed = 12
        self.aspd2thr = [
            {"thr": 10, "aspd": 10},
            {"thr": 20, "aspd": 15},
            {"thr": 33, "aspd": 22},
            {"thr": 50, "aspd": 27},
            {"thr": 80, "aspd": 33},
            {"thr": 100, "aspd": 36},
        ]

    @property
    def thr_aspd(self):
        return self.airspeed

    @thr_aspd.setter
    def set_thr_aspd(self, aspd):
        aspd = max(aspd, self.min_speed)
        aspd = min(aspd, self.max_speed)

        thr = -1
        for item in self.aspd2thr:
            if item["aspd"] >= aspd:
                thr = item["thr"]
                break
        if thr == -1:
            thr = 100

        if self.mode.name == "AUTO":
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
            return True
        if self.mode.name == 'GUIDED':
            speed_type = 0  # air speed
            msg = self.message_factory.command_int_encode(
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
                mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,  # command
                1,#current
                False,#autocontinue
                speed_type,  # param 1
                aspd,  # speed in metres/second
                0, 0, 0, 0, 0  # param 3 - 7
            )
            print('sd',aspd)
            self.send_mavlink(msg)
            return  True   
        return False

    def follow(self,pos):

        self.commands.clear()
        self.commands.wait_ready()
        self.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, pos.lat, pos.lon, pos.alt+10))
        self.commands.upload()
        self.commands.wait_ready()
        self.mode = VehicleMode("AUTO")
        self.commands.next = 1
