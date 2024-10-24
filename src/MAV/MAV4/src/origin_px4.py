#!/usr/bin/env python3

##
#
# Send SET_GPS_GLOBAL_ORIGIN and SET_HOME_POSITION messages for PX4
#
##

import rospy
from pymavlink.dialects.v20 import common as MAV_PX4  # 使用 PX4 MAVLink 方言
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink

# Global position of the origin
lat = 0   # Replace with actual latitude
lon = 0   # Replace with actual longitude
alt = 0   # Replace with actual altitude

class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

def send_message(mavlink_msg, mav, pub):
    """
    Send a mavlink message
    """
    packed_data = mavlink_msg.pack(mav, True)

    rosmsg = convert_to_rosmsg(mavlink_msg)
    pub.publish(rosmsg)

    print("sent message %s" % mavlink_msg)
    

def set_global_origin(mav, pub):
    """
    Send a mavlink SET_GPS_GLOBAL_ORIGIN message for PX4
    """
    target_system = mav.srcSystem
    lattitude = lat
    longitude = lon
    altitude = alt

    msg = MAV_PX4.MAVLink_set_gps_global_origin_message(
            target_system,
            lattitude, 
            longitude,
            altitude)

    send_message(msg, mav, pub)

def set_home_position(mav, pub):
    """
    Send a mavlink SET_HOME_POSITION message for PX4
    """
    target_system = mav.srcSystem

    lattitude = lat
    longitude = lon
    altitude = alt
    
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = MAV_PX4.MAVLink_set_home_position_message(
            target_system,
            lattitude,
            longitude,
            altitude,
            x,
            y,
            z,
            q,
            approach_x,
            approach_y,
            approach_z)

    send_message(msg, mav, pub)

if __name__=="__main__":
    UAVID = rospy.get_param('UAV_ID', default=0)
    rospy.sleep(10)
    try:
        rospy.init_node("origin_publisher")

        topic = "/MAV" + str(UAVID) + "/mavlink/to"
        print("send to topic: ", format(topic))
        mavlink_pub = rospy.Publisher(topic, Mavlink, queue_size=20)

        # Set up mavlink instance
        f = fifo()
        mav = MAV_PX4.MAVLink(f, srcSystem=UAVID, srcComponent=1)
        print(mav)

        # wait to initialize
        while mavlink_pub.get_num_connections() <= 0:
            pass
   
        for _ in range(2):
            rospy.sleep(1)
            set_global_origin(mav, mavlink_pub)
            set_home_position(mav, mavlink_pub)

    except rospy.ROSInterruptException:
        pass
