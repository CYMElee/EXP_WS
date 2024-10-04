#!/usr/bin/env python3

import rospy
from pymavlink.dialects.v20 import common as MAV_PX4  # 使用 PX4 MAVLink 方言
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink

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
    Send a MAVLink message
    """
    packed_data = mavlink_msg.pack(mav, True)

    rosmsg = convert_to_rosmsg(mavlink_msg)
    pub.publish(rosmsg)

    print("Sent message: %s" % mavlink_msg)

def set_stream_rate(mav, pub, message_id, rate_hz):
    """
    Set stream rate using MAV_CMD_SET_MESSAGE_INTERVAL
    """
    target_system = mav.srcSystem
    target_component = mav.srcComponent
    interval_us = int(1e6 / rate_hz)  # Convert Hz to microseconds

    msg = MAV_PX4.MAVLink_command_long_message(
        target_system,
        target_component,
        MAV_PX4.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # Confirmation
        message_id,  # The ID of the message to set the interval for
        interval_us,  # Desired interval in microseconds
        0, 0, 0, 0, 0  # Unused parameters
    )

    send_message(msg, mav, pub)

if __name__ == "__main__":
    rospy.init_node("stream_rate_setter")

    UAVID = rospy.get_param('UAV_ID', default=1)
    stream_rate = rospy.get_param('stream_rate', default=100)  # Default to 100Hz

    topic = "/platform"  + "/mavlink/to"
    mavlink_pub = rospy.Publisher(topic, Mavlink, queue_size=10)

    # Setup mavlink instance
    f = fifo()
    mav = MAV_PX4.MAVLink(f, srcSystem=UAVID, srcComponent=1)

    # Wait until the publisher is connected
    while mavlink_pub.get_num_connections() == 0:
        rospy.sleep(1)

    # Set stream rates for each message
    messages = {
        "ATTITUDE": 30,
        "ATTITUDE_QUATERNION": 31,
        "HEARTBEAT": 0,
        "RC_CHANNELS": 65,
        "RAW_IMU": 27,
        "ALTITUDE": 141,
        "LOCAL_POSITION_NED": 32
    }

    for message_name, message_id in messages.items():
        set_stream_rate(mav, mavlink_pub, message_id, stream_rate)
        rospy.sleep(0.5)  # Optional delay between commands

    rospy.spin()
