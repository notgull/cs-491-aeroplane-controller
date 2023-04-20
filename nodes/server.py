#!/usr/bin/env python
# NO IMPLIED WARRANTY
# Written by John Nunley, <your names here>

import rospy

from dynamic_reconfigure.server import Server
from cs_491_controller.cfg import TutorialsConfig

from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import String

# TODO: I don't know how ROS works well enough to say if this works or not
TOPIC_NAME = "/minihawk_SIM/mavros/rc/override"
TAG_DETECTION = "tag_detection"

def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
    #       {str_param}, {bool_param}, {size}""".format(**config))
    return config

def publisher():
    # TODO: Use the actual mavros message type
    pub = rospy.Publisher(TOPIC_NAME, OverrideRCIn, queue_size=10)
    rospy.init_node("cs_491_controller", anonymous=True)
    rate = rospy.Rate(10)

    channels = [
        1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    ]

    def on_receive_config(config, level):
        channels[0] = config["roll"]
        channels[1] = config["pitch"]
        channels[2] = config["throttle"]
        channels[3] = config["yaw"]
        channels[4] = config["channel1"]
        channels[5] = config["channel2"]
        channels[6] = config["channel3"]
        channels[7] = config["channel4"]
        channels[8] = config["channel5"]
        channels[9] = config["channel6"]
        channels[10] = config["channel7"]
        channels[11] = config["channel8"]
        channels[12] = config["channel9"]
        channels[13] = config["channel10"]
        channels[14] = config["channel11"]
        channels[15] = config["channel12"]
        channels[16] = config["channel13"]
        channels[17] = config["channel14"]

        rospy.loginfo("Updated channels to {}".format(channels))

        return config

    srv = Server(TutorialsConfig, on_receive_config)

    # Like and subscribe to the tag detection topic
    def process_tag_detection(data):
        rospy.loginfo(data.data)
    rospy.Subscriber(TAG_DETECTION, String, process_tag_detection)

    while not rospy.is_shutdown():
        the_data = OverrideRCIn()
        the_data.channels = channels

        pub.publish(the_data)
        rate.sleep()






if __name__ == "__main__":
    publisher()