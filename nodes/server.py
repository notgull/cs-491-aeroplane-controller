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
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    return config

def publisher():
    # TODO: Use the actual mavros message type
    pub = rospy.Publisher(TOPIC_NAME, OverrideRCIn, queue_size=10)
    rospy.init_node("cs_491_controller", anonymous=True)
    rate = rospy.Rate(10)

    srv = Server(TutorialsConfig, callback)

    # Like and subscribe to the tag detection topic
    def process_tag_detection(data):
        rospy.loginfo(data.data)
    rospy.Subscriber(TAG_DETECTION, String, process_tag_detection)

    while not rospy.is_shutdown():
        # TODO: Adjust on user input, somehow
        the_data = OverrideRCIn()
        the_data.channels = [
            1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        ]

        pub.publish(the_data)
        rate.sleep()

if __name__ == "__main__":
    publisher()