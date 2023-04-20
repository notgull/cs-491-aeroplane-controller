#!/usr/bin/env
# NO IMPLIED WARRANTY
# Written by John Nunley, <your names here>

import rospy
from mavros.msg import OverrideRCIn

# TODO: I don't know how ROS works well enough to say if this works or not
TOPIC_NAME = "/minihawk_SIM/mavros/rc/override"
TAG_DETECTION = "tag_detection"

def publisher():
    # TODO: Use the actual mavros message type
    pub = rospy.Publisher(TOPIC_NAME, OverrideRCIn, queue_size=10)
    rospy.Subscriber(TAG_DETECTION, String, process_tag_detection)
    rospy.init_node("cs_491_controller", anonymous=True)

    rate = rospy.Rate(10)

    # Like and subscribe to the tag detection topic
    # def process_tag_detection(data):
    #     rospy.loginfo(data.data)

    while not rospy.is_shutdown():
        # TODO: Adjust on user input, somehow
        the_data = channels: [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        pub.publish(the_data)
        rate.sleep()

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
