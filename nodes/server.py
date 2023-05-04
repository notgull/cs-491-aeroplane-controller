#!/usr/bin/env python
# NO IMPLIED WARRANTY
# Written by John Nunley, <your names here>

import rospy
import enum
import time

from pid import PID

from dynamic_reconfigure.server import Server
from cs_491_controller.cfg import TutorialsConfig

from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool
from apriltag_ros.msg import AprilTagDetectionArray

from scipy.spatial.transform import Rotation as R

TOPIC_NAME = "/minihawk_SIM/mavros/rc/override"
TAG_DETECTION = "/minihawk_SIM/MH_usb_camera_link_optical/tag_detections"

class RobotState(enum.Enum):
    SEEKING = 0
    CENTERING = 1
    DESCENDING = 2

def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
    #       {str_param}, {bool_param}, {size}""".format(**config))
    return config

state = RobotState.SEEKING
print('Enter Seeking Mode:')

detection_pose = None

# Approximate center in terms of the X coordinate
CENTER_X = 3
GLOBALPARAM = 0
def publisher():
    global state
    pub = rospy.Publisher(TOPIC_NAME, OverrideRCIn, queue_size=10)
    rospy.init_node("cs_491_controller", anonymous=True)
    rate = rospy.Rate(10)

    channels = [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def on_receive_config(config, level):
        global GLOBALPARAM
        # channels[0] = config["roll"]
        # channels[1] = config["pitch"]
        # channels[2] = config["throttle"]
        # channels[3] = config["yaw"]
        print('updating gain')
        GLOBALPARAM = config["channel1"]
        # channels[5] = config["channel2"]
        # channels[6] = config["channel3"]
        # channels[7] = config["channel4"]
        # channels[8] = config["channel5"]
        # channels[9] = config["channel6"]
        # channels[10] = config["channel7"]
        # channels[11] = config["channel8"]
        # channels[12] = config["channel9"]
        # channels[13] = config["channel10"]
        # channels[14] = config["channel11"]
        # channels[15] = config["channel12"]
        # channels[16] = config["channel13"]
        # channels[17] = config["channel14"]

        # rospy.loginfo("Updated channels to {}".format(channels))

        return config

    srv = Server(TutorialsConfig, on_receive_config)
    set_mode = rospy.ServiceProxy("/minihawk_SIM/mavros/set_mode", SetMode)
    arm = rospy.ServiceProxy("/minihawk_SIM/mavros/cmd/arming", CommandBool)

    # Begin launching.
    # TODO: Assumes that the waypoints are already loaded
    set_mode(0, "AUTO")
    arm(True) 
    previously_triggered = False
    last_check = time.time()

    # Like and subscribe to the tag detection topic
    def process_tag_detection(msg):
        global state
        global detection_pose
        if len(msg.detections) > 0:
            detection_pose = msg.detections[-1].pose
            last_check = time.time()
            previously_triggered = True

            # If we're still looking for the tag... we've found it! Begin Centering!
            if state == RobotState.SEEKING and abs(detection_pose.pose.pose.position.x) < 3.0 and abs(detection_pose.pose.pose.position.y) < 3.0 :
                state = RobotState.CENTERING
                set_mode(0, "QLOITER")
                print('Enter Centering Mode')

            # rospy.loginfo(msg.detections[0].pose) # http://docs.ros.org/en/indigo/api/apriltags_ros/html/msg/AprilTagDetectionArray.html

    sub = rospy.Subscriber(TAG_DETECTION, AprilTagDetectionArray, process_tag_detection)

    roll_PID = PID(60, 0.0, 0.0, setpoint=0)

    pitch_PID = PID(10, 0.0, 0.0, setpoint=0)

    yaw_PID = PID(0, 0, 0, setpoint=1)

    altitude_PID = PID(10, 0.0, 1000, setpoint=3)
    

    roll_PID.output_limits = (1000, 2000)
    pitch_PID.output_limits =  (1000, 2000)
    altitude_PID.output_limits =  (1000, 2000)
    yaw_PID.output_limits =  (1000, 2000)


    while not rospy.is_shutdown():
        if state == RobotState.CENTERING:    
            # Actively center on the target.



            rot = R.from_quat([detection_pose.pose.pose.orientation.x, detection_pose.pose.pose.orientation.y, detection_pose.pose.pose.orientation.z, detection_pose.pose.pose.orientation.w])
            euler = rot.as_euler('zxy', degrees = True)
            yaw = euler[0]
            # print('yaw', detection_pose.pose.pose.orientation.x)

            pitch_PID.Kp = GLOBALPARAM
            print(pitch_PID.Kp)


            roll_command = roll_PID(detection_pose.pose.pose.position.x)
            pitch_command = pitch_PID(detection_pose.pose.pose.position.y)
            throttle_command = altitude_PID(detection_pose.pose.pose.position.z)
            yaw_command = yaw_PID(yaw)

            # channels[0] = roll_command
            channels[1] = pitch_command
            # channels[2] = throttle_command
            # channels[3] = yaw_command

            # channels[2] = 2000




            print("Roll: {:4.4f} Pitch: {:4.4f} Throttle: {:4.4f} Yaw: {:4.4f}".format(roll_command, pitch_command, throttle_command, yaw_command))
            # print('x', detection_pose.pose.pose.position.x)
            # print('y', detection_pose.pose.pose.position.y)
            # print('z', detection_pose.pose.pose.position.z)

            the_data = OverrideRCIn()
            the_data.channels = channels

            pub.publish(the_data)
            # set_mode(0, "QHOVER")

            # If it's been 2 seconds since we last saw the tag, just keep descending.
            # if previously_triggered and time.time() - last_check > 2: 
            #     set_mode(0, "QLAND")
            #     state = RobotState.DESCENDING

        rate.sleep()






if __name__ == "__main__":
    publisher()