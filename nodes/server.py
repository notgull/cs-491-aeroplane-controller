#!/usr/bin/env python
# NO IMPLIED WARRANTY
# Written by John Nunley, <your names here>

import rospy
import enum
import time
import math

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
    CENTERING_AND_DESCENDING = 2
    DESCENDING = 4

    def is_centering(self):
        return self == RobotState.CENTERING or self == RobotState.CENTERING_AND_DESCENDING


def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
    #       {str_param}, {bool_param}, {size}""".format(**config))
    return config

state = RobotState.SEEKING
print('Enter Seeking Mode:')

detection_pose = None

RESTING = [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
SCALE_FACTOR = 3
MAX_DIRECTION = 250

def clampDir(d):
    if d > MAX_DIRECTION:
        return MAX_DIRECTION
    elif d < -MAX_DIRECTION:
        return -MAX_DIRECTION
    else:
        return d

def applyDirection(
    roll, pitch, throttle, yaw
):
    res = [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
    res[0] += clampDir(roll)
    res[1] += clampDir(pitch)
    res[2] += clampDir(throttle)
    res[3] += clampDir(yaw)
    return res

# Approximate center in terms of the X coordinate
CENTER_X = 3
GLOBALPARAM = 0
CENTERING_START = None
LAST_POSE_TIME = time.time()

def publisher():
    global state
    pub = rospy.Publisher(TOPIC_NAME, OverrideRCIn, queue_size=10)
    rospy.init_node("cs_491_controller", anonymous=True)
    rate = rospy.Rate(10)

    channels = RESTING

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

    # Like and subscribe to the tag detection topic
    def process_tag_detection(msg):
        global state
        global detection_pose
        global CENTERING_START
        global LAST_POSE_TIME

        if len(msg.detections) > 0:
            LAST_POSE_TIME = time.time()
            detection_pose = msg.detections[-1].pose

            # If we're still looking for the tag... we've found it! Begin Centering!
            if state == RobotState.SEEKING and abs(detection_pose.pose.pose.position.x) < 3.0 and abs(detection_pose.pose.pose.position.y) < 3.0 :
                state = RobotState.CENTERING
                set_mode(0, "QLOITER")
                print('Enter Centering Mode')
                CENTERING_START = time.time()
                
    sub = rospy.Subscriber(TAG_DETECTION, AprilTagDetectionArray, process_tag_detection)

    # Try to center as close as we can
    x_setpoint = 0.0
    # y_setpoint = -3.0
    y_setpoint = 0.0
    z_setpoint = 10.0

    diff = 1.0
    last_diffpoint = None

    xcoord_PID = PID(25.0, 2.5, 2.0, setpoint=x_setpoint)
    ycoord_PID = PID(25.0, 0.5, 2.0, setpoint=y_setpoint)
    zcoord_PID = PID(25.0, 5.0, 10.0, setpoint=z_setpoint)

    last_update_time = time.time()
    while not rospy.is_shutdown():
        if state.is_centering():
            # If we haven't gotten the tag for a while, we're probably 
            # Actively center on the target.
            posn = detection_pose.pose.pose.position
            current_time = time.time()
            dt = current_time - last_update_time
            last_update_time = current_time

            # Get the amount we should move forwards/backwards.
            roll_factor = xcoord_PID(posn.x, dt)
            pitch_factor = ycoord_PID(posn.y, dt)
            throttle_factor = zcoord_PID(posn.z, dt)

            # Apply it to the channels
            # if state.is_descending():
            #     desiredMove = applyDirection(-roll_factor, pitch_factor, throttle_factor, 0)
            # else:
            desiredMove = applyDirection(-roll_factor, pitch_factor, 0, 0)

            print("X Error: {:4.4f}, Y Error: {:4.4f}, Z Error: {:4.4f}".format(posn.x, posn.y, posn.z))

            # If our difference is marginal, start the landing cycle.
            if state == RobotState.CENTERING:
                if abs(y_setpoint - posn.y) < diff and abs(x_setpoint - posn.x) < diff:
                    if last_diffpoint is None:
                        last_diffpoint = current_time
                    elif current_time - last_diffpoint > 10:
                        state = RobotState.CENTERING_AND_DESCENDING
                        set_mode(0, "QLAND")
                        print("Entering the descent...")
                else:
                    last_diffpoint = current_time
            elif state == RobotState.CENTERING_AND_DESCENDING:
                # If we haven't gotten a new read in a while, enter the resting position.
                if current_time - LAST_POSE_TIME > 0.5:
                    state = RobotState.DESCENDING
                    desiredMove = [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
                    print("Releasing control from PID controllers...")


            print(
                "Roll: {:4.4f} Pitch: {:4.4f} Throttle: {:4.4f} Yaw: {:4.4f}"
                  .format(desiredMove[0], desiredMove[1], desiredMove[2], desiredMove[3])
            )

            # Send the move to the aerial robot
            the_data = OverrideRCIn()
            the_data.channels = desiredMove
            pub.publish(the_data)





        rate.sleep()

if __name__ == "__main__":
    publisher()
