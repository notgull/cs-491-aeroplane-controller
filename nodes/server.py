#!/usr/bin/env python
# NO IMPLIED WARRANTY
# Written by John Nunley, <your names here>

import rospy
import enum
import time
import math

from pid import PID

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
    LANDING = 4

    def is_centering(self):
        return self == RobotState.CENTERING
    def is_descending(self):
        return self == RobotState.DESCENDING
    def is_landing(self):
        return self == RobotState.LANDING

def clampDir(d):
    if d > MAX_DIRECTION:
        return MAX_DIRECTION
    elif d < -MAX_DIRECTION:
        return -MAX_DIRECTION
    else:
        return d

def applyDirection(
    roll, pitch, throttle, yaw):

    res = [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
    res[0] += clampDir(roll)
    res[1] += clampDir(pitch)
    res[2] += clampDir(throttle)
    res[3] += clampDir(yaw)
    return res

# Like and subscribe to the tag detection topic
def process_tag_detection(msg):
    global state
    global detection_pose
    global CENTERING_START
    global LAST_POSE_TIME
    global set_mode


    if len(msg.detections) > 0:
        LAST_POSE_TIME = time.time()
        detection_pose = msg.detections[-1].pose

        # If we're still looking for the tag... we've found it! Begin Centering!
        if state == RobotState.SEEKING and abs(detection_pose.pose.pose.position.x) < 5.0 and abs(detection_pose.pose.pose.position.y) < 5.0 :
            state = RobotState.CENTERING
            set_mode(0, "QLOITER")
            print('Enter Centering Mode')
            CENTERING_START = time.time()

            



set_mode = rospy.ServiceProxy("/minihawk_SIM/mavros/set_mode", SetMode)
arm = rospy.ServiceProxy("/minihawk_SIM/mavros/cmd/arming", CommandBool)

CENTERING_START = None
detection_pose = None
LAST_POSE_TIME = time.time()

RESTING = [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
MAX_DIRECTION = 250

x_setpoint = 0.0
y_setpoint = 0.0
z_setpoint = 12.0
diff = 1.0
settle_time = 6.0




state = RobotState.SEEKING
print('Enter Seeking Mode:')

def main():
    rospy.init_node("cs_491_controller", anonymous=True)
    rate = rospy.Rate(10)

    pub = rospy.Publisher(TOPIC_NAME, OverrideRCIn, queue_size=10) 
    sub = rospy.Subscriber(TAG_DETECTION, AprilTagDetectionArray, process_tag_detection)

    global state
    global x_setpoint
    global y_setpoint
    global z_setpoint
    global diff
    global set_mode
    global settle_time
    global CENTERING_START

    # init pids
    xcoord_PID = PID(25.0, 2.5, 2.0, setpoint=x_setpoint)
    ycoord_PID = PID(21.5, 1.0, 0.5, setpoint=y_setpoint)
    zcoord_PID = PID(10.0, 1.0, 0.0, setpoint=z_setpoint)

    # Begin launching.
    set_mode(0, "AUTO")
    arm(True)

    last_diffpoint = None
    last_update_time = None

    while not rospy.is_shutdown():
        if state.is_centering() or state.is_descending():

            # Actively center on the target.
            posn = detection_pose.pose.pose.position
            current_time = time.time()
            if last_update_time == None:
                last_update_time = CENTERING_START
            dt = current_time - last_update_time
            last_update_time = current_time

            # Get the amount we should move forwards/backwards.
            roll_factor = xcoord_PID(posn.x, dt)
            pitch_factor = ycoord_PID(posn.y, dt)
            throttle_factor = zcoord_PID(posn.z, dt)

            # Apply it to the channels
            if state.is_descending():   # if descending also use PID on throttle/altitude
                desiredMove = applyDirection(-roll_factor, pitch_factor, throttle_factor, 0)
            else:
                desiredMove = applyDirection(-roll_factor, pitch_factor, 0, 0)

            print("X Error: {:4.4f}, Y Error: {:4.4f}, Z Error: {:4.4f}".format(posn.x, posn.y, posn.z - z_setpoint))

            # If our difference is marginal, progress to the next stage of the landing cycle.
            if state.is_centering():
                if abs(y_setpoint - posn.y) < diff and abs(x_setpoint - posn.x) < diff:
                    if last_diffpoint is None:
                        last_diffpoint = current_time
                    elif current_time - last_diffpoint > settle_time:
                        state = RobotState.DESCENDING
                        last_diffpoint = None
                        print("\n\n\n...Beginning descent...\n\n\n")
                else:
                    last_diffpoint = current_time

            elif state.is_descending():
                if abs(y_setpoint - posn.y) < diff and abs(x_setpoint - posn.x) < diff and abs(z_setpoint - posn.z) < diff:
                    if last_diffpoint is None:
                        last_diffpoint = current_time
                    elif current_time - last_diffpoint > settle_time:
                        state = RobotState.LANDING
                        set_mode(0, "QLAND")
                        print("\n\n\n...Releasing control from PID controllers...\n\n\n")
                else:
                    last_diffpoint = current_time

                # if not_detected:
                #     if last_detect_point is None:
                #         last_detect_point = current_time
                #     elif current_time - last_detect_point > 3.0:
                #         state = RobotState.LANDING
                #         set_mode(0, "QLAND")
                # else:
                #     last_detect_point = None

    
            print("Roll: {:4.4f} Pitch: {:4.4f} Throttle: {:4.4f} Yaw: {:4.4f}"
                  .format(desiredMove[0], desiredMove[1], desiredMove[2], desiredMove[3]))

            # Send the move to the aerial robot
            the_data = OverrideRCIn()
            the_data.channels = desiredMove
            pub.publish(the_data)


        rate.sleep()

if __name__ == "__main__":
    main()
