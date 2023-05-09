from cs_491_controller.cfg import TutorialsConfig
from dynamic_reconfigure.server import Server


def on_receive_config(config, level):
    channels[0] = config["roll"]
    channels[1] = config["pitch"]
    channels[2] = config["throttle"]
    channels[3] = config["yaw"]
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
