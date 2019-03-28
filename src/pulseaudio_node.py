#!/usr/bin/env python2
import rospy
import time
import subprocess
from std_srvs.srv import SetBool, SetBoolResponse


def submit_module(module, mode, *args):
    command = [
        "pactl", mode + "-module", module,
    ]
    if args and mode == 'load':
        command.extend(args)

    time.sleep(0.1)

    cmd = " ".join(command)
    rospy.loginfo('Executing {} ...'.format(cmd))

    process = subprocess.Popen(cmd, shell=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    out, err = process.communicate()
    if err:
        rospy.logerr(err[:-1])
    elif out:
        rospy.loginfo(out[:-1])

    return process.returncode


class Controller:
    def __init__(self):
        rospy.Service(
            '/pulseaudio/allow_network',
            SetBool,
            self._allow_network,
        )

    def _allow_network(self, req):
        data = req.data

        resp = SetBoolResponse()
        resp.success = False

        if submit_module(
            'module-zeroconf-discover',
            'load' if data else 'unload',
        ) != 0:
            resp.message = "Couldn't load module zeroconf discover"
            return resp

        if submit_module(
            'module-native-protocol-tcp',
            'load' if data else 'unload',
            'auth-anonymous=1',
        ) != 0:
            resp.message = "Couldn't load module native protocol"
            return resp

        if submit_module(
            'module-zeroconf-publish',
            'load' if data else 'unload',
        ) != 0:
            resp.message = "Couldn't load module zeroconf publish"
            return resp

        resp.success = True
        return resp

    @staticmethod
    def spin():
        rospy.spin()

    def shutdown(self):
        rospy.signal_shutdown('shutdown occured')


if __name__ == "__main__":
    rospy.init_node('pulseaudio_controller')

    controller = Controller()

    try:
        controller.spin()
    except BaseException as err:
        rospy.logerr(err)
    finally:
        controller.shutdown()
