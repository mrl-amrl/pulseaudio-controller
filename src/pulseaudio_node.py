#!/usr/bin/env python2
import rospy
import pulseaudio
from std_srvs.srv import SetBool, SetBoolResponse


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

        if pulseaudio.submit_module(
            'module-zeroconf-discover',
            'load' if data else 'unload',
        ) != 0:
            resp.message = "Couldn't load module zeroconf discover"
            return resp

        if pulseaudio.submit_module(
            'module-native-protocol-tcp',
            'load' if data else 'unload',
            'auth-anonymous=1',
        ) != 0:
            resp.message = "Couldn't load module native protocol"
            return resp

        if pulseaudio.submit_module(
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
