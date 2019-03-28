#!/usr/bin/env python2
import rospy
import pulseaudio
from pulseaudio_controller.msg import Item
from pulseaudio_controller.srv import ListItems, ListItemsResponse, SetItem
from std_srvs.srv import SetBool, SetBoolResponse

class Controller:
    def __init__(self):
        rospy.Service(
            '/pulseaudio/allow_network',
            SetBool,
            self._allow_network,
        )

        rospy.Service(
            '/pulseaudio/loopback',
            SetBool,
            self._loopback,
        )

        rospy.Service(
            '/pulseaudio/sources',
            ListItems,
            self._list_sources,
        )
        rospy.Service(
            '/pulseaudio/sinks',
            ListItems,
            self._list_sinks,
        )

        rospy.Service(
            '/pulseaudio/set_sink',
            SetItem,
            self._set_sink_item,
        )
        rospy.Service(
            '/pulseaudio/set_source',
            SetItem,
            self._set_source_item,
        )

    def _set_sink_item(self, req):        
        return pulseaudio.set_default_sink(req.name)

    def _set_source_item(self, req):
        return pulseaudio.set_default_source(req.name)

    def _list_sinks(self, req):
        resp = ListItemsResponse()
        resp.items = []
        for sink in pulseaudio.list_sinks():
            item = Item()
            item.description = sink["description"]
            item.number = sink["number"]
            item.name = sink["name"]
            resp.items.append(item)
        return resp

    def _list_sources(self, req):
        resp = ListItemsResponse()
        resp.items = []
        for source in pulseaudio.list_sources():
            item = Item()
            item.description = source["description"]
            item.number = source["number"]
            item.name = source["name"]
            resp.items.append(item)
        return resp

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

    def _loopback(self, req):
        data = req.data
        
        resp = SetBoolResponse()
        resp.success = False

        if pulseaudio.submit_module(
            'module-loopback',
            'load' if data else 'unload',            
        ) != 0:
            resp.message = "Couldn't load module native protocol"
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
