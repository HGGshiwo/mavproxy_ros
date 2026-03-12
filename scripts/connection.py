#!/usr/bin/python3
# -*- coding: utf-8 -*-
import logging
from pathlib import Path
from typing import Any, Dict

import rospy
from event_callback import http, ros
from event_callback.components.http.core import HTTPConfig
from event_callback.components.http.message_handler import EventMessageHandler
from event_callback.components.http.message_handler import MessageType as _MessageType
from event_callback.components.ros import ROSConfig
from event_callback.core import CallbackManager
from event_callback.ros_utils import rospy_init_node
from event_callback.utils import print_logger_info, setup_logger
from mavros_msgs.msg import GPSRAW, State, StatusText
from mavros_msgs.srv import StreamRate, StreamRateRequest
from std_msgs.msg import Float64, UInt32

from mavproxy_ros.ui import *

logger = logging.getLogger(__file__)


class MessageType(_MessageType):
    EVENT = EventMessageHandler.create("event")


class Connection(CallbackManager):

    def __init__(self, component_config=None, mixins=None):
        super().__init__(component_config, mixins)
        self.armed = None  # 是否解锁
        self._set_rate()

    def _publish_state(self, state: Dict[str, Any]):
        http.ws_send(self, state, MessageType.STATE)

    def _publish_event(self, event: Dict[str, Any]):
        http.ws_send(self, event, MessageType.EVENT)

    def _publish_error(self, error: Dict[str, Any]):
        http.ws_send(self, error, MessageType.ERROR)

    def _set_rate(self):
        rospy.wait_for_service("/mavros/set_stream_rate", timeout=3)
        set_rate = rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)
        req = StreamRateRequest()
        req.stream_id = 0
        req.message_rate = 10
        req.on_off = True
        resp = set_rate(req)
        logger.info("set stream done")


    # print_logger_info()
    @ros.topic("/mavros/state", State)
    def mode_cb(self, data: State):
        if data.armed == False and self.armed != False:
            # self._publish_event(dict(event="disarm"))
            pass
        elif data.armed == True and self.armed != True:
            self._publish_event(dict(event="arm"))
        arm_desc = "已解锁" if data.armed else "未解锁"
        connected_desc = "已连接" if data.connected else "未连接"
        self._publish_state(
            dict(mode=data.mode, arm=data.armed, connected=data.connected)
        )
        self.armed = data.armed

    @ros.topic("/mavros/global_position/raw/satellites", UInt32)
    def gps_cb(self, data: UInt32):
        self._publish_state(dict(gps_nsats=data.data))

    @ros.topic("/mavros/gpsstatus/gps1/raw", GPSRAW)
    def gps_fix_cb(self, data: GPSRAW):
        self._publish_state(dict(gps_fix_type=data.fix_type))

    @ros.topic("/mavros/global_position/rel_alt", Float64, frequency=2)
    def alt_cb(self, data: Float64):
        self._publish_state(dict(rel_alt=data.data))

    @ros.topic("/mavros/statustext/recv", StatusText)
    def state_cb(self, data: StatusText):
        if data.severity > StatusText.WARNING:
            return

        logger.error(f"{data.severity}, {data.text}")
        self._publish_error(dict(error=data.text))


if __name__ == "__main__":
    setup_logger(Path(__file__).parent.parent.joinpath("log").absolute())
    rospy_init_node("connection")
    Connection(
        [
            HTTPConfig(
                port=8000,
                register=True,
                static_dir=Path(__file__).parent.parent / "static",
            ),
            ROSConfig(),
        ]
    )
    rospy.spin()
