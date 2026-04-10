#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import logging
import threading
from pathlib import Path
from typing import Any, Dict, Type

import rospy
from event_callback.components.http.core import HTTPComponent
from event_callback.components.ros import ROSComponent
from event_callback.core import BaseManager
from event_callback.ros_utils import rospy_init_node
from event_callback.utils import setup_logger, throttle
from mavros_msgs.msg import GPSRAW, State, StatusText
from mavros_msgs.srv import StreamRate, StreamRateRequest
from std_msgs.msg import Float64, String, UInt32

from mavproxy_ros.message_handler import (
    MessageHandler,
    MessageType,
    StateMessageHandler,
)
from mavproxy_ros.ui import *
from mavproxy_ros.utils import wait_for_debugger

logger = logging.getLogger(__file__)


class Connection(BaseManager):
    def __init__(self, ros_comp: ROSComponent, http_comp: HTTPComponent):
        super().__init__(ros_comp, http_comp)
        self.http_comp = http_comp
        self.armed = None  # 是否解锁
        self.handler_map = {}
        self.handler_lock = threading.Lock()
        http_comp.start_server(port=8000)
        self.ws_sub = rospy.Subscriber("/mavproxy/ws", String, self._ws_callback)
        logger.info("HTTPComponent create ROS topic: do_register")
        self._set_rate()

    def _publish(self, data, msg_type: MessageType):
        handler_type: Type[MessageHandler] = msg_type.value
        with self.handler_lock:
            if msg_type not in self.handler_map:
                self.handler_map[msg_type] = handler_type()
            handler = self.handler_map[msg_type]
            data = handler.on_send(data)
        if data is None:
            return
        self.http_comp.publish(data)

    def _publish_state(self, state: Dict[str, Any]):
        self._publish(state, MessageType.STATE)

    def _publish_event(self, event: Dict[str, Any]):
        self._publish(event, MessageType.EVENT)

    def _publish_error(self, error: Dict[str, Any]):
        self._publish(error, MessageType.ERROR)

    def _set_rate(self):
        rospy.wait_for_service("/mavros/set_stream_rate", timeout=3)
        set_rate = rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)
        req = StreamRateRequest()
        req.stream_id = 0
        req.message_rate = 10
        req.on_off = True
        resp = set_rate(req)
        logger.info("set stream done")

    def _ws_callback(self, data: Any):
        json_data = json.loads(data.data)
        data_type: str = json_data.get("type", "state")
        json_data.update(dict(type=data_type))
        data_type = getattr(MessageType, data_type.upper())
        # self.http_comp.publish(json_data)
        self._publish(json_data, data_type)

    @HTTPComponent.on_connect()
    def on_connect(self, conn_id):
        with self.handler_lock:
            state_handler: StateMessageHandler = self.handler_map.get(
                MessageType.STATE, None
            )
            if state_handler is not None:
                data = state_handler.on_connect()
            else:
                data = {}
            self.http_comp.send_json(conn_id, data)

    # print_logger_info()
    @ROSComponent.on_topic("/mavros/state", State)
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

    @ROSComponent.on_topic("/mavros/global_position/raw/satellites", UInt32)
    def gps_cb(self, data: UInt32):
        self._publish_state(dict(gps_nsats=data.data))

    @ROSComponent.on_topic("/mavros/gpsstatus/gps1/raw", GPSRAW)
    def gps_fix_cb(self, data: GPSRAW):
        self._publish_state(dict(gps_fix_type=data.fix_type))

    @ROSComponent.on_topic("/mavros/global_position/rel_alt", Float64)
    @throttle(frequency=2)
    def alt_cb(self, data: Float64):
        self._publish_state(dict(rel_alt=data.data))

    @ROSComponent.on_topic("/mavros/statustext/recv", StatusText)
    def state_cb(self, data: StatusText):
        if data.severity > StatusText.WARNING:
            return

        logger.error(f"{data.severity}, {data.text}")
        self._publish_error(dict(error=data.text))


if __name__ == "__main__":
    setup_logger(Path(__file__).parent.parent.joinpath("log").absolute())
    rospy_init_node("connection")
    # wait_for_debugger()
    http_comp = HTTPComponent(
        register=True,
        static_dir=Path(__file__).parent.parent / "static",
    )
    ros_comp = ROSComponent()
    Connection(ros_comp, http_comp)
    rospy.spin()
