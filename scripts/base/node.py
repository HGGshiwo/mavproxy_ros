import asyncio
from functools import partial
import threading
import rospy
from mavproxy_ros.srv import ProcessRequest, ProcessRequestResponse
from mavproxy_ros.srv import Register
from mavros_msgs.srv import ParamSet, ParamGet
from mavros_msgs.msg import ParamValue, OverrideRCIn
from std_msgs.msg import Empty
import json
import time
from typing import Any


def SUCCESS_RESPONSE(msg="OK"):
    return {"msg": msg, "status": "success"}


def ERROR_RESPONSE(msg):
    return {"msg": msg, "status": "error"}


class Node:
    def __init__(self):
        self.loop = None
        self.name = self.__class__.__name__.lower()
        rospy.init_node(self.name)
        rospy.wait_for_service("/mavros/param/set")
        rospy.wait_for_service("register")
        rospy.wait_for_service("/mavros/param/pull")
        self.param_set_service = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        self.register_service = rospy.ServiceProxy("register", Register)
        self.rc_override_pub = rospy.Publisher(
            "/mavros/rc/override", OverrideRCIn, queue_size=10
        )
        rospy.Subscriber("do_register", Empty, self.register_route)

    def _get_param(self, name):
        return rospy.get_param(f"{self.name}/{name}", {})

    def wait_for_param(self, param_name, timeout=300):
        rospy.loginfo("Waiting for parameter sync...")
        start_time = rospy.get_time()
        param_get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
        while not rospy.is_shutdown() and rospy.get_time() - start_time < timeout:
            try:
                resp = param_get(param_name)
                if resp.success:
                    rospy.loginfo("Parameter sync complete")
                    return True
            except rospy.ServiceException as e:
                rospy.logwarn("Failed to get param: %s", e)
            rospy.sleep(1.0)
        rospy.logerr("Parameter sync timeout")
        return False

    def _set_config(self):
        param_cfg = self._get_param("param")
        print(self.name, param_cfg)
        for param_name, value in param_cfg.items():
            self.wait_for_param(param_name)
            self._set_param(param_name, value)
            time.sleep(0.1)  # 避免过快发送请求

        rc_cfg = self._get_param("rc")
        for rc_name, value in rc_cfg.items():
            rc_name = int(rc_name)
            self._set_rc(rc_name, value)
            time.sleep(0.1)

    def _set_param(self, param_name: str, value):
        """
        设置单个飞控参数
        :param param_name: 参数名称
        :param value: 参数值
        :return: 是否成功
        """
        try:
            # 创建参数对象
            param = ParamValue()
            if isinstance(value, int):
                param.integer = int(value)
            else:
                param.real = float(value)

            # 调用服务
            response = self.param_set_service(param_id=param_name, value=param)

            if response.success:
                rospy.loginfo(f"参数设置成功: {param_name} = {value}")
                return True
            else:
                rospy.logerr(f"参数设置失败: {param_name}")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False

    def _set_rc(self, channel: int, pwm: int) -> bool:
        """
        设置单个 RC 通道
        :param channel: 通道号 (0-7)
        :param pwm: PWM 值 (1000-2000)
        :return: 是否成功
        """
        try:
            # 创建 RC 覆盖消息
            override = OverrideRCIn()
            override.channels[channel] = pwm

            # 发布消息
            self.rc_override_pub.publish(override)

            rospy.loginfo(f"设置 RC 通道 {channel}: {pwm} PWM")
            return True

        except Exception as e:
            rospy.logerr(f"设置 RC 通道失败: {e}")
            return False

    @classmethod
    def route(cls, path, method):
        def wrapper(func):
            func.path = path
            func.method = method
            return func

        return wrapper

    @classmethod
    def ros(cls, topic: str, topic_type: Any):
        def wrapper(func):
            func.topic = topic
            func.topic_type = topic_type
            func.last = time.time()
            return func

        return wrapper

    def register(self):
        self.register_route()  # 出现了do_register后再注册
        self.register_ros()

    def register_ros(self):
        for topic, topic_type, func in self.ros_list:
            rospy.Subscriber(topic, topic_type, partial(func, self), queue_size=1)

    def register_route(self, data=None):
        for path, method, func in self.route_list:
            topic = path.split("{")[0]  # 忽略url参数
            topic = topic.split("?")[0]  # 忽略查询参数

            if topic.startswith("/"):
                topic = "/" + topic[1:].replace("/", "_")
            else:
                topic = topic.replace("/", "_")

            # 闭包陷阱
            def _cb(data, func=func):
                try:
                    res = func(self, **json.loads(data.request))
                    return ProcessRequestResponse(response=json.dumps(res))
                except Exception as e:
                    import traceback

                    return ProcessRequestResponse(
                        response=json.dumps(
                            {
                                "status": "error",
                                "msg": str(e),
                                "detail": traceback.format_exc(),
                            }
                        )
                    )

            try:
                rospy.Service(topic, ProcessRequest, _cb)
            except rospy.service.ServiceException:
                pass  # 可能重复被调用
            res = self.register_service(path=path, method=method, topic=topic)
            response = json.loads(res.response)
            if not response["status"] == "success":
                raise ValueError("register service died")
            # requests.post(
            #     "http://localhost:8000/register",
            #     json={"path": path, "method": method, "topic": path}
            # )
            print(f"register: HTTP: {path}, {method} -> ROS: {topic} {func.__name__}")

    def __init_subclass__(cls):
        route = []
        ros = []
        for name, attr in cls.__dict__.items():
            if hasattr(attr, "path"):
                route.append([attr.path, attr.method, attr])
            if hasattr(attr, "topic"):
                ros.append([attr.topic, attr.topic_type, attr])
        cls.route_list = route
        cls.ros_list = ros

    def run_in_loop(self, task):
        asyncio.run_coroutine_threadsafe(task, self.loop)

    async def async_run(self):
        self.loop = asyncio.get_running_loop()
        while not rospy.is_shutdown():
            await asyncio.sleep(100)

    def run(self):
        self.register()
        self._set_config()
        rospy.loginfo(f"{self.__class__.__name__.lower()} start")

        asyncio.run(self.async_run())
