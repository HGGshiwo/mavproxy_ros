import asyncio
from functools import partial
import threading
import rospy
import requests
from mavproxy_ros.srv import ProcessRequest, ProcessRequestResponse
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue, OverrideRCIn
import json
import time

def SUCCESS_RESPONSE(msg = "OK"):
    return {"msg": msg, "status": "success"}

def ERROR_RESPONSE(msg):
    return {"msg": msg, "status": "error"}

class Node:
    def __init__(self):
        self.loop = None
        self.name = self.__class__.__name__.lower()
        rospy.init_node(self.name)
        self.param_set_service = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.rc_override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.set_config()
    
    def get_param(self, name):
        return rospy.get_param(f"{self.name}/{name}", {})
     
    def set_config(self):
        param_cfg = self.get_param("param")
        for param_name, value in param_cfg.items():
            self.set_param(param_name, value)
            time.sleep(0.1) # 避免过快发送请求
            
        rc_cfg = self.get_param("rc")
        for rc_name, value in rc_cfg.items():
            rc_name = int(rc_name)
            self.set_rc(rc_name, value)
            time.sleep(0.1)
    
    def set_param(self, param_name: str, value):
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
    
    def set_rc(self, channel: int, pwm: int) -> bool:
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
    def ros(cls, topic, topic_type):
        def wrapper(func):
            func.topic = topic
            func.topic_type = topic_type
            return func
        return wrapper
    
    def register(self):
        self.register_route()
        self.register_ros()
        
    def register_ros(self):
        for topic, topic_type, func in self.ros_list:
            rospy.Subscriber(topic, topic_type, partial(func, self), queue_size=1)
        
    def register_route(self):
        for path, method, func in self.route_list:
            if path.startswith("/"):
                topic = "/" + path[1:].replace("/", "_")
            else:
                topic = path.replace("/", "_")
            # 闭包陷阱
            def _cb(data, func=func):
                try:
                    res = func(self, json.loads(data.request))
                    return ProcessRequestResponse(response=json.dumps(res))
                except Exception as e:
                    import traceback
                    return ProcessRequestResponse(
                        response=json.dumps({
                        "status": "error", 
                        "msg": str(e), 
                        "detail": traceback.format_exc()
                    }))
            rospy.Service(topic, ProcessRequest, _cb)

            requests.post(
                "http://localhost:8000/register", 
                json={"path": path, "method": method, "topic": path}
            )
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
            await asyncio.sleep(0)
    
    def run(self):
        self.register()
        t = threading.Thread(
            target=lambda: asyncio.run(self.async_run()),
            daemon=True
        )
        t.start()
        rospy.loginfo(f"{self.__class__.__name__.lower()} start")
        rospy.spin()
        
        
class FPSHelper:
    def __init__(self, fps=-1, ps_cb=None):
        self.target_fps = fps
        self.start = time.time()
        self.cnt_start = time.time()
        self.frame_cnt = 0
        self.fps = 0
        self.ps_cb = ps_cb

    def step(self, block=True):
        now = time.time()
        if now - self.cnt_start > 1:
            self.fps = self.frame_cnt / (now - self.cnt_start)
            self.frame_cnt = 0
            self.cnt_start = time.time()
            if self.ps_cb is not None:
                self.ps_cb(self.fps)
        
        self.frame_cnt += 1
        if block:
            if self.target_fps > 0 and now - self.start < 1 / self.target_fps:
                time.sleep(1 / self.target_fps - now + self.start)
            self.start = time.time()
            # self.frame_cnt += 1
        else:
            if self.target_fps > 0 and now - self.start < 1 / self.target_fps:
                trigger = False
            else:
                trigger = True
                self.start = time.time()
                # self.frame_cnt += 1
            return trigger