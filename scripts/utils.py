import asyncio
import threading
import rospy
import requests
from mavproxy_ros.srv import ProcessRequest, ProcessRequestResponse
import json

def SUCCESS_RESPONSE(msg = "OK"):
    return {"msg": msg, "status": "success"}

def ERROR_RESPONSE(msg):
    return {"msg": msg, "status": "error"}

class Node:
    def __init__(self):
        self.loop = None
        rospy.init_node(self.__class__.__name__.lower())
    
    @classmethod
    def route(cls, path, method):
        def wrapper(func):
            func.path = path
            func.method = method
            return func
        return wrapper
    
    def register(self):
        for path, method, func in self.route:
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
        for name, attr in cls.__dict__.items():
            if hasattr(attr, "path"):
                route.append([attr.path, attr.method, attr])
        cls.route = route
    
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
        rospy.spin()
        
        
    