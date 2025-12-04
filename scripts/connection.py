#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, UInt32, Float64
from mavros_msgs.msg import State
from mavproxy_ros.srv import ProcessRequest, ProcessRequestResponse
from mavproxy_ros.srv import Register, RegisterResponse
import uvicorn
import asyncio
import json
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from fastapi import WebSocket, WebSocketDisconnect, Depends
import threading
from pathlib import Path
from uvicorn.config import Config
from base.utils import SUCCESS_RESPONSE
from base.camera import VideoBuilder
from mavros_msgs.srv import StreamRate, StreamRateRequest
from mavros_msgs.msg import StatusText
from std_msgs.msg import Empty
import time
from fastapi.middleware.cors import CORSMiddleware
import datetime

loop = None
def run_in_loop(task):
    global loop
    if loop is None:
        return
    asyncio.run_coroutine_threadsafe(task, loop)


class RequestHandler:
    def __init__(self, path, method, topic):
        self.res = None
        self.path = path
        self.method = method
        self.topic = topic
        rospy.wait_for_service(topic)
        self.service = rospy.ServiceProxy(topic, ProcessRequest)
            
    async def __call__(self, request: Request):
        request_data = {}
        request_data.update(request.path_params)
        request_data.update(request.query_params)
        try:
            body = await request.json()
            request_data.update(body)
        except Exception as e:
            pass  # 没有body或者不是json格式
            
        response = self.service(request=json.dumps(request_data))
        return json.loads(response.response)

class WSManager:
    def __init__(self):
        self.ws_list = []
        self.data = {"type": "state", "connected": False, "record": False, "event": []}
        self.lock = threading.Lock()
        
    async def add(self, ws):
        await ws.send_json(self.data)  # 直接等待
        with self.lock:
            self.ws_list.append(ws)
    
    def remove(self, ws):
        with self.lock:
            if ws in self.ws_list:
                self.ws_list.remove(ws)
    
    def publish(self, data, data_type="state"):
        with self.lock:
            if data_type == "state":
                self.data.update(data)
            elif data_type == "event":
                now = datetime.datetime.now()
                # 格式化为字符串
                now_str = now.strftime('%Y-%m-%d %H:%M:%S')
                self.data["event"].append({"time": now_str, **data})
            ws_list_copy = self.ws_list.copy()  # 复制列表避免长时间持有锁
        
        # 为每个 WebSocket 创建发送任务
        for ws in ws_list_copy:
            asyncio.run_coroutine_threadsafe(
                self._safe_send(ws, {**data, "type": data_type}), 
                loop
            )
    
    async def _safe_send(self, ws, data):
        try:
            await ws.send_json(data)
        except Exception as e:
            print(f"WebSocket send error: {e}")
            self.remove(ws)
                
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 或指定你的前端地址
    allow_credentials=True,
    allow_methods=["*"],  # 允许所有请求方法
    allow_headers=["*"],  # 允许所有请求头
)

ws_manager = WSManager()

def get_manager():
    return ws_manager

static_dir = Path(__file__).parent.parent.joinpath("static")
@app.get("/")
async def index():    
    return FileResponse(static_dir.joinpath("index.html"))

app.mount("/static", StaticFiles(directory=static_dir), name="static")

@app.get("/video/{path}")
async def video_feed(path: str):
    builder = VideoBuilder.create("http", path)
    return builder.build()

@app.get("/video/{path:path}")
async def video_feed(path: str):
    builder = VideoBuilder.create("http", path)
    return builder.build()

@app.post("/offer/{path:path}")
async def webrtc(path: str, arg: dict):
    builder = VideoBuilder.create("webrtc", path)
    return await builder.build(**arg)

@app.post("/register")
async def register_route(request: Request):
    data = await request.json()
    path = data["path"]
    method = data["method"]
    topic = data["topic"]
    handler = RequestHandler(path, method, topic)
    
    # 动态添加路由
    app.add_api_route(
        path,
        handler.__call__,
        methods=[method],
    )
    app.openapi_schema = None
    app.setup()
    return SUCCESS_RESPONSE()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket, ws_manager: WSManager = Depends(get_manager)):
    await websocket.accept()
    await ws_manager.add(websocket)  # 等待添加完成
    while True:
        try:
            # 添加超时，避免永久阻塞
            data = await asyncio.wait_for(
                websocket.receive_text(), 
                timeout=1.0
            )
        except asyncio.TimeoutError:
            # 超时是正常的，继续循环
            pass
        except WebSocketDisconnect:
            ws_manager.remove(websocket)
            break
        except Exception:
            import traceback
            traceback.print_exc()
            ws_manager.remove(websocket)
            break

def mode_cb(data):
    global ws_manager
    if data.armed == False and ws_manager.data.get("arm", False) == True:
        ws_manager.publish({"event": "disarm"}, "event")
    ws_manager.publish({"mode": data.mode, "arm": data.armed, "connected": data.connected})
    
    
def gps_cb(data):
    global ws_manager
    ws_manager.publish({"gps_nsats": data.data})

def alt_cb(data):
    global ws_manager
    ws_manager.publish({"rel_alt": data.data})

def state_cb(data):
    global ws_manager
    if data.severity > StatusText.WARNING:
        return
    ws_manager.publish({"error": data.text}, "error")

def ws_cb(data):
    global ws_manager
    data = json.loads(data.data)
    data_type = data.get("type", "event")
    ws_manager.publish(data, data_type)

async def run_server():
    config = Config(app, host="0.0.0.0", port=8000)
    server = uvicorn.Server(config)
    global loop
    loop = asyncio.get_event_loop()
    await server.serve()


def register_cb(data):
    path = data.path
    method = data.method
    topic = data.topic
    handler = RequestHandler(path, method, topic)
    
    # 动态添加路由
    app.add_api_route(
        path,
        handler.__call__,
        methods=[method],
    )
    app.openapi_schema = None
    app.setup()
    res = SUCCESS_RESPONSE()
    return RegisterResponse(response=json.dumps(res))

if __name__ == "__main__":
    # 启动 ROS 节点
    rospy.init_node('connection')
    rospy.Subscriber("/mavros/state", State, mode_cb)
    rospy.Subscriber("/mavros/global_position/raw/satellites", UInt32, gps_cb)
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, alt_cb)
    rospy.Subscriber("/mavros/statustext/recv", StatusText, state_cb)
    rospy.Subscriber("ws", String, ws_cb)
    rospy.loginfo('wait for mavros service')
    rospy.wait_for_service('/mavros/set_stream_rate')
    set_rate = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
    rospy.loginfo("done")
    
    rospy.Service("register", Register, register_cb)
    start_pub = rospy.Publisher("do_register", Empty, queue_size=10)
    
    req = StreamRateRequest()
    req.stream_id = 0
    req.message_rate = 10
    req.on_off = True
    resp = set_rate(req)
    time.sleep(1)
    start_pub.publish(Empty())
    rospy.loginfo("publish")
    asyncio.run(run_server())