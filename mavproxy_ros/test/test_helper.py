import asyncio
import json
import math
import os
import signal
import subprocess
import threading
import time
import uuid
from contextlib import contextmanager
from queue import Queue
from typing import Any, Dict
from urllib import response

import psutil
import requests
import rospy
import tf
import tf.transformations
import websockets
from event_callback.components.ros import ROSComponent
from event_callback.core import BaseManager
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState

from mavproxy_ros.utils import wait_for_debugger

HOST = "127.0.0.1"
PORT = 8000


@contextmanager
def sitl_env():
    # 1. 核心魔法：生成一个独一无二的 UUID 作为本次启动的“狗牌”
    # 例如：'sitl_test_8f2a1b...'
    session_id = f"sitl_test_{uuid.uuid4().hex[:8]}"
    print(f"[INFO] 正在启动 SITL... 会话狗牌: {session_id}")

    # 2. 复制当前环境变量，并注入我们的狗牌
    custom_env = os.environ.copy()
    custom_env["MY_ROS_SITL_SESSION"] = session_id

    cmd = "sim_vehicle.py --no-rebuild --no-mavproxy -v ArduCopter -f gazebo-iris --custom-location=30.1119319,120.140883,0,0".split()

    # 3. 带着狗牌启动进程
    sitl_process = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=custom_env,  # <--- 将带有狗牌的环境变量传给 SITL
        preexec_fn=os.setsid,  # 依然创建独立进程组，双保险
    )

    print("[INFO] SITL 启动完毕。")

    # ==========================================
    # 定义“按牌杀人”的绝对清理逻辑
    # ==========================================
    def session_cleanup():
        print(f"\n[WARN] 触发按环境变量清理机制 (目标狗牌: {session_id})...")
        victims_killed = 0

        # 遍历全系统进程，重点检查它们的 environ（环境变量）
        # 捕获 AccessDenied 以免读取系统核心进程时报错
        for p in psutil.process_iter(["pid", "name", "environ"]):
            try:
                env = p.info.get("environ")
                # 如果这个进程的环境变量里，有我们刚才注入的狗牌
                if env and env.get("MY_ROS_SITL_SESSION") == session_id:
                    # 发现目标！直接击毙，管你是 xterm 还是 mavproxy
                    p.kill()
                    victims_killed += 1
                    print(
                        f"       -> 击毙绑定会话的进程: {p.info['name']} (PID: {p.info['pid']})"
                    )
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        # 容错：顺手把整个进程组也发一个 SIGKILL，作为最后一道保险
        try:
            os.killpg(os.getpgid(sitl_process.pid), 9)
        except OSError:
            pass

        time.sleep(2.0)
        print(f"[INFO] SITL 彻底清理完毕。共击毙 {victims_killed} 个携带狗牌的进程。")

    # 4. 注册 ROS 关闭钩子
    rospy.on_shutdown(session_cleanup)
    try:
        yield
    finally:
        if not rospy.is_shutdown():
            session_cleanup()


class TestHelper(BaseManager):
    def __init__(self):
        super().__init__(ROSComponent())
        self.stop = threading.Event()
        self.state = {}
        self.lock = threading.Lock()
        self.ws_open = threading.Event()
        self.ws_event_queue = Queue()
        threading.Thread(
            target=asyncio.run, args=(self.connect_websocket(),), daemon=True
        ).start()
        rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
        self.get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState
        )
        rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
        self.set_state_service = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState
        )

    async def connect_websocket(self):
        # 替换为你需要连接的WebSocket地址
        uri = f"ws://{HOST}:{PORT}/ws"
        msg = None
        while not self.stop.is_set():
            try:
                # 建立连接
                async with websockets.connect(uri) as websocket:
                    rospy.loginfo("成功连接到服务器！")
                    self.ws_open.set()
                    while True:
                        msg = await websocket.recv()
                        data = json.loads(msg)
                        if data["type"] == "state":
                            with self.lock:
                                self.state.update(data)
                        elif data["type"] == "event":
                            self.ws_event_queue.put(data)

            except websockets.exceptions.ConnectionClosed as e:
                print(f"连接已关闭: {e}")
            except Exception as e:
                print(f"发生错误: {e}")
                print(msg)
            time.sleep(1)
            self.ws_open.clear()

    def http_post(self, url, data=None) -> Dict[str, Any]:
        response = requests.post(f"http://{HOST}:{PORT}{url}", json=data)
        return response.json()

    def http_get(self, url):
        response = requests.get(f"http://{HOST}:{PORT}{url}")
        return response.json()

    def get_state(self, model_name: str):
        """获取模型状态，返回: (x, y, z), (vel_x, vel_y, vel_z), (roll, pitch, yaw)"""
        response = self.get_model_state(model_name, "world")

        if not response.success:
            rospy.logerr("获取模型状态失败！可能是模型名称写错了。")
            return False
        # --- 获取位置坐标 ---
        pos_x = response.pose.position.x
        pos_y = response.pose.position.y
        pos_z = response.pose.position.z  # 高度！判断降落的关键
        # --- 获取速度 (Twist) ---
        vel_x = response.twist.linear.x
        vel_y = response.twist.linear.y
        vel_z = response.twist.linear.z  # Z轴速度！判断是否停稳的关键
        # --- 获取姿态并转换为欧拉角 (Roll, Pitch, Yaw) ---
        orientation_q = response.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return (pos_x, pos_y, pos_z), (vel_x, vel_y, vel_z), (roll, pitch, yaw)

    def set_state(
        self,
        model_name="iris",
        x: float = 0,
        y: float = 0,
        z: float = 0,
        roll: float = 0,
        pitch: float = 0,
        yaw: float = 0,
    ):
        # 等待一小会儿让飞控状态机稳定
        rospy.sleep(1.0)
        # ================= 步骤 2：重置位置到原点 (物理层面) =================
        rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
        try:
            # 构造期望的状态
            state_msg = ModelState()
            state_msg.model_name = (
                model_name  # 你的无人机在Gazebo里的名字，通常是 iris, px4 等
            )
            state_msg.reference_frame = "world"

            # 2.1 重置位姿到原点
            state_msg.pose.position.x = x
            state_msg.pose.position.y = y
            state_msg.pose.position.z = z  # 或者地面的高度，如 0.1

            orientation = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            state_msg.pose.orientation.x = orientation[0]
            state_msg.pose.orientation.y = orientation[1]
            state_msg.pose.orientation.z = orientation[2]
            state_msg.pose.orientation.w = orientation[3]

            # 2.2 【关键】清空所有的线速度和角速度残余！
            state_msg.twist.linear.x = 0.0
            state_msg.twist.linear.y = 0.0
            state_msg.twist.linear.z = 0.0
            state_msg.twist.angular.x = 0.0
            state_msg.twist.angular.y = 0.0
            state_msg.twist.angular.z = 0.0

            # 发送瞬移请求
            resp = self.set_state_service(state_msg)
            if resp.success:
                rospy.loginfo("无人机物理位置重置成功。")
            else:
                rospy.logerr("位置重置失败!")

        except rospy.ServiceException as e:
            rospy.logerr("SetModelState 服务调用失败: %s" % e)
        rospy.sleep(1.0)  # 再次缓冲

    def wait_for_state(self, name: str, value: Any, timeout=10):
        cur_value = None
        for i in range(int(timeout / 0.1)):
            with self.lock:
                cur_value = self.state.get(name, None)
                if cur_value == value:
                    break
            time.sleep(0.1)
        else:
            raise RuntimeError(f"Wait for {name} == {value} timeout!(cur: {cur_value})")

    def init(self):
        """初始化状态"""
        # wait_for_debugger()
        self.ws_open.wait(timeout=20)
        rospy.loginfo("disarm vehicle!")
        for i in range(1000):
            time.sleep(2)  # 等待control注册成功
            res = self.http_post("/disarm")
            if res.get("status", None) == "success":
                break
        else:
            raise RuntimeError("等待control服务超时")
        self.wait_for_state("arm", False, 1000)
        rospy.loginfo("等待飞控状态稳定")

        rospy.loginfo("wait for 地面状态")
        self.wait_for_state("state", "地面状态", 1000)

    def takeoff(self):
        rospy.loginfo("起飞状态检查")
        for j in range(10):
            for i in range(100):
                time.sleep(3)
                res = self.http_get("/prearms")
                if "msg" not in res:
                    raise RuntimeError(res)
                msg = res["msg"]

                rospy.logerr(123, res)
                if msg.get("arm") == False:
                    rospy.logerr(f"Error: {msg['reason']}")
                    continue

                rospy.loginfo(f"起飞检查通过，尝试第{j+1}/10次起飞")
                out = self.http_post("/takeoff", data=dict(alt=5))
                if out["status"] == "success":
                    rospy.loginfo("起飞成功！")
                    rospy.loginfo("wait for 悬停状态")
                    self.wait_for_state("state", "悬停状态", 1000)
                    return
                else:
                    rospy.loginfo(f"起飞失败，{out['msg']}")

        raise RuntimeError("Prearm timeout!")

    def print_report(self, data: Dict[str, Any]):
        # 打印完美的性能报告
        rospy.loginfo("============ 测试通过 ==============")
        for k, v in data.items():
            if isinstance(v, float):
                v = round(v, 2)
            rospy.loginfo(f"{k}: {v}")
        rospy.loginfo("====================================")
