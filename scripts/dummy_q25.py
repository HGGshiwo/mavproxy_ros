#!/usr/bin/python3
# -*- coding: utf-8 -*-

import concurrent
import concurrent.futures
import math
import random
import socket
import threading
import time
from typing import Any, Optional, Tuple

# ROS imports
ros_loaded = False
try:
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32, String

    ros_loaded = True
except ImportError:
    rospy.loginfo("WARNING: ROS not available, running in UDP only mode")

from mavproxy_ros.controller.dog_utils import *

MAX_VEL_YAW = 1
MAX_VEL_X = 1
MAX_VEL_Y = 1
MAX_ACCEL = 1.0
MAX_ANGULAR_ACCEL = 0.2


# -------------------------- 机器人状态管理器 --------------------------
class Q25RobotState:
    """Q25机器人状态管理器，维护所有运行状态"""

    def __init__(self):
        # 基础状态
        self.robot_name = "天狼Q25"
        self.start_time = time.time()
        self.current_milege = 0  # 本次里程（cm）
        self.total_milege = 12500  # 累计里程（cm）
        self.is_nav_mode = 0  # 0=手动，1=导航
        self.in_l_mode = False  # 是否在L模式
        self.emergency_stop = False  # 是否软急停

        # 运动状态
        self.platform_height = 2  # 0=匍匐，2=正常
        self.gait_state = 0  # 0=行走，1=跑步
        self.speed_gear = 0  # 0=低速，1=高速

        # 速度信息
        self.vel_x = 0.0  # X轴速度（m/s）
        self.vel_y = 0.0  # Y轴速度（m/s）
        self.vel_yaw = 0.0  # Yaw角速度（rad/s）
        self.max_forward_vel = 1.2  # 最大前进速度（m/s）
        self.max_backward_vel = 0.8  # 最大后退速度（m/s）

        # 位姿信息
        self.pos_x = 0.0  # 世界坐标系X（m）
        self.pos_y = 0.0  # 世界坐标系Y（m）
        self.pos_yaw = 0.0  # 世界坐标系Yaw（rad）
        self.aim_pose = FireAim(0, 0, 0)  # 对准姿态

        # 设备状态
        self.camera_state = 0  # 0=关闭所有，1=主视开启，2=环视开启，3=全部开启
        self.charge_state = 0x0000  # 充电状态（默认空闲）

        # 硬件状态
        self.battery_level = 85  # 电池电量（%）
        self.is_charging = False  # 是否充电中
        self.motor_temps = [35.0 + random.uniform(0, 5) for _ in range(12)]  # 电机温度
        self.driver_temps = [
            32.0 + random.uniform(0, 4) for _ in range(12)
        ]  # 驱动器温度
        self.cpu_temp = 45.0 + random.uniform(0, 3)  # CPU温度
        self.cpu_freq = 1800.0 + random.uniform(0, 200)  # CPU主频（MHz）

        # 错误状态
        self.errors = {
            "imu_error": False,
            "wifi_error": False,
            "driver_heat_warn": False,
            "driver_error": False,
            "motor_heat_warn": False,
            "battery_low_warn": False,
        }

        # 心跳状态
        self.heartbeat_timeout = 0.6  # 心跳超时时间（600ms）
        self.last_heartbeat_time = time.time()
        self.connection_alive = True

        # 锁（线程安全）
        self.lock = threading.Lock()

        # 加速度限制
        self.target_vel_x = 0.0
        self.target_vel_y = 0.0
        self.target_vel_yaw = 0.0
        self.last_update_time = time.time()

    def update_heartbeat(self):
        """更新心跳时间"""
        self.last_heartbeat_time = time.time()
        self.connection_alive = True

    def handle_command(
        self,
        cmd_type: CommandType,
        param: int,
        data: Any,
        current_state: str = "passive",
    ) -> Tuple[bool, Optional[str]]:
        """处理控制指令，返回(执行结果, 指令信息用于转发)"""
        with self.lock:
            if not self.connection_alive:
                rospy.loginfo("⚠️  连接已断开，拒绝执行指令")
                return False, None

            try:
                if cmd_type == CommandType.MANUAL_HEARTBEAT:
                    self.update_heartbeat()
                    rospy.loginfo("❤️  收到心跳指令，维持连接")
                    return True, None

                elif cmd_type == CommandType.EMERGENCY_STOP:
                    rospy.loginfo("🛑  收到软急停指令")
                    return True, None

                elif cmd_type == CommandType.TOGGLE_STAND_DOWN:
                    if self.emergency_stop:
                        self.emergency_stop = False
                        rospy.loginfo("✅  解除软急停，恢复站立状态")
                        return True, "fixedstand"
                    else:
                        if current_state in ["passive"]:
                            new_state = "freestand"
                        else:
                            new_state = "passive"
                        rospy.loginfo(
                            f"🔄  切换姿态：{'趴下' if new_state == 'passive' else '站立'}"
                        )
                        return True, new_state

                elif cmd_type == CommandType.MOTION_MODE_MANUAL:
                    self.is_nav_mode = 0
                    rospy.loginfo("📱  切换到手动模式")
                    return True, None

                elif cmd_type == CommandType.MOTION_MODE_NAVIGATION:
                    self.is_nav_mode = 1
                    rospy.loginfo("🗺️  切换到导航模式")
                    return True, None

                elif cmd_type == CommandType.ENTER_L_MODE:
                    self.in_l_mode = True
                    rospy.loginfo("🧠  进入L模式（强化学习模式）")
                    return True, None

                elif cmd_type == CommandType.EXIT_L_MODE:
                    self.in_l_mode = False
                    rospy.loginfo("🧠  退出L模式")
                    return True, None

                elif cmd_type == CommandType.SET_PLATFORM_HEIGHT:
                    if self.gait_state == 1 and param == 0:
                        rospy.loginfo("❌  跑步步态下禁止切换为匍匐高度")
                        return False, None
                    self.platform_height = param
                    rospy.loginfo(
                        f"📏  切换平台高度：{'匍匐' if param == 0 else '正常'}"
                    )
                    return True, None

                elif cmd_type == CommandType.GAIT_WALK:
                    self.gait_state = 0
                    self.max_forward_vel = 1.2
                    self.max_backward_vel = 0.8
                    rospy.loginfo("🚶  切换到行走步态")
                    return True, None

                elif cmd_type == CommandType.GAIT_RUN:
                    if self.platform_height == 0:
                        rospy.loginfo("❌  匍匐高度下仅支持行走步态")
                        return False, None
                    self.gait_state = 1
                    self.max_forward_vel = 2.5 if self.speed_gear == 1 else 1.8
                    self.max_backward_vel = 1.0
                    rospy.loginfo("🏃  切换到跑步步态")
                    return True, None

                elif cmd_type == CommandType.SWITCH_SPEED_GEAR:
                    if self.gait_state != 1:
                        rospy.loginfo("❌  仅跑步步态支持速度档位切换")
                        return False, None
                    self.speed_gear = param
                    self.max_forward_vel = 2.5 if param == 1 else 1.8
                    rospy.loginfo(
                        f"⚡  切换速度档位：{'高速' if param == 1 else '低速'}"
                    )
                    return True, None

                elif cmd_type == CommandType.CAMERA_CONTROL:
                    if param in [0, 1, 2, 3]:
                        self.camera_state = param
                        state_desc = {
                            0: "关闭所有相机",
                            1: "开启主视相机，关闭环视相机",
                            2: "开启环视相机，关闭主视相机",
                            3: "开启所有相机",
                        }
                        rospy.loginfo(f"📷  {state_desc[param]}")
                        return True, None
                    else:
                        rospy.loginfo("❌  相机指令参数无效")
                        return False, None

                elif cmd_type == CommandType.MOVE_X_AXIS:
                    if current_state not in ["fixedstand", "freestand", "trotting"]:
                        rospy.loginfo("❌  非站立状态，X轴指令无效")
                        return False, None
                    y = param
                    if -32767 <= y <= -6553:
                        self.vel_x = ((y + 6553) / 26215) * (-self.max_backward_vel)
                    elif 6553 <= y <= 32767:
                        self.vel_x = ((y - 6553) / 26215) * self.max_forward_vel
                    else:
                        self.vel_x = 0.0
                    rospy.loginfo(f"➡️  X轴速度：{self.vel_x:.2f} m/s")
                    return True, None

                elif cmd_type == CommandType.MOVE_Y_AXIS:
                    if current_state not in ["fixedstand", "freestand", "trotting"]:
                        rospy.loginfo("❌  非站立状态，Y轴指令无效")
                        return False, None
                    x = param
                    if -32767 <= x <= -24576:
                        self.vel_y = (
                            (-1) * ((x + 24576) / 8554) * self.max_forward_vel * 0.8
                        )
                    elif 24576 <= x <= 32767:
                        self.vel_y = (
                            (-1) * ((x - 24576) / 8554) * self.max_forward_vel * 0.8
                        )
                    else:
                        self.vel_y = 0.0
                    rospy.loginfo(f"⬅️  Y轴速度：{self.vel_y:.2f} m/s")
                    return True, None

                elif cmd_type == CommandType.MOVE_YAW_AXIS:
                    if current_state not in ["fixedstand", "freestand", "trotting"]:
                        rospy.loginfo("❌  非站立状态，Yaw轴指令无效")
                        return False, None
                    x = param
                    if abs(x) >= 28212:
                        self.vel_yaw = (-1) * (x / 32768) * 1.5
                    else:
                        self.vel_yaw = 0.0
                    rospy.loginfo(f"🔄  Yaw角速度：{self.vel_yaw:.2f} rad/s")
                    return True, None

                elif cmd_type == CommandType.SET_AIM_POSE and isinstance(data, FireAim):
                    self.aim_pose = data
                    rospy.loginfo(
                        f"🎯  设置对准姿态：roll={data.roll}, pitch={data.pitch}, yaw={data.yaw}"
                    )
                    return True, None

                elif cmd_type == CommandType.RESET_AIM_POSE:
                    self.aim_pose = FireAim(0, 0, 0)
                    rospy.loginfo("🔄  重置对准姿态")
                    return True, None
                elif cmd_type == CommandType.AXIS_COMMAND_NO_DEAD_ZONE:
                    # 计算目标速度
                    target_vx = data.left_y / 1e3 * MAX_VEL_X
                    target_vy = data.left_x / 1e3 * MAX_VEL_Y
                    target_yr = data.right_x / 1e3 * MAX_VEL_YAW

                    # 应用加速度限制
                    current_time = time.time()
                    dt = current_time - self.last_update_time
                    if dt <= 0:
                        dt = 0.05

                    dvx = (target_vx - self.target_vel_x) / dt
                    dvy = (target_vy - self.target_vel_y) / dt
                    dyr = (target_yr - self.target_vel_yaw) / dt

                    linear_accel = math.sqrt(dvx**2 + dvy**2)
                    if linear_accel > MAX_ACCEL:
                        scale = MAX_ACCEL / linear_accel
                        self.target_vel_x += dvx * scale * dt
                        self.target_vel_y += dvy * scale * dt
                    else:
                        self.target_vel_x = target_vx
                        self.target_vel_y = target_vy

                    if abs(dyr) > MAX_ANGULAR_ACCEL:
                        sign = 1 if dyr > 0 else -1
                        self.target_vel_yaw += sign * MAX_ANGULAR_ACCEL * dt
                    else:
                        self.target_vel_yaw = target_yr

                    self.vel_x = self.target_vel_x
                    self.vel_y = self.target_vel_y
                    self.vel_yaw = self.target_vel_yaw
                    self.last_update_time = current_time
                    return True, None
                else:
                    rospy.loginfo(f"❓  未知指令：{cmd_type.name}")
                    return False, None

            except Exception as e:
                rospy.loginfo(f"❌  执行指令失败：{str(e)}")
                return False, None


def pack_report_data(
    cmd_enum: CommandType, state: Q25RobotState, current_state_int: int = 0
) -> bytes:
    """打包状态上报数据"""
    current_time = time.time()
    run_time = to_uint32(current_time - state.start_time)

    try:
        if cmd_enum == CommandType.RUN_STATUS_REPORT:
            # 运行状态数据上报（200Hz）
            rcs_data = RcsData(
                robot_name=state.robot_name,
                current_milege=state.current_milege,
                total_milege=state.total_milege,
                current_run_time=run_time,
                total_run_time=run_time + 3600,  # 模拟累计运行时间
                current_motion_time=to_uint32(run_time * 0.7),
                total_motion_time=to_uint32((run_time + 3600) * 0.7),
                joystick_lx=0.0,
                joystick_ly=0.0,
                joystick_rx=0.0,
                joystick_ry=0.0,
                is_nav_mode=state.is_nav_mode,
                imu_error=state.errors["imu_error"],
                wifi_error=state.errors["wifi_error"],
                driver_heat_warn=state.errors["driver_heat_warn"],
                driver_error=state.errors["driver_error"],
                motor_heat_warn=state.errors["motor_heat_warn"],
                battery_low_warn=state.errors["battery_low_warn"],
            )
            head = CommandHead(
                command_id=cmd_enum.value,
                parameter_size=len(rcs_data.to_bytes()),
                command_type=1,
            )
            return head.to_bytes() + rcs_data.to_bytes()

        elif cmd_enum == CommandType.MOTION_STATE_REPORT:
            # 运动状态数据上报（200Hz）
            motion_data = MotionStateData(
                touch_state=0,
                basic_state=current_state_int,
                gait_state=state.gait_state,
                max_forward_vel=state.max_forward_vel,
                max_backward_vel=state.max_backward_vel,
                pos_x=state.pos_x,
                pos_y=state.pos_y,
                pos_yaw=state.pos_yaw,
                vel_x=state.vel_x,
                vel_y=state.vel_y,
                vel_yaw=state.vel_yaw,
                robot_distance=state.current_milege,
                auto_charge_state=state.charge_state,
                pos_ctrl_state=0,
            )
            head = CommandHead(
                command_id=cmd_enum.value,
                parameter_size=len(motion_data.to_bytes()),
                command_type=1,
            )
            return head.to_bytes() + motion_data.to_bytes()

        elif cmd_enum == CommandType.SENSOR_DATA_REPORT:
            # 运动控制传感器数据上报（200Hz）
            imu_data = ImuSensorData(
                timestamp=to_uint32(current_time * 1000),
                roll=random.uniform(-0.5, 0.5),
                pitch=random.uniform(-0.5, 0.5),
                yaw=state.pos_yaw * (180 / 3.14159),
                omega_x=random.uniform(-0.01, 0.01),
                omega_y=random.uniform(-0.01, 0.01),
                omega_z=state.vel_yaw,
                acc_x=state.vel_x * 0.1 + random.uniform(-0.05, 0.05),
                acc_y=state.vel_y * 0.1 + random.uniform(-0.05, 0.05),
                acc_z=9.8 + random.uniform(-0.1, 0.1),
            )

            # 关节数据（模拟正常姿态）
            joint_pos = LegJointData(
                fl_hipx=0.0 + random.uniform(-0.05, 0.05),
                fl_hipy=-0.3 + random.uniform(-0.05, 0.05),
                fl_knee=0.6 + random.uniform(-0.05, 0.05),
                fr_hipx=0.0 + random.uniform(-0.05, 0.05),
                fr_hipy=-0.3 + random.uniform(-0.05, 0.05),
                fr_knee=0.6 + random.uniform(-0.05, 0.05),
                hl_hipx=0.0 + random.uniform(-0.05, 0.05),
                hl_hipy=-0.3 + random.uniform(-0.05, 0.05),
                hl_knee=0.6 + random.uniform(-0.05, 0.05),
                hr_hipx=0.0 + random.uniform(-0.05, 0.05),
                hr_hipy=-0.3 + random.uniform(-0.05, 0.05),
                hr_knee=0.6 + random.uniform(-0.05, 0.05),
            )

            joint_vel = LegJointData(
                **{
                    k: v * 0.1 + random.uniform(-0.02, 0.02)
                    for k, v in vars(joint_pos).items()
                }
            )

            joint_tau = LegJointData(
                **{
                    k: abs(v) * 5 + random.uniform(-0.5, 0.5)
                    for k, v in vars(joint_vel).items()
                }
            )

            sensor_data = ControllerSensorData(
                imu_data=imu_data,
                joint_pos=joint_pos,
                joint_vel=joint_vel,
                joint_tau=joint_tau,
            )

            head = CommandHead(
                command_id=cmd_enum.value,
                parameter_size=len(sensor_data.to_bytes()),
                command_type=1,
            )
            return head.to_bytes() + sensor_data.to_bytes()

        elif cmd_enum == CommandType.CONTROLLER_SAFE_DATA_REPORT:
            # 运动控制系统数据上报（1Hz）
            safe_data = ControllerSafeData(
                motor_temperatures=state.motor_temps,
                driver_temperatures=[to_uint32(t) for t in state.driver_temps],
                cpu_info=CpuInfo(
                    temperature=state.cpu_temp,
                    frequency=state.cpu_freq,
                ),
            )
            head = CommandHead(
                command_id=cmd_enum.value,
                parameter_size=len(safe_data.to_bytes()),
                command_type=1,
            )
            return head.to_bytes() + safe_data.to_bytes()

        elif cmd_enum == CommandType.BATTERY_LEVEL_REPORT:
            # 电池电量上报（0.5Hz）
            return BatteryLevel(level=to_uint32(state.battery_level)).to_bytes()

        elif cmd_enum == CommandType.BATTERY_CHARGE_STATE_REPORT:
            # 电池充电状态上报（0.5Hz）
            return BatteryChargeState(
                level=to_uint32(state.battery_level),
                is_charging=state.is_charging,
            ).to_bytes()

        elif cmd_enum == CommandType.ERROR_CODE_REPORT:
            # 错误码上报（按需）
            for err_code, err_msg in ErrorCode._ERROR_MAP.items():
                if err_code == 0x00D00601 and state.errors["battery_low_warn"]:
                    error = ErrorCode(
                        error_level=1,
                        error_code=err_code,
                        error_msg=err_msg,
                    )
                    return error.to_bytes()
            return ErrorCode(0, 0, "").to_bytes()

        else:
            return b""

    except Exception as e:
        rospy.loginfo(f"❌  {cmd_enum.name} 打包上报数据失败：{str(e)}")
        return b""


# -------------------------- UDP服务器主逻辑 --------------------------
class Q25UDPServer:
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 43893,
        report_host: str = "0.0.0.0",
        report_port: int = 6000,
    ):
        self.host = host
        self.port = port
        self.report_host = report_host
        self.report_port = report_port

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_addr: Optional[Tuple[str, int]] = None  # 记录客户端地址
        self.robot_state = Q25RobotState()
        self.running = False
        self.report_threads = []

        # ROS publishers
        self.pub_robot_state = None
        self.pub_cmd_vel = None
        self._current_state = "passive"

        if ros_loaded:
            try:
                rospy.init_node("q25_dummy", anonymous=True)
                self.pub_robot_state = rospy.Publisher(
                    "/robot_state", String, queue_size=10
                )
                self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
                rospy.Subscriber("/current_state", String, self.current_state_callback)
                rospy.loginfo(
                    "ROS publishers initialized: /robot_state, /cmd_vel, /current_state"
                )
            except rospy.exceptions.ROSException as e:
                rospy.loginfo(f"WARNING: Failed to initialize ROS: {e}")

    def start_report_threads(self):
        """启动状态上报线程"""
        # 定义上报任务：（指令类型，上报频率Hz）
        report_tasks = [
            (CommandType.RUN_STATUS_REPORT, 200),
            (CommandType.MOTION_STATE_REPORT, 1),
            (CommandType.SENSOR_DATA_REPORT, 1),
            (CommandType.CONTROLLER_SAFE_DATA_REPORT, 1),
            (CommandType.BATTERY_LEVEL_REPORT, 0.5),
            (CommandType.BATTERY_CHARGE_STATE_REPORT, 0.5),
        ]

        for cmd_enum, freq in report_tasks:
            thread = threading.Thread(
                target=self.report_task,
                args=(cmd_enum, freq),
                daemon=True,
                name=f"report-{cmd_enum.name}",
            )
            thread.start()
            self.report_threads.append(thread)
            rospy.loginfo(f"📡  启动{cmd_enum.name}上报线程（{freq}Hz）")

        # 启动ROS发布线程（50Hz）
        if ros_loaded:
            ros_thread = threading.Thread(
                target=self.ros_publish_task, daemon=True, name="ros-publish"
            )
            ros_thread.start()
            self.report_threads.append(ros_thread)
            rospy.loginfo(f"📡  启动ROS消息发布线程")

    def report_task(self, cmd_enum: CommandType, freq: float):
        """状态上报任务"""
        interval = 1.0 / freq
        while self.running:
            try:
                current_state_int = self.get_current_state_int()
                report_data = pack_report_data(
                    cmd_enum, self.robot_state, current_state_int
                )
                if report_data:
                    self.socket.sendto(
                        report_data, (self.report_host, self.report_port)
                    )
            except Exception as e:
                rospy.loginfo(f"❌  上报{cmd_enum.name}失败：{str(e)}")
            time.sleep(interval)

    def current_state_callback(self, msg):
        """接收unitree_guide发布的当前状态，用于上报"""
        self._current_state = msg.data

    def get_current_state_int(self) -> int:
        """将当前状态字符串转换为int值"""
        state_map = {
            "passive": 0,
            "fixedstand": 2,
            "freestand": 3,
            "trotting": 4,
        }
        return state_map.get(self._current_state, 0)

    def ros_publish_task(self):
        """ROS消息发布任务（50Hz）- 只发布cmd_vel"""
        if not ros_loaded or self.pub_cmd_vel is None:
            return

        hz = 10
        interval = 1 / hz  # 10Hz
        rate = rospy.Rate(hz)

        while self.running:
            try:
                twist = Twist()
                current_state_int = self.get_current_state_int()
                if current_state_int in [2, 3, 4]:
                    twist.linear.x = self.robot_state.vel_x
                    twist.linear.y = self.robot_state.vel_y
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = self.robot_state.vel_yaw
                else:
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0
                self.pub_cmd_vel.publish(twist)
                # print(twist)
                rate.sleep()
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.loginfo(f"WARNING: ROS publish error: {e}")
                time.sleep(interval)

    def handle_client_data(self, data: bytes, addr: Tuple[str, int]):
        """处理客户端指令"""
        self.client_addr = addr  # 记录客户端地址（用于上报）

        # 解析指令
        head, data_obj = unpack_q25_udp_cmd(data)
        cmd_enum = CommandType(head.command_id)
        info = command_info_map.get(cmd_enum)

        # if not cmd_enum:
        #     # 发送失败响应
        #     response = struct.pack("<III", 0, 0, FAIL_CODE)
        #     self.socket.sendto(response, addr)
        #     rospy.loginfo(f"📤  发送失败响应：{response.hex()}")
        #     rospy.loginfo(f"{'='*80}\n")
        #     return
        # 执行指令
        filter = [0x21040001]
        if cmd_enum.value not in filter:

            # rospy.loginfo(f"\n{'='*80}")
            # rospy.loginfo(f"📥  收到客户端数据 | 客户端：{addr[0]}:{addr[1]} | 长度：{len(data)}字节")
            # rospy.loginfo(f"   原始数据(十六进制)：{data.hex()}")

            # rospy.loginfo(f"📋  解析结果：")
            # rospy.loginfo(f"   指令名称：{info[0]}")
            # rospy.loginfo(f"   指令码：0x{cmd_enum.value:08X}")
            # rospy.loginfo(f"   参数：{head.parameter_size}")
            # rospy.loginfo(f"   数据: {data_obj}")
            pass

        success, cmd_info = self.robot_state.handle_command(
            cmd_enum, head.parameter_size, data_obj, self._current_state
        )

        if success and ros_loaded and self.pub_robot_state is not None and cmd_info:
            self.pub_robot_state.publish(String(data=cmd_info))
            self._current_state = cmd_info

        # 生成响应
        # if cmd_enum in [CommandType.CHARGE_REQUEST, CommandType.CHARGE_QUERY_STATUS]:
        #     # 充电指令响应
        #     response = ChargeResponse(
        #         state=self.robot_state.charge_state, state_msg=""
        #     ).to_bytes()
        # else:
        #     # 普通指令响应
        #     response = struct.pack(
        #         "<III", SUCCESS_CODE if success else FAIL_CODE, cmd_enum.value, 0
        #     )

        # self.socket.sendto(response, addr)
        # resp_status = "成功" if success else "失败"
        # rospy.loginfo(f"📤  发送{resp_status}响应：{response.hex()}")
        # rospy.loginfo(f"{'='*80}\n")

    def start(self):
        """启动服务器"""
        try:
            self.socket.bind((self.host, self.port))
        except OSError as e:
            rospy.loginfo(
                f"❌ UDP服务器启动失败：端口{self.port}被占用/权限不足，错误：{str(e)}"
            )
            return

        self.running = True
        self.start_report_threads()

        rospy.loginfo(f"✅  天狼Q25模拟UDP服务器启动成功")
        rospy.loginfo(f"📌  监听地址：{self.host}:{self.port}")
        rospy.loginfo(f"📌  支持所有Q25控制指令和状态上报")
        rospy.loginfo(f"📌  按 Ctrl+C 停止服务器\n")

        try:
            with concurrent.futures.ThreadPoolExecutor() as excutor:
                # 主循环接收数据
                while self.running:
                    try:
                        data, addr = self.socket.recvfrom(BUFFER_SIZE)
                        # 异步处理数据（避免阻塞接收）
                        excutor.submit(self.handle_client_data, data, addr)
                    except socket.timeout:
                        continue
                    except Exception as e:
                        rospy.loginfo(f"❌  接收数据异常：{str(e)}")

        except KeyboardInterrupt:
            rospy.loginfo(f"\n🛑  接收到退出信号，服务器正在关闭...")
        finally:
            self.running = False
            self.socket.close()
            rospy.loginfo(f"✅  服务器已成功关闭，套接字资源已释放")


# -------------------------- 全局配置 --------------------------
SERVER_HOST = "0.0.0.0"  # 固定监听所有网卡
SERVER_PORT = 9112  # 文档默认UDP指令端口
BUFFER_SIZE = 4096  # 接收缓冲区大小（适配最大指令包）

REPORT_HOST = "localhost"  # 数据上报的host
REPORT_PORT = 9111  # 数据上报的端口

HEADER_LENGTH = 12  # 指令头长度（固定12字节）
SUCCESS_CODE = 0  # 成功响应码
FAIL_CODE = 1  # 失败响应码

# -------------------------- 启动服务器 --------------------------
if __name__ == "__main__":
    server = Q25UDPServer(
        host=SERVER_HOST,
        port=SERVER_PORT,
        report_host=REPORT_HOST,
        report_port=REPORT_PORT,
    )
    server.start()
