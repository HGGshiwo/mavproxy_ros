import struct
from enum import IntEnum, unique
from typing import Optional, Union, List, Dict, Any
from dataclasses import dataclass
import ctypes

from event_callback.utils import classproperty


def to_uint32(data):
    data = int(data)
    return ctypes.c_uint32(data).value


def to_int32(data):
    data = int(data)
    return ctypes.c_int32(data).value


# -------------------------- 核心枚举（控制指令+状态上报指令） --------------------------
@unique
class CommandType(IntEnum):
    """天狼Q25机器人指令枚举（控制指令+状态上报指令）"""

    # 控制指令（新增无死区轴指令）
    MANUAL_HEARTBEAT = 0x21040001
    ENTER_L_MODE = 0x21010220
    EXIT_L_MODE = 0x21010221
    TOGGLE_STAND_DOWN = 0x21010202
    SET_AIM_POSE = 0x010B06
    RESET_AIM_POSE = 59
    MOTION_MODE_MANUAL = 0x21010C02
    MOTION_MODE_NAVIGATION = 0x21010C04
    MOVE_X_AXIS = 0x21010130
    MOVE_Y_AXIS = 0x21010131
    MOVE_YAW_AXIS = 0x21010135
    AXIS_COMMAND_NO_DEAD_ZONE = 0x21010140  # 新增无死区轴指令
    SET_PLATFORM_HEIGHT = 0x21010406
    GAIT_WALK = 0x21010420
    GAIT_RUN = 0x21010423
    SWITCH_SPEED_GEAR = 0x31010F11
    EMERGENCY_STOP = 0x21010C0E
    CAMERA_CONTROL = 0x21010F12
    UPPER_POWER_SWITCH = 0x80110801  # 上装上下电开关
    DRIVER_JOINT_DATA_SWITCH = 0x80110201  # 允许发布驱动器及关节采样数据开关
    STATE_DATA_SWITCH = 0x80110202  # 允许发布状态数据开关

    # 状态上报指令
    RUN_STATUS_REPORT = 0x1008
    MOTION_STATE_REPORT = 0x1009
    SENSOR_DATA_REPORT = 0x100A
    CONTROLLER_SAFE_DATA_REPORT = 0x100B
    BATTERY_LEVEL_REPORT = 0x11050F01
    BATTERY_CHARGE_STATE_REPORT = 0x11050F21
    ERROR_CODE_REPORT = 0x12050F01


# -------------------------- 指令属性映射表 --------------------------
# fmt: off
command_info_map: Dict[CommandType, tuple[str, int, str]] = {
    # 控制指令
    CommandType.MANUAL_HEARTBEAT: ("手动模式下心跳指令", 0, "/heartbeat/manual"),
    CommandType.ENTER_L_MODE: ("进入L模式指令", 0, "/l-mode/enter"),
    CommandType.EXIT_L_MODE: ("退出L模式指令", 0, "/l-mode/exit"),
    CommandType.TOGGLE_STAND_DOWN: ("站立/趴下切换指令", 0, "/pose/toggle-stand-down"),
    CommandType.SET_AIM_POSE: ("对准姿态指令", 1, "/pose/set-aim"),
    CommandType.RESET_AIM_POSE: ("重置对准指令", 0, "/pose/reset-aim"),
    CommandType.MOTION_MODE_MANUAL: ("手动模式指令", 0, "/motion-mode/manual"),
    CommandType.MOTION_MODE_NAVIGATION: ("导航模式指令", 0, "/motion-mode/navigation"),
    CommandType.MOVE_X_AXIS: ("左摇杆Y轴指令（X轴速度）", 0, "/move/x-axis"),
    CommandType.MOVE_Y_AXIS: ("左摇杆X轴指令（Y轴速度）", 0, "/move/y-axis"),
    CommandType.MOVE_YAW_AXIS: ("右摇杆X轴指令（Yaw角速度）", 0, "/move/yaw-axis"),
    CommandType.AXIS_COMMAND_NO_DEAD_ZONE: ("无死区轴指令", 1, "/move/axis-no-dead-zone"),
    CommandType.SET_PLATFORM_HEIGHT: ("平台高度切换指令", 0, "/platform-height/set"),
    CommandType.GAIT_WALK: ("行走步态指令", 0, "/gait/set/walk"),
    CommandType.GAIT_RUN: ("跑步步态指令", 0, "/gait/set/run"),
    CommandType.SWITCH_SPEED_GEAR: ("速度切换指令", 0, "/speed-gear/switch"),
    CommandType.EMERGENCY_STOP: ("软急停指令", 0, "/emergency_stop"),
    CommandType.CAMERA_CONTROL: ("相机指令", 0, "/camera/control"),
    # 状态上报指令
    CommandType.RUN_STATUS_REPORT: ("运行状态数据上报", 1, "/report/run-status"),
    CommandType.MOTION_STATE_REPORT: ("运动状态数据上报", 1, "/report/motion-state"),
    CommandType.SENSOR_DATA_REPORT: ("运动控制传感器数据上报", 1, "/report/sensor-data"),
    CommandType.CONTROLLER_SAFE_DATA_REPORT: ("运动控制系统数据上报", 1, "/report/controller-safe-data"),
    CommandType.BATTERY_LEVEL_REPORT: ("电池电量状态上报", 0, "/report/battery/level"),
    CommandType.BATTERY_CHARGE_STATE_REPORT: ("电池充电状态上报", 0, "/report/battery/charge-state"),
    CommandType.ERROR_CODE_REPORT: ("错误码上报", 0, "/report/error-code"),
    CommandType.UPPER_POWER_SWITCH: ("上装上下电开关指令", 0, "/advanced/upper-power"),
    CommandType.DRIVER_JOINT_DATA_SWITCH: ("驱动器及关节采样数据开关指令", 0, "/advanced/data-switch/driver-joint"),
    CommandType.STATE_DATA_SWITCH: ("状态数据开关指令", 0, "/advanced/data-switch/state"),
}
# fmt: on


# -------------------------- 数据结构类 --------------------------
@dataclass
class BaseCommand:
    @classproperty
    def fmt_size(cls) -> int:
        return struct.calcsize(cls.fmt)


@dataclass
class CommandHead:
    """指令头部（12字节）"""

    fmt = "<III"  # 公共打包格式
    command_id: int  # uint32_t
    parameter_size: int  # uint32_t
    command_type: int  # uint32_t（0=基本指令，1=扩展指令）

    def to_bytes(self) -> bytes:
        return struct.pack(
            self.fmt, self.command_id, self.parameter_size, self.command_type
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "CommandHead":
        if len(data) < struct.calcsize(cls.fmt):
            raise ValueError(f"至少需要{struct.calcsize(cls.fmt)}字节")
        cmd_id, param_size, cmd_type = struct.unpack(
            cls.fmt, data[: struct.calcsize(cls.fmt)]
        )
        return cls(command_id=cmd_id, parameter_size=param_size, command_type=cmd_type)


@dataclass
class FireAim:
    """对准指令数据（12字节）"""

    fmt = "<III"  # 公共打包格式
    roll: int  # uint32_t（无效字段）
    pitch: int  # uint32_t（[-10°,17°]对应[1000,1000]）
    yaw: int  # uint32_t（[-20°,20°]对应[1000,1000]）

    def to_bytes(self) -> bytes:
        return struct.pack(self.fmt, self.roll, self.pitch, self.yaw)

    @classmethod
    def from_bytes(cls, data: bytes) -> "FireAim":
        if len(data) < struct.calcsize(cls.fmt):
            raise ValueError(f"至少需要{struct.calcsize(cls.fmt)}字节")
        roll, pitch, yaw = struct.unpack(cls.fmt, data[: struct.calcsize(cls.fmt)])
        return cls(roll=roll, pitch=pitch, yaw=yaw)


@dataclass
class AxisCommand:
    """无死区轴指令数据（16字节，新增）"""

    fmt = "<iiii"  # 公共打包格式
    left_x: int  # uint32_t（X轴速度，[-1000,1000]）
    left_y: int  # uint32_t（Y轴速度，[-1000,1000]）
    right_x: int  # uint32_t（Yaw角速度，[-1000,1000]）
    right_y: int  # uint32_t（预留，建议为0）

    def to_bytes(self) -> bytes:
        return struct.pack(
            self.fmt, self.left_x, self.left_y, self.right_x, self.right_y
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "AxisCommand":
        if len(data) < struct.calcsize(cls.fmt):
            raise ValueError(f"至少需要{struct.calcsize(cls.fmt)}字节")
        left_x, left_y, right_x, right_y = struct.unpack(
            cls.fmt, data[: struct.calcsize(cls.fmt)]
        )
        return cls(left_x=left_x, left_y=left_y, right_x=right_x, right_y=right_y)


@dataclass
class ImuSensorData(BaseCommand):
    """IMU传感器数据（40字节）"""

    fmt = "<i9f"  # 公共打包格式
    timestamp: int  # int32_t
    roll: float  # 翻滚角（°）
    pitch: float  # 俯仰角（°）
    yaw: float  # 偏航角（°）
    omega_x: float  # X轴角速度（rad/s）
    omega_y: float  # Y轴角速度（rad/s）
    omega_z: float  # Z轴角速度（rad/s）
    acc_x: float  # X轴加速度（m/s²）
    acc_y: float  # Y轴加速度（m/s²）
    acc_z: float  # Z轴加速度（m/s²）

    def to_bytes(self) -> bytes:
        return struct.pack(
            self.fmt,
            to_int32(self.timestamp),
            self.roll,
            self.pitch,
            self.yaw,
            self.omega_x,
            self.omega_y,
            self.omega_z,
            self.acc_x,
            self.acc_y,
            self.acc_z,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "ImuSensorData":
        if len(data) < struct.calcsize(cls.fmt):
            raise ValueError(f"至少需要{struct.calcsize(cls.fmt)}字节")
        unpacked = struct.unpack(cls.fmt, data[: struct.calcsize(cls.fmt)])
        return cls(
            timestamp=unpacked[0],
            roll=unpacked[1],
            pitch=unpacked[2],
            yaw=unpacked[3],
            omega_x=unpacked[4],
            omega_y=unpacked[5],
            omega_z=unpacked[6],
            acc_x=unpacked[7],
            acc_y=unpacked[8],
            acc_z=unpacked[9],
        )


@dataclass
class LegJointData(BaseCommand):
    """关节数据（48字节）"""

    fmt = "<12f"  # 公共打包格式
    # 左前腿
    fl_hipx: float  # 侧摆关节
    fl_hipy: float  # 髋关节
    fl_knee: float  # 膝关节
    # 右前腿
    fr_hipx: float  # 侧摆关节
    fr_hipy: float  # 髋关节
    fr_knee: float  # 膝关节
    # 左后腿
    hl_hipx: float  # 侧摆关节
    hl_hipy: float  # 髋关节
    hl_knee: float  # 膝关节
    # 右后腿
    hr_hipx: float  # 侧摆关节
    hr_hipy: float  # 髋关节
    hr_knee: float  # 膝关节

    def to_bytes(self) -> bytes:
        data_list = [
            self.fl_hipx,
            self.fl_hipy,
            self.fl_knee,
            self.fr_hipx,
            self.fr_hipy,
            self.fr_knee,
            self.hl_hipx,
            self.hl_hipy,
            self.hl_knee,
            self.hr_hipx,
            self.hr_hipy,
            self.hr_knee,
        ]
        return struct.pack(self.fmt, *data_list)

    @classmethod
    def from_bytes(cls, data: bytes) -> "LegJointData":
        if len(data) < struct.calcsize(cls.fmt):
            raise ValueError(f"至少需要{struct.calcsize(cls.fmt)}字节")
        unpacked = struct.unpack(cls.fmt, data[: struct.calcsize(cls.fmt)])
        return cls(
            fl_hipx=unpacked[0],
            fl_hipy=unpacked[1],
            fl_knee=unpacked[2],
            fr_hipx=unpacked[3],
            fr_hipy=unpacked[4],
            fr_knee=unpacked[5],
            hl_hipx=unpacked[6],
            hl_hipy=unpacked[7],
            hl_knee=unpacked[8],
            hr_hipx=unpacked[9],
            hr_hipy=unpacked[10],
            hr_knee=unpacked[11],
        )


@dataclass
class CpuInfo:
    """CPU状态数据（8字节）"""

    fmt = "<ff"  # 公共打包格式
    temperature: float  # 温度（℃）
    frequency: float  # 主频（MHz）

    def to_bytes(self) -> bytes:
        return struct.pack(self.fmt, self.temperature, self.frequency)

    @classmethod
    def from_bytes(cls, data: bytes) -> "CpuInfo":
        if len(data) < struct.calcsize(cls.fmt):
            raise ValueError(f"至少需要{struct.calcsize(cls.fmt)}字节")
        temp, freq = struct.unpack(cls.fmt, data[: struct.calcsize(cls.fmt)])
        return cls(temperature=temp, frequency=freq)


# -------------------------- 状态上报核心数据类 --------------------------
@dataclass
class RcsData:
    """运行状态数据（76字节）"""

    fmt = "<15siiqqqqffff10BI"  # 公共打包格式
    robot_name: str  # char[15]
    current_milege: int  # 本次运行里程（cm，int32_t）
    total_milege: int  # 累计运行里程（cm，int32_t）
    current_run_time: int  # 本次运行时间（s，long）
    total_run_time: int  # 累计运行时间（s，long）
    current_motion_time: int  # 本次运动时间（s，long）
    total_motion_time: int  # 累计运动时间（s，long）
    joystick_lx: float  # 左摇杆X轴（[-1.0,1.0]）
    joystick_ly: float  # 左摇杆Y轴（[-1.0,1.0]）
    joystick_rx: float  # 右摇杆X轴（[-1.0,1.0]）
    joystick_ry: float  # 右摇杆Y轴（[-1.0,1.0]）
    is_nav_mode: int  # 0=手动，1=导航（uint8_t）
    # 错误状态位
    imu_error: bool
    wifi_error: bool
    driver_heat_warn: bool
    driver_error: bool
    motor_heat_warn: bool
    battery_low_warn: bool

    def to_bytes(self) -> bytes:
        robot_name_bytes = self.robot_name.encode("utf-8")[:15].ljust(15, b"\x00")
        error_bits = 0
        error_bits |= (1 << 31) if self.imu_error else 0
        error_bits |= (1 << 0) if self.wifi_error else 0
        error_bits |= (1 << 1) if self.driver_heat_warn else 0
        error_bits |= (1 << 2) if self.driver_error else 0
        error_bits |= (1 << 3) if self.motor_heat_warn else 0
        error_bits |= (1 << 4) if self.battery_low_warn else 0

        return struct.pack(
            self.fmt,
            robot_name_bytes,
            self.current_milege,
            self.total_milege,
            self.current_run_time,
            self.total_run_time,
            self.current_motion_time,
            self.total_motion_time,
            self.joystick_lx,
            self.joystick_ly,
            self.joystick_rx,
            self.joystick_ry,
            self.is_nav_mode,
            *[0] * 9,
            error_bits,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "RcsData":
        if len(data) < struct.calcsize(cls.fmt):
            raise ValueError(f"至少需要{struct.calcsize(cls.fmt)}字节")
        unpacked = struct.unpack(cls.fmt, data[: struct.calcsize(cls.fmt)])
        return cls(
            robot_name=unpacked[0].decode("utf-8").rstrip("\x00"),
            current_milege=unpacked[1],
            total_milege=unpacked[2],
            current_run_time=unpacked[3],
            total_run_time=unpacked[4],
            current_motion_time=unpacked[5],
            total_motion_time=unpacked[6],
            joystick_lx=unpacked[7],
            joystick_ly=unpacked[8],
            joystick_rx=unpacked[9],
            joystick_ry=unpacked[10],
            is_nav_mode=unpacked[11],
            imu_error=(unpacked[21] >> 31) & 1,
            wifi_error=(unpacked[21] >> 0) & 1,
            driver_heat_warn=(unpacked[21] >> 1) & 1,
            driver_error=(unpacked[21] >> 2) & 1,
            motor_heat_warn=(unpacked[21] >> 3) & 1,
            battery_low_warn=(unpacked[21] >> 4) & 1,
        )


@dataclass
class MotionStateData:
    """运动状态数据（56字节）"""

    fmt = "<2Bff3f3ffII10B"  # 公共打包格式
    # basic_state取值：0=趴下,1=起立中,2=初始站立,3=力控站立,4=踏步,5=趴下中,6=急停/摔倒,0x10=L模式
    basic_state: int
    " "
    # gait_state取值：0x20=行走,0x23=跑步
    gait_state: int
    max_forward_vel: float  # 最大前进速度（m/s）
    max_backward_vel: float  # 最大后退速度（m/s）
    pos_x: float  # 世界坐标系X（m）
    pos_y: float  # 世界坐标系Y（m）
    pos_yaw: float  # 世界坐标系Yaw（rad）
    vel_x: float  # 机身X轴速度（m/s）
    vel_y: float  # 机身Y轴速度（m/s）
    vel_yaw: float  # 机身Yaw角速度（rad/s）
    robot_distance: float  # 本次运行里程（cm）
    touch_state: int  # 预留（unsigned）
    auto_charge_state: int  # 充电状态（uint8_t）
    pos_ctrl_state: int  # 位置控制状态（uint8_t）

    def to_bytes(self) -> bytes:
        control_state = 0  # 预留字段
        task_reserved = [0] * 8
        return struct.pack(
            self.fmt,
            self.basic_state,
            self.gait_state,
            self.max_forward_vel,
            self.max_backward_vel,
            self.pos_x,
            self.pos_y,
            self.pos_yaw,
            self.vel_x,
            self.vel_y,
            self.vel_yaw,
            self.robot_distance,
            self.touch_state,
            control_state,
            self.auto_charge_state,
            self.pos_ctrl_state,
            *task_reserved,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "MotionStateData":
        if len(data) < struct.calcsize(cls.fmt):
            raise ValueError(f"至少需要{struct.calcsize(cls.fmt)}字节")
        unpacked = struct.unpack(cls.fmt, data[: struct.calcsize(cls.fmt)])
        return cls(
            basic_state=unpacked[0],
            gait_state=unpacked[1],
            max_forward_vel=unpacked[2],
            max_backward_vel=unpacked[3],
            pos_x=unpacked[4],
            pos_y=unpacked[5],
            pos_yaw=unpacked[6],
            vel_x=unpacked[7],
            vel_y=unpacked[8],
            vel_yaw=unpacked[9],
            robot_distance=unpacked[10],
            touch_state=unpacked[11],
            auto_charge_state=unpacked[13],
            pos_ctrl_state=unpacked[14],
        )


@dataclass
class ControllerSensorData:
    """运动控制传感器数据（184字节）"""

    imu_data: ImuSensorData
    joint_pos: LegJointData
    joint_vel: LegJointData
    joint_tau: LegJointData

    def to_bytes(self) -> bytes:
        return (
            self.imu_data.to_bytes()
            + self.joint_pos.to_bytes()
            + self.joint_vel.to_bytes()
            + self.joint_tau.to_bytes()
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "ControllerSensorData":
        total_size = ImuSensorData.fmt_size + LegJointData.fmt_size * 3
        if len(data) < total_size:
            raise ValueError(f"至少需要{total_size}字节")
        imu = ImuSensorData.from_bytes(data[: ImuSensorData.fmt_size])
        joint_pos = LegJointData.from_bytes(
            data[
                ImuSensorData.fmt_size : ImuSensorData.fmt_size + LegJointData.fmt_size
            ]
        )
        joint_vel = LegJointData.from_bytes(
            data[
                ImuSensorData.fmt_size
                + LegJointData.fmt_size : ImuSensorData.fmt_size
                + LegJointData.fmt_size * 2
            ]
        )
        joint_tau = LegJointData.from_bytes(
            data[ImuSensorData.fmt_size + LegJointData.fmt_size * 2 : total_size]
        )
        return cls(
            imu_data=imu, joint_pos=joint_pos, joint_vel=joint_vel, joint_tau=joint_tau
        )

    @property
    def fmt_size(self) -> int:
        return (
            struct.calcsize(ImuSensorData.fmt) + struct.calcsize(LegJointData.fmt) * 3
        )


@dataclass
class ControllerSafeData:
    """运动控制系统数据（68字节，原Q20ControllerSafeData重命名）"""

    fmt = "<12f12Bff"  # 公共打包格式
    motor_temperatures: List[float]  # 12个电机温度（℃）
    driver_temperatures: List[int]  # 12个驱动器温度（℃，uint8_t）
    cpu_info: CpuInfo  # CPU状态

    def to_bytes(self) -> bytes:
        if len(self.motor_temperatures) != 12 or len(self.driver_temperatures) != 12:
            raise ValueError("温度列表必须包含12个元素")
        return struct.pack(
            self.fmt,
            *self.motor_temperatures,
            *self.driver_temperatures,
            self.cpu_info.temperature,
            self.cpu_info.frequency,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "ControllerSafeData":
        if len(data) < struct.calcsize(cls.fmt):
            raise ValueError(f"至少需要{struct.calcsize(cls.fmt)}字节")
        unpacked = struct.unpack(cls.fmt, data[: struct.calcsize(cls.fmt)])
        return cls(
            motor_temperatures=list(unpacked[:12]),
            driver_temperatures=list(unpacked[12:24]),
            cpu_info=CpuInfo(temperature=unpacked[24], frequency=unpacked[25]),
        )


@dataclass
class BatteryLevel:
    """电池电量状态（12字节）"""

    level: int  # 电量（%）

    def to_bytes(self) -> bytes:
        head = CommandHead(
            command_id=CommandType.BATTERY_LEVEL_REPORT,
            parameter_size=self.level,
            command_type=0,
        )
        return head.to_bytes()

    @classmethod
    def from_bytes(cls, head: CommandHead) -> "BatteryLevel":
        return cls(level=head.parameter_size)


@dataclass
class BatteryChargeState:
    """电池充电状态（12字节）"""

    level: int  # 电量（%，高24位）
    is_charging: bool  # 低8位：0=未充电，1=充电中

    def to_bytes(self) -> bytes:
        param = (self.level << 8) | (1 if self.is_charging else 0)
        head = CommandHead(
            command_id=CommandType.BATTERY_CHARGE_STATE_REPORT,
            parameter_size=param,
            command_type=0,
        )
        return head.to_bytes()

    @classmethod
    def from_bytes(cls, head: CommandHead) -> "BatteryChargeState":
        level = (head.parameter_size >> 8) & 0xFFFFFF
        is_charging = (head.parameter_size & 0xFF) == 1
        return cls(level=level, is_charging=is_charging)


@dataclass
class ErrorCode:
    """错误码（12字节）"""

    error_level: int  # 0=通知，1=警告，2=错误（低8位）
    error_code: int  # 具体错误码（高24位）
    error_msg: str  # 错误描述

    _ERROR_MAP = {
        0x00D00101: "IMU故障",
        0x00D00201: "地形图形传输超时",
        0x00D00402: "驱动器开机自检异常",
        0x00D00502: "驱动器运行异常",
        0x00D00601: "低电量保护触发",
        0x00D00701: "关节电机过温保护触发",
        0x00D00802: "电池过温保护触发",
        0x00D00901: "核心板CPU占用率过高(≥98%)",
        0x00D00A01: "核心板CPU温度过高(≥70℃)",
        0x11050F10: "硬急停触发",
    }

    def to_bytes(self) -> bytes:
        param = (self.error_code << 8) | self.error_level
        head = CommandHead(
            command_id=CommandType.ERROR_CODE_REPORT,
            parameter_size=param,
            command_type=0,
        )
        return head.to_bytes()

    @classmethod
    def from_bytes(cls, head: CommandHead) -> "ErrorCode":
        error_level = head.parameter_size & 0xFF
        error_code = (head.parameter_size >> 8) & 0xFFFFFF
        error_msg = cls._ERROR_MAP.get(error_code, f"未知错误（0x{error_code:06X}）")
        return cls(error_level=error_level, error_code=error_code, error_msg=error_msg)


# -------------------------- 核心映射：data类型→command_type --------------------------
_DATA_TYPE_TO_COMMAND = {
    FireAim: CommandType.SET_AIM_POSE,
    AxisCommand: CommandType.AXIS_COMMAND_NO_DEAD_ZONE,  # 新增映射
}


# -------------------------- 打包/解包函数 --------------------------
def pack_q25_udp_cmd(
    command_type: Optional[CommandType] = None,
    parameter_size: int = 0,
    data: Optional[Union[List[int], bytes, FireAim, AxisCommand]] = None,
) -> bytes:
    # 自动推导command_type
    if command_type is None:
        if data is None:
            raise ValueError("基本指令必须指定command_type")
        data_type = type(data)
        if data_type not in _DATA_TYPE_TO_COMMAND:
            raise ValueError(
                f"仅支持FireAim/AxisCommand类型自动匹配，当前为{data_type}"
            )
        command_type = _DATA_TYPE_TO_COMMAND[data_type]

    if command_type not in command_info_map:
        raise ValueError(f"未定义指令：{command_type}")
    cmd_name, cmd_type, url = command_info_map[command_type]
    command_id = int(command_type)

    # 校验指令类型与data一致性
    if cmd_type == 0 and data is not None:
        raise ValueError(f"基本指令[{cmd_name}]不支持携带data")
    if cmd_type == 1 and data is None:
        raise ValueError(f"扩展指令[{cmd_name}]必须传入data")

    # 编码数据
    if cmd_type == 0:
        return struct.pack(
            "<III",
            to_uint32(command_id),
            to_uint32(parameter_size),
            to_uint32(cmd_type),
        )
    else:
        if isinstance(data, (FireAim, AxisCommand)):
            data_bytes = data.to_bytes()
        elif isinstance(data, list):
            for val in data:
                if not (0 <= val <= 0xFFFFFFFF):
                    raise ValueError("数据超出uint32_t范围")
            data_bytes = struct.pack(f"<{len(data)}I", *data)
        elif isinstance(data, bytes):
            data_bytes = data
        else:
            raise TypeError("data仅支持list/bytes/FireAim/AxisCommand")

        if len(data_bytes) != parameter_size:
            raise ValueError(
                f"数据长度（{len(data_bytes)}）与parameter_size（{parameter_size}）不匹配"
            )

        return struct.pack("<III", command_id, parameter_size, cmd_type) + data_bytes


def unpack_q25_udp_cmd(data: bytes) -> tuple[CommandHead, Optional[Any]]:
    # 解包头部
    if len(data) < 12:
        raise ValueError("至少需要12字节指令头部")
    head = CommandHead.from_bytes(data[:12])

    # 匹配指令类型
    try:
        command_type = CommandType(head.command_id)
    except ValueError:
        raise ValueError(f"未定义指令码：0x{head.command_id:08X}")

    cmd_name, cmd_type, url = command_info_map[command_type]
    data_body = data[12 : 12 + head.parameter_size]

    # 解码数据体
    if cmd_type == 0:
        if command_type == CommandType.BATTERY_LEVEL_REPORT:
            return head, BatteryLevel.from_bytes(head)
        elif command_type == CommandType.BATTERY_CHARGE_STATE_REPORT:
            return head, BatteryChargeState.from_bytes(head)
        elif command_type == CommandType.ERROR_CODE_REPORT:
            return head, ErrorCode.from_bytes(head)
        else:
            return head, None
    else:
        if len(data_body) != head.parameter_size:
            raise ValueError("数据体长度与parameter_size不匹配")

        if command_type == CommandType.RUN_STATUS_REPORT:
            return head, RcsData.from_bytes(data_body)
        elif command_type == CommandType.MOTION_STATE_REPORT:
            return head, MotionStateData.from_bytes(data_body)
        elif command_type == CommandType.SENSOR_DATA_REPORT:
            return head, ControllerSensorData.from_bytes(data_body)
        elif command_type == CommandType.CONTROLLER_SAFE_DATA_REPORT:
            return head, ControllerSafeData.from_bytes(data_body)
        elif command_type == CommandType.SET_AIM_POSE:
            return head, FireAim.from_bytes(data_body)
        elif command_type == CommandType.AXIS_COMMAND_NO_DEAD_ZONE:
            return head, AxisCommand.from_bytes(data_body)
        else:
            return head, data_body


def speed_to_cmd_value(
    axis_type: str,
    speed: float,
    v_max: float = None,
) -> int:
    if axis_type not in ["x", "y", "yaw"]:
        raise ValueError(f"仅支持'x'/'y'/'yaw'，当前输入：{axis_type}")

    # X轴速度 → 左摇杆Y轴指令值
    if axis_type == "x":
        if speed > 0:
            cmd_value = (speed / v_max) * 26215 - 6553
        elif speed < 0:
            cmd_value = (speed / v_max) * 26215 + 6553
        else:
            cmd_value = 0
    # Y轴速度 → 左摇杆X轴指令值
    elif axis_type == "y":
        if speed > 0:
            cmd_value = -((speed / v_max) * 8554 + 24576)
        elif speed < 0:
            cmd_value = -((speed / v_max) * 8554 - 24576)
        else:
            cmd_value = 0
    # Yaw角速度 → 右摇杆X轴指令值
    elif axis_type == "yaw":
        cmd_value = -speed * 32768

    cmd_value = round(cmd_value)
    return max(-32767, min(32767, cmd_value))
