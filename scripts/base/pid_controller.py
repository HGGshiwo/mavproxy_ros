class PIDController:
    """
    通用 PID 控制器类
    参数说明：
    - kp: 比例系数 (Proportional)
    - ki: 积分系数 (Integral)
    - kd: 微分系数 (Derivative)
    - setpoint: 目标值（比如想要的温度、转速）
    - output_min/max: 输出值的上下限（防止输出过大/过小）
    """

    def __init__(
        self, kp, ki, kd, setpoint=0, output_min=-float("inf"), output_max=float("inf")
    ):
        # PID 核心参数
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # 目标值
        self.setpoint = setpoint
        # 输出限制（抗积分饱和）
        self.output_min = output_min
        self.output_max = output_max

        # 内部状态变量
        self.last_error = 0  # 上一次的误差
        self.integral = 0  # 积分累积值
        self.last_time = None  # 上一次计算的时间（用于计算时间差）

    def __call__(self, current_value, current_time, kp_rate=None):
        """
        计算 PID 输出值
        参数：
        - current_value: 系统当前的实际值（比如当前温度）
        - current_time: 当前时间戳（秒），用于计算微分和积分的时间差
        返回：
        - output: PID 输出值（比如控制加热器的功率、电机的电压）
        """
        # 1. 计算当前误差（目标值 - 实际值）
        error = current_value - self.setpoint 

        # 2. 计算时间差（dt），首次调用时 dt 设为 0，避免除以 0
        if self.last_time is None:
            dt = 0
        else:
            dt = current_time - self.last_time

        # 3. 比例项（P）：直接与误差成正比
        kp = self.kp
        if kp_rate is not None:
            kp = kp * kp_rate
        proportional = kp * error

        # 4. 积分项（I）：累积误差 × 时间，同时限制积分范围（抗积分饱和）
        integral = self.integral + error * dt
        integral = max(
            min(integral, self.output_max / self.ki if self.ki != 0 else integral),
            self.output_min / self.ki if self.ki != 0 else integral,
        )
        self.integral = integral
        integral_term = self.ki * integral

        # 5. 微分项（D）：误差变化率 × 微分系数（避免 dt=0 时出错）
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        derivative_term = self.kd * derivative

        # 6. 计算总输出，并限制输出范围
        output = proportional + integral_term + derivative_term
        # print(current_value, proportional, integral_term, derivative_term)
        output = max(min(output, self.output_max), self.output_min)

        # 7. 更新状态变量，为下一次计算做准备
        self.last_error = error
        self.last_time = current_time

        return output
