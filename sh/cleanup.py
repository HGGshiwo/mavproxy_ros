import subprocess
import time


def cleanup_sitl_processes():
    """独立的清理函数，确保在任何情况下都能被调用"""
    # 【重点1】在销毁阶段，绝对不要用 rospy.loginfo，改用原生 print
    print("[WARN] 触发 SITL 进程清理逻辑...")

    targets = [
        "sim_vehicle.py",
        "arducopter",
        "ardurover",
        "gz",
    ]
    for target in targets:
        try:
            subprocess.run(
                ["pkill", "-9", "-f", target],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            print(f"[ERROR] 清理 {target} 时发生错误: {e}")

    # 如果是在 Ctrl+C 销毁阶段，尽量缩短 sleep 时间，防止被 roslaunch 强杀
    time.sleep(1.0)
    print("[INFO] SITL 清理完毕。")


cleanup_sitl_processes()
