from event_callback.components.http.ui_config import (
    CopyConfig,
    FormConfig,
    InnerButtonConfig,
    InputFormItemConfig,
    NumberFormItemConfig,
    PrimaryButtonConfig,
    StatusConfig,
    ToastConfig,
)

StatusConfig("connected", "连接状态")
StatusConfig("arm", "解锁")
StatusConfig("mode", "飞控模式")
StatusConfig("state", "单控模式")
StatusConfig("gps_nstats", "GPS星数")
StatusConfig("gps_fix_type", "GPS fix_type")
StatusConfig("rel_alt", "相对高度")
StatusConfig("yaw_diff", "偏航误差")
StatusConfig("alt_diff", "高度误差")
StatusConfig("dis", "目标距离")
StatusConfig("wp_idx", "当前航点")
StatusConfig("planner", "避障状态")
StatusConfig("version", "version")
StatusConfig("follow x", "跟随速度X")
StatusConfig("follow y", "跟随速度Y")
StatusConfig("follow z", "跟随速度Z")
StatusConfig("pland", "精准降落状态")

PrimaryButtonConfig(name="起飞检查", target=ToastConfig(url="/prearms", method="GET"))
PrimaryButtonConfig(
    name="起飞",
    target=FormConfig(
        title="起飞",
        items=dict(alt=NumberFormItemConfig(name="起飞高度")),
        submit=InnerButtonConfig(target=ToastConfig(url="/takeoff", method="POST")),
    ),
)
PrimaryButtonConfig(name="解锁", target=ToastConfig(url="/arm", method="POST"))
PrimaryButtonConfig(
    name="GUIDED",
    target=ToastConfig(url="/set_mode", method="POST", data=dict(mode="GUIDED")),
)
PrimaryButtonConfig(name="返航", target=ToastConfig(url="/return", method="POST"))
PrimaryButtonConfig(name="降落", target=ToastConfig(url="/land", method="POST"))
PrimaryButtonConfig(name="重启飞控", target=ToastConfig(url="/reboot_fcu", method="POST"))
PrimaryButtonConfig(
    name="输入航点",
    target=FormConfig(
        title="输入航点",
        items=dict(
            waypoint=InputFormItemConfig(name="输入航点", default=""),
            vel=NumberFormItemConfig(name="输入速度", default=5),
        ),
        submit=InnerButtonConfig(
            target=ToastConfig(url="/set_waypoint", method="POST")
        ),
    ),
)
PrimaryButtonConfig(name="获取GPS", target=CopyConfig(url="/get_gps", method="GET"))
PrimaryButtonConfig(name="获取GPSv2", target=CopyConfig(url="/get_gpsv2", method="GET"))

PrimaryButtonConfig(
    name="启用pland", target=ToastConfig(url="/start_pland", method="POST")
)
PrimaryButtonConfig(
    name="关闭pland", target=ToastConfig(url="/stop_pland", method="POST")
)
PrimaryButtonConfig(
    name="启用避障", target=ToastConfig(url="/start_planner", method="POST")
)
PrimaryButtonConfig(name="关闭避障", target=ToastConfig(url="/stop_planner", method="POST"))

PrimaryButtonConfig(name="停止跟随", target=ToastConfig(url="/stop_follow", method="POST"))

PrimaryButtonConfig(name="开始录制", target=ToastConfig(url="/start_record", method="POST"))
PrimaryButtonConfig(name="结束录制", target=ToastConfig(url="/stop_record", method="POST"))
