from event_callback.components.http.ui_config import (
    CopyConfig,
    FormConfig,
    InnerButtonConfig,
    InputFormItemConfig,
    NumberFormItemConfig,
    PrimaryButtonConfig,
    RadioFormItemConfig,
    StatusConfig,
    ToastConfig,
)

StatusConfig("connected", "连接状态")
StatusConfig("arm", "解锁")
StatusConfig("mode", "飞控模式")
StatusConfig("state", "单控模式")
StatusConfig("gps_nsats", "GPS星数")
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
        items=dict(alt=NumberFormItemConfig(name="起飞高度", default=10)),
        submit=InnerButtonConfig(target=ToastConfig(url="/takeoff", method="POST")),
    ),
)
PrimaryButtonConfig(name="解锁", target=ToastConfig(url="/arm", method="POST"))
PrimaryButtonConfig(
    name="GUIDED",
    target=ToastConfig(url="/set_mode", method="POST", data=dict(mode="GUIDED")),
)
PrimaryButtonConfig(
    name="悬停",
    target=ToastConfig(url="/set_mode", method="POST", data=dict(mode="LOITER")),
)
PrimaryButtonConfig(name="返航", target=ToastConfig(url="/return", method="POST"))
PrimaryButtonConfig(name="降落", target=ToastConfig(url="/land", method="POST"))
PrimaryButtonConfig(name="重启飞控", target=ToastConfig(url="/reboot_fcu", method="POST"))
PrimaryButtonConfig(
    name="输入航点",
    target=FormConfig(
        title="输入航点",
        items=dict(
            waypoint=InputFormItemConfig(name="输入航点", default="", transform="json"),
            vel=NumberFormItemConfig(name="输入速度", default=5),
        ),
        submit=InnerButtonConfig(
            target=ToastConfig(url="/set_waypoint", method="POST")
        ),
    ),
)
PrimaryButtonConfig(name="获取航点", target=CopyConfig(url="/get_waypoint", method="GET"))
PrimaryButtonConfig(name="获取GPS", target=CopyConfig(url="/get_gps", method="GET"))
PrimaryButtonConfig(name="获取GPSv2", target=CopyConfig(url="/get_gpsv2", method="GET"))


PrimaryButtonConfig(name="停止跟随", target=ToastConfig(url="/stop_follow", method="POST"))

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

PrimaryButtonConfig(
    name="开启检测",
    target=FormConfig(
        title="开启检测",
        url="/get_detect",
        method="GET",
        items=dict(
            type=RadioFormItemConfig(
                name="检测类型", options=dict(smoke="烟雾", nohardhat="头盔")
            )
        ),
        submit=InnerButtonConfig(
            target=ToastConfig(url="/start_detect", method="POST")
        ),
    ),
)
PrimaryButtonConfig(name="关闭检测", target=ToastConfig(url="/stop_detect", method="POST"))

PrimaryButtonConfig(
    name="开始录制",
    target=FormConfig(
        title="开始录制",
        items=dict(bag_name=InputFormItemConfig(name="录制名称", default="task")),
        submit=InnerButtonConfig(
            name="提交", target=ToastConfig(url="/start_record", method="POST")
        ),
    ),
)
PrimaryButtonConfig(name="结束录制", target=ToastConfig(url="/stop_record", method="POST"))

PrimaryButtonConfig(
    name="云台控制",
    target=FormConfig(
        title="云台控制",
        url="/get_gimbal",
        method="GET",
        items=dict(
            mode=RadioFormItemConfig(name="模式", options=dict(body="body", abs="abs")),
            angle=NumberFormItemConfig(name="角度"),
        ),
        submit=InnerButtonConfig(
            name="提交", target=ToastConfig(url="/set_gimbal", method="POST")
        ),
    ),
)

PrimaryButtonConfig(
    name="曝光控制",
    target=FormConfig(
        url="/get_exposure",
        method="GET",
        title="曝光控制",
        items=dict(
            shutter=NumberFormItemConfig(name="shutter"),
            sensitivity=NumberFormItemConfig(name="sensitivity"),
        ),
        submit=InnerButtonConfig(
            name="提交", target=ToastConfig(url="/set_exposure", method="POST")
        ),
    ),
)
