# 发送信息
在/mavproxy/ws发送topic，可以直接传递给平板

目标检测数据格式：
```json
{
    "type": "event", 
    "event": "detect",
    "other": "..."
}
```

发送方法(python)
```python
from std_msgs.msg import String
import json

ws_pub = rospy.Publisher("/mavros/ws", String, queue_size=1)
ws_pub.publish(json.dumps({
    "type": "event",
    "event": "detect",
    #...
}))
```