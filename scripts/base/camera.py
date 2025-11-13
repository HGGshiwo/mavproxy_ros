from base.utils import FPSHelper
import asyncio
from fastapi.responses import StreamingResponse
import rospy
from sensor_msgs.msg import Image
import cv_bridge
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame
import cv2
import threading

def draw_fps(img, fps):
    
    text = f"FPS: {fps:.2f}"
    org = (10, 30)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color_text = (0, 0, 0)      # 黑色
    color_border = (255, 255, 255)  # 白色
    thickness_border = 8
    thickness_text = 2

    # 先画白色边框
    cv2.putText(img, text, org, font, font_scale, color_border, thickness_border, cv2.LINE_AA)
    # 再画黑色文字
    cv2.putText(img, text, org, font, font_scale, color_text, thickness_text, cv2.LINE_AA)
    return img

def create_no_image(img=None, text="No Image", font_scale=2):
    import numpy as np
    import cv2
    if img is None:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
    height, width = img.shape[:2]
    # 设置字体、大小、粗细等
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = font_scale
    thickness = 3
    color = (255, 255, 255)  # 白色

    # 计算文字的宽高，用于居中
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x = (width - text_width) // 2
    y = (height + text_height) // 2

    # 在图片上写字
    cv2.putText(img, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)
    return img

class VideoBuilder:
    def __init__(self, topic):
        rospy.Subscriber(topic, Image, self.image_cb)
        self.image = create_no_image()
        self.bridge = cv_bridge.CvBridge()
        self.cond = threading.Condition()
    
    def image_cb(self, data):
        with self.cond:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cond.notify_all()
    
    def build(self):
        return NotImplementedError()

    @staticmethod
    def create(type, topic):
        if type == "http":
            builder = HTTPVideoBuilder(topic)
        elif type == "webrtc":
            builder = RTCVideoBuilder(topic)
        else:
            raise ValueError(f"unknown video type: {type}")
        return builder
    
class HTTPVideoBuilder(VideoBuilder):
    def cb(self, fps):
        self.fps = fps

    def build(self):
        self.fps = 0
        self.fps_helper = FPSHelper(-1, self.cb)
    
        async def generate_frames():
            try:
                while True:
                    with self.cond:
                        self.cond.wait()
                    frame = draw_fps(self.image.copy(), self.fps)
                    self.fps_helper.step()
                    ret, frame = cv2.imencode(".jpg", frame)
                    frame = frame.tobytes()
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(frame)}\r\n\r\n".encode()
                        + frame
                        + b"\r\n"
                    )
            finally:
                pass

        return StreamingResponse(
            generate_frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

class CallbackStreamTrack(VideoStreamTrack):
    def __init__(self, builder, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fps = 0
        self.fps_helper = FPSHelper(-1, self.cb)
        self.last_frame = create_no_image()
        self.cond = builder.cond
        self.builder = builder

    def cb(self, fps):
        self.fps = fps

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        with self.cond:
            self.cond.wait()
        self.fps_helper.step()
        try:
            frame = draw_fps(self.builder.image.copy(), self.fps)
            video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
            video_frame.pts = pts
            video_frame.time_base = time_base
            
            return video_frame
        except Exception as e:
            import traceback
            rospy.loginfo(traceback.format_exc())
                    
class RTCVideoBuilder(VideoBuilder):

    async def build(self, sdp, type):
        offer = RTCSessionDescription(sdp, type)
        pc = RTCPeerConnection()
        
        track = CallbackStreamTrack(self)
        pc.addTrack(track)

        @pc.on("iceconnectionstatechange")
        def on_iceconnectionstatechange():
            print("ICE connection state is %s" % pc.iceConnectionState)
            if pc.iceConnectionState == "failed":
                asyncio.ensure_future(pc.close())
            if pc.iceConnectionState in ["closed", "failed"]:
                pass
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
