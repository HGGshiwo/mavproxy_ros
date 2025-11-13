import time

def SUCCESS_RESPONSE(msg = "OK"):
    return {"msg": msg, "status": "success"}

def ERROR_RESPONSE(msg):
    return {"msg": msg, "status": "error"}
        
        
class FPSHelper:
    def __init__(self, fps=-1, ps_cb=None):
        self.target_fps = fps
        self.start = time.time()
        self.cnt_start = time.time()
        self.frame_cnt = 0
        self.fps = 0
        self.ps_cb = ps_cb

    def step(self, block=True):
        now = time.time()
        if now - self.cnt_start > 1:
            self.fps = self.frame_cnt / (now - self.cnt_start)
            self.frame_cnt = 0
            self.cnt_start = time.time()
            if self.ps_cb is not None:
                self.ps_cb(self.fps)
        
        self.frame_cnt += 1
        if block:
            if self.target_fps > 0 and now - self.start < 1 / self.target_fps:
                time.sleep(1 / self.target_fps - now + self.start)
            self.start = time.time()
            # self.frame_cnt += 1
        else:
            if self.target_fps > 0 and now - self.start < 1 / self.target_fps:
                trigger = False
            else:
                trigger = True
                self.start = time.time()
                # self.frame_cnt += 1
            return trigger