import time
import threading

class EventType:
    IDLE = "idle"

class CtrlNode:
    def __init_subclass__(cls):
        event_handler = {}
        for name, func in cls.__dict__.items():
            if hasattr(func, "event"):
                handler = event_handler.get(func.event, [])
                handler.append(func)
                event_handler[func.event] =  handler
        cls.event_handler = event_handler
        
    def __init__(self, node_type):
        self.type = node_type
        self.runner = None
    
    def enter(self):
        pass
    
    def _register(self, event_type, func):
        func_new = lambda _, **kwargs: func(**kwargs) # 忽略self
        func_new.__name__ = func.__name__
        handler = self.event_handler.get(event_type, [])
        handler.append(func_new)
        self.event_handler[event_type] =  handler
    
    @staticmethod
    def on(event_type):
        def warpper(func):
            func.event = event_type
            return func
        return warpper
    
    def step(self, node_type):
        self.runner.step(node_type)

class Event:
    def __init__(self, event_type, **kwargs):
        self.type = event_type
        self.data = kwargs

class Runner:
    def __init__(self, node_list=None, step_cb=None, context=None, idle_hz=10):
        self.node = None
        self.step_cb = step_cb
        self.node_map = {}
        self.idle_hz = idle_hz
        self.context = context
        self.lock = threading.Lock()
        
        if node_list is not None:
            self.add_node_list(node_list)
            self.node = node_list[0]
        
        threading.Thread(target=self.idle, daemon=True).start()
        
    def idle(self):
        last = time.time()
        while True:
            cur = time.time()
            if cur - last > 1 / self.idle_hz:
                self.trigger(EventType.IDLE)
        
    def step(self, node_type):
        if self.step_cb is not None:
            self.step_cb(self.node.type, node_type)
        self.node = self.node_map[node_type]
        self.node.enter()
        
    def add_node(self, node):
        self.node_map[node.type] = node
        node.context = self.context
        node.runner = self
    
    def add_node_list(self, node_list):
        for node in node_list:
            self.add_node(node)
    
    def set_init_node(self, node_type):
        self.init_node = self.node_map[node_type]
        
    def set_step_cb(self, step_cb):
        self.step_cb = step_cb
        
    def trigger(self, event_type, **kwargs):
            if self.node is None:
                return
            handlers = self.node.event_handler.get(event_type, [])
            for h in handlers:
                with self.lock:
                    # print(self.node.type, h.__name__, kwargs)
                    h(self.node, **kwargs)
                    