class CtrlNode:
    def __init_subclass__(cls):
        event_handler = {}
        for name, func in cls.__dict__.items():
            if hasattr(func, "event"):
                handler = event_handler.get(func.event, [])
                handler.append(func)
                event_handler[func.event] =  handler
        
    def __init__(self, node_type):
        self.type = node_type
        self.runner = None
        
    def on(self, event_type):
        def warpper(func):
            func.event = event_type
            return func
        return warpper
    
    def step(self, node_type):
        self.runner.step(node_type)
    
class Runner:
    def __init__(self):
        self.init_node = None
        self.step_cb = None
        self.node_map = {}
        
    def step(self, node_type):
        self.step_cb(node_type)
        self.node = self.node_map[node_type]
        
    def add_node(self, node):
        self.node_map[node.type] = node
    
    def add_node_list(self, node_list):
        for node in node_list:
            self.add_node(node)
    
    def set_init_node(self, node_type):
        self.init_node = self.node_map[node_type]
        
    def set_step_cb(self, step_cb):
        self.step_cb = step_cb