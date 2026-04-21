# src/euro/euro/shared_state.py

class _RobotState:
    def __init__(self):
        self.IDLE = True
        self.IDLE_FOR_SYSTEMS = True
        self.PREVIOUS_IDLE_FOR_SYSTEMS = True
        self.SYSTEMS_IDLE_COUNT = 0
        self.EMERGENCY_STOP = False
        self.ESP_FRONT_IDLE = True
        self.ESP_BACK_IDLE = True
        self.color_pt_front = None
        self.color_pt_back = None
        self.color_pt = None
        self.SHUTDOWN_REQUESTED = False
        self.WAITING_TIRETTE = False
        self.replanning = False
        self.last_stm_command = ""
        self.return_home_triggered = False
        self.known_id_front = None
        self.known_id_back = None
        self.STEAL_LOOP_ACTIVE = False
        self.STEAL_LOOP_RUNNING = False
        self.STEAL_REPEAT_COUNT = 30
        self.ESP_DISPATCH_EPOCH = 0
        self.TIME_ENDED = False

# Singleton instance of the robot state
# This allows all modules to import and modify the same object.
state = _RobotState()
