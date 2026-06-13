import numpy as np

class Vehicle:
    def __init__(self, source: int, destination: int, top_speed: float = 15.0, acceleration: float = 2.0, decelaration: float = 3.0, emergency_break: float = 9.0, length: float = 4.0, width: float = 2.0, min_gap: float = 2.0, safty_time: float = 3.0):
        # Vehicle stats
        self.source = source
        self.destination = destination
        self.top_speed = top_speed
        self.acceleration = acceleration
        self.deceleration = decelaration
        self.emergency_break = emergency_break
        self.length = length
        self.width = width
        # Following configuration
        self.min_gap = min_gap # minimum gap between vehicles
        self.safety_time = safty_time # safety time gap between vehicles
        # Vehicle state
        self.current_speed = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_facing = 0.0
        self.distance_travelled = 0.0 # Can be use for predefined trajectory
        # Vehicle control
        self.current_acceleration = 0.0
        self.current_deceleration = 0.0

        self.status = "pending"
        '''
        pending: Newly spawned vehicle
        buffer: In intersection spawn buffer (waiting to enter)
        active: Has been placed on the map
        exited: Exited the map
        crashed: Collided with something else
        '''

        self.phase = "enter"
        '''
        enter: Waiting to enter the intersection
        in: In the intersection
        exit: Exitting the intersection
        '''

        self.target_zone = -1
        '''
        current target conflict zone
        -1: Cannot enter yet
        -2: Exit
        0~3 (N-1): Conflict zone index
        '''
        self.last_zone = -1
        self.target = 0.0

    def action(self, command: str, follow_distance: float | None = None, follow_speed: float | None = None):
        if not self.status == "active":
            return

        if command == "emergency_break":
            self.current_acceleration = 0.0
            self.current_deceleration = self.emergency_break
        elif command == "stop":
            self.current_acceleration = 0.0
            self.current_deceleration = self.deceleration
        elif command == "drive":
            if follow_distance is None: # drive freely
                self.current_acceleration = self.acceleration
                self.current_deceleration = 0.0
            else:
                if follow_speed is None: # driving to target
                    min_gap = 0.0
                    follow_speed = 0.0
                else:
                    min_gap = self.min_gap
                acceleration = 0.5 * (follow_distance - min_gap - self.current_speed * self.safety_time) + 0.3 * (follow_speed - self.current_speed)
                acceleration: float = np.clip(acceleration, -self.emergency_break, self.acceleration)
                if acceleration >= 0:
                    self.current_acceleration = acceleration
                    self.current_deceleration = 0.0
                else:
                    self.current_deceleration = -acceleration
                    self.current_acceleration = 0.0

    def set_status(self, status: str):
        self.status = status