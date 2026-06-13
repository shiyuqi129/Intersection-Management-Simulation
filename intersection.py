from typing import List, Dict, Tuple
from collections import deque
from vehicle import Vehicle
from math import pi


class IntersectionBufferManager:
    class SpawnBuffer:
        '''
        Store the spawned vehicles in a buffer
        '''
        def __init__(self, maxlen: int | None = None) -> None:
            self.buffer = deque(maxlen=maxlen)
        def push(self, vehicle: Vehicle):
            self.buffer.appendleft(vehicle)
        def pop(self) -> Vehicle:
            return self.buffer.pop()
        def peak(self):
            return self.buffer[-1]
        def __len__(self):
            return len(self.buffer)
        @property
        def list(self):
            return list(self.buffer)

    def __init__(self, num_lanes: int, source_lanes: List[int], destination_lanes: List[int] | None = None, invalid_routes: List[Tuple[int, int]] | None = None) -> None:
        if any(source_lane < 0 or source_lane >= num_lanes for source_lane in source_lanes):
            raise ValueError("Some source lanes are out of bound")
        if destination_lanes is not None:
            if any(destination_lane < 0 or destination_lane >= num_lanes for destination_lane in destination_lanes):
                raise ValueError("Some destination lanes are out of bound")
        self.num_lanes = num_lanes
        self.source_lanes = set(source_lanes)
        self.destination_lanes = set([i for i in range(num_lanes) if i not in source_lanes])
        if invalid_routes is None:
            invalid_routes = []
        self.invalid_pairs = set(invalid_routes)
        self.buffers = {lane : self.SpawnBuffer() for lane in source_lanes}

    def push(self, vehicle: Vehicle):
        if vehicle.source not in self.source_lanes:
            raise ValueError(f"{vehicle.source} is not in the set of source lanes")
        if vehicle.destination not in self.destination_lanes:
            raise ValueError(f"{vehicle.destination} is not in the set of destination lanes")
        if (vehicle.source, vehicle.destination) in self.invalid_pairs:
            raise ValueError(f"({vehicle.source}, {vehicle.destination}) is in the set of invalid routes")
        self.buffers[vehicle.source].push(vehicle)
    
    def pop(self, source: int) -> Vehicle | None:
        try:
            return self.buffers[source].pop()
        except IndexError:
            return None
    
    def peak(self, source: int) -> Vehicle | None:
        try:
            return self.buffers[source].peak()
        except IndexError:
            return None
class FourWayIntersection:
    '''
    Only two lane for now
    Lanes are indexed from the top going clock-wise from 0 to 7.
    '''
    TURN_TYPE: List[str] = ["Invalid", "Invalid", "Invalid", "left", "Invalid", "straight", "Invalid", "right"]
    DRIVING_DIRECTION: List[float] = [3*pi/2, pi/2, pi, 0, pi/2, 3*pi/2, 0, pi]

    def __init__(self, lane_width: float = 6.0, turning_radius: float = 2.0, arm_length: float = 20, spawn_gap: float = 2):
        self.lane_width = lane_width
        self.turning_radius = turning_radius
        self.arm_length = arm_length
        self.spawn_gap = spawn_gap

        self.source_lanes = [0,2,4,6]
        self.destination_lanes = [1,3,5,7]
        self.buffer_manager = IntersectionBufferManager(8, self.source_lanes, self.destination_lanes, [(0,1),(2,3),(4,5),(6,7)])
        self.active_enter_vehicle_list: List[Vehicle] = []
        self.active_exit_vehicle_list: List[Vehicle] = []
        self.active_in_vehicle_list: List[Vehicle] = []

        # The user decides what to do with this
        self.crashed_vehicle_list: List[Vehicle] = []

    @property
    def span(self):
        return 2*(self.lane_width+self.turning_radius+self.arm_length)

    def add_vehicle(self, vehicle: Vehicle) -> None:
        self.buffer_manager.push(vehicle)
        vehicle.status = "buffer"

    def add_vehicles(self, vehicles: List[Vehicle]) -> None:
        for vehicle in vehicles:
            self.buffer_manager.push(vehicle)

    def _spawn_from_lane(self, source:int) -> Vehicle | None:
        try:
            vehicle = self.buffer_manager.pop(source)
            if vehicle == None:
                return
            self.active_enter_vehicle_list.append(vehicle)
            vehicle.status = "active"
            return vehicle
        except KeyError:
            raise ValueError(f"{source} is not in the list of valid sources")
        
    def get_active_vehicles(self) -> List[Vehicle]:
        return self.active_enter_vehicle_list+self.active_exit_vehicle_list+self.active_in_vehicle_list
    
    def update_active_vehicles_list(self, clear_crash: bool = False) -> None:
        '''
        Move all the active vehicles in the correction list and remove exited vehicles
        '''
        all_active_list = self.get_active_vehicles()
        self.active_enter_vehicle_list.clear()
        self.active_in_vehicle_list.clear()
        self.active_exit_vehicle_list.clear()

        for vehicle in all_active_list:
            if vehicle.status == "exited":
                continue
            if vehicle.status == "crashed":
                if not clear_crash:
                    self.crashed_vehicle_list.append(vehicle)
                continue
            if vehicle.phase == "enter":
                self.active_enter_vehicle_list.append(vehicle)
            elif vehicle.phase == "in":
                self.active_in_vehicle_list.append(vehicle)
            elif vehicle.phase == "exit":
                self.active_exit_vehicle_list.append(vehicle)
            else:
                raise ValueError(f"{vehicle.phase} is not a valid phase for a vehicle")

    def try_spawn(self) -> List[Vehicle]:
        spawn_room = [self.arm_length * 2] * 8
        for vehicle in self.get_active_vehicles():
            if spawn_room[vehicle.source] >= vehicle.distance_travelled - vehicle.length / 2:
                spawn_room[vehicle.source] = vehicle.distance_travelled - vehicle.length / 2
        
        spawn_list = []
        for source in self.source_lanes:
            next = self.buffer_manager.peak(source)
            if next == None:
                continue
            if spawn_room[source] >= self.spawn_gap + next.length / 2:
                vehicle = self._spawn_from_lane(source)
                if vehicle is not None:
                    spawn_list.append(vehicle)
        return spawn_list
    
    def estimate_arrival_time(self, vehicle: Vehicle) -> float:
        '''
        (very) rough estimate of vehicle arrival time
        '''
        distance = self.arm_length- vehicle.distance_travelled + vehicle.length / 2
        if distance <= 0:
            return 0.0
        return distance / vehicle.top_speed


    @classmethod
    def get_turn_type(cls, source: int, destination: int) -> str:
        return cls.TURN_TYPE[(destination-source)%8]
