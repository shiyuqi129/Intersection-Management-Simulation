from intersection import FourWayIntersection
from vehicle import Vehicle
N4_CONFLICTZONES = {
    (0, 3) : [0, 3, 2],
    (0, 5) : [0, 3],
    (0, 7) : [0],
    (2, 5) : [1, 0, 3],
    (2, 7) : [1, 0],
    (2, 1) : [1],
    (4, 7) : [2, 1, 0],
    (4, 1) : [2, 1],
    (4, 3) : [2],
    (6, 1) : [3, 2, 1],
    (6, 3) : [3, 2],
    (6, 5) : [3],
}


from typing import Tuple
import math
from math import pi

class FourWayIntersectionTrajectory:
    def __init__(self, intersection: FourWayIntersection):
        self.intersection = intersection
        self.type_to_straight_dist = {
            "left": intersection.arm_length + intersection.lane_width,
            "right": intersection.arm_length,
            "straight": intersection.span / 2,
        }
        self.type_to_turn_angle = {
            "left": pi/2,
            "right": -pi/2,
            "straight": 0,
        }
        self.turn_radius = intersection.turning_radius + intersection.lane_width / 2

    def lane_to_position_and_facing(self, lane, distance_from_end) -> Tuple[float, float, float]:
        if lane == 0:
            return(self.intersection.span/2-self.intersection.lane_width/2,self.intersection.span-distance_from_end, 3*pi/2)
        if lane == 1:
            return(self.intersection.span/2+self.intersection.lane_width/2,self.intersection.span-distance_from_end, pi/2)
        if lane == 2:
            return(self.intersection.span-distance_from_end,self.intersection.span/2+self.intersection.lane_width/2, pi)
        if lane == 3:
            return(self.intersection.span-distance_from_end,self.intersection.span/2-self.intersection.lane_width/2, 0)
        if lane == 4:
            return(self.intersection.span/2+self.intersection.lane_width/2,distance_from_end, pi/2)
        if lane == 5:
            return(self.intersection.span/2-self.intersection.lane_width/2,distance_from_end, 3*pi/2)
        if lane == 6:
            return(distance_from_end,self.intersection.span/2-self.intersection.lane_width/2, 0)
        if lane == 7:
            return(distance_from_end,self.intersection.span/2+self.intersection.lane_width/2, pi)
        else:
            raise ValueError(f"lane({lane}) is out of bound")

    def set_vehicle_state_with_distance_travelled(self, vehicle: Vehicle) -> bool:
        source = vehicle.source
        destination = vehicle.destination
        turn_type = FourWayIntersection.get_turn_type(source, destination)

        turn_angle = self.type_to_turn_angle[turn_type]
        arc_length = abs(turn_angle) * self.turn_radius
        straight_length = self.type_to_straight_dist[turn_type]

        # Calculate target
        vehicle.target = self.conflict_zone_target_position(vehicle, vehicle.target_zone)

        ret = False
        # Calculate phase and status
        if vehicle.distance_travelled <= self.intersection.arm_length:
            vehicle.phase = "enter"
        elif vehicle.distance_travelled >= straight_length * 2 + arc_length - self.intersection.arm_length:
            vehicle.phase = "exit"
            if vehicle.target_zone != -2:
                vehicle.last_zone = vehicle.target_zone
                vehicle.target_zone = -2
            if vehicle.distance_travelled >= straight_length * 2 + arc_length:
                vehicle.status = "exited"
                ret = True
        else:
            vehicle.phase = "in"

        # Calculate position
        if vehicle.distance_travelled <= straight_length:
            vehicle.current_x, vehicle.current_y, vehicle.current_facing = self.lane_to_position_and_facing(source, vehicle.distance_travelled)
            return ret
        if vehicle.distance_travelled >= straight_length + arc_length:
            vehicle.current_x, vehicle.current_y, vehicle.current_facing = self.lane_to_position_and_facing(destination, straight_length * 2 + arc_length - vehicle.distance_travelled)
            return ret

        turn_progress = (vehicle.distance_travelled - straight_length) / arc_length
        
        arc_start_x, arc_start_y, arc_start_facing = self.lane_to_position_and_facing(source, straight_length)

        turn_center_x = arc_start_x + math.cos(arc_start_facing + turn_angle) * self.turn_radius
        turn_center_y = arc_start_y + math.sin(arc_start_facing + turn_angle) * self.turn_radius


        vehicle.current_facing = (arc_start_facing + turn_angle * turn_progress)

        offset_angle = arc_start_facing + turn_angle * turn_progress - turn_angle

        vehicle.current_x = turn_center_x + math.cos(offset_angle) * self.turn_radius
        vehicle.current_y = turn_center_y + math.sin(offset_angle) * self.turn_radius

        return ret
    
    def conflict_zone_target_position(self, vehicle, zone) -> float:
        zones = N4_CONFLICTZONES[(vehicle.source, vehicle.destination)]
        if zone == -1:
            return self.intersection.arm_length - vehicle.length / 2
        if zone == -2 or zone == zones[-1]:
            return self.intersection.span * 1000
        
        if zone == zones[0]:
            return self.intersection.span / 2 - vehicle.length / 2

        return self.type_to_straight_dist["left"]  + pi/4 * self.turn_radius