'''
Drawing thing
'''

import pygame

from enum import Enum
from typing import Tuple, List

from math import pi

from vehicle import Vehicle
from intersection import FourWayIntersection
from util import local2world
class COLOR(Enum):
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    CYAN = (0, 255, 255)
    PURPLE = (255, 0, 255)
    YELLOW = (255, 255, 0)
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)

destination_color = [None, COLOR.BLUE.value, None, COLOR.GREEN.value, None, COLOR.RED.value, None, COLOR.YELLOW.value]

class Renderer:
    def __init__(self, surface: pygame.Surface, top_y: float, scale: float = 8.0, border: int = 40):
        '''
        surface: The surface to draw on
        top_y: Where the top is in world coordinate
        scale: Multiply world coordinate unit with scale to get pixels
        border: Border width around the edge of the window
        '''
        self.surface = surface
        self.scale = scale
        self.border = border
        self.top_y = top_y

    def translate_coord(self, coord: Tuple[float, float]) -> Tuple[float, float]:
        x, y = coord
        return x * self.scale + self.border, (self.top_y - y) * self.scale + self.border
    
    def translate_coords(self, coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        return [self.translate_coord(coord) for coord in coords]
    
    def translate_rect(self, rect: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        x, y, w, h = rect
        x *= self.scale
        x += self.border
        y = self.top_y - y
        y *= self.scale
        y += self.border
        w *= self.scale
        h *= self.scale
        return x, y, w, h

    def draw_vehicle(self, vehicle: Vehicle):
        x = vehicle.current_x
        y = vehicle.current_y
        angle = vehicle.current_facing
        w = vehicle.width
        l = vehicle.length
        pts =[
            local2world(x, y, angle,   -0.5 * w,    0.4 * l),
            local2world(x, y, angle,   -0.3 * w,    0.5 * l),
            local2world(x, y, angle,    0.3 * w,    0.5 * l),
            local2world(x, y, angle,    0.5 * w,    0.4 * l),
            local2world(x, y, angle,    0.5 * w,   -0.5 * l),
            local2world(x, y, angle,   -0.5 * w,   -0.5 * l),
        ]
        pts = self.translate_coords(pts)
        pygame.draw.polygon(self.surface, destination_color[vehicle.destination], pts)
    
    def draw_four_way_intersection(self, intersection: FourWayIntersection):
        arm = intersection.arm_length
        turn = intersection.turning_radius
        lane = intersection.lane_width
        span = intersection.span
        # Horizontal
        pygame.draw.rect(self.surface, COLOR.BLACK.value, self.translate_rect((0, span - arm - turn, span, 2 * lane)))
        # Vertican
        pygame.draw.rect(self.surface, COLOR.BLACK.value, self.translate_rect((arm + turn, span, 2 * lane, span)))
        # Turning
        pygame.draw.rect(self.surface, COLOR.BLACK.value, self.translate_rect((arm, arm + turn, turn, turn)))
        pygame.draw.rect(self.surface, COLOR.BLACK.value, self.translate_rect((arm, span - arm, turn, turn)))
        pygame.draw.rect(self.surface, COLOR.BLACK.value, self.translate_rect((span - arm - turn, arm + turn, turn, turn)))
        pygame.draw.rect(self.surface, COLOR.BLACK.value, self.translate_rect((span - arm - turn, span - arm, turn, turn)))
        
        pygame.draw.circle(self.surface, COLOR.WHITE.value, self.translate_coord((arm, arm)), turn * self.scale)
        pygame.draw.circle(self.surface, COLOR.WHITE.value, self.translate_coord((arm, span - arm)), turn * self.scale)
        pygame.draw.circle(self.surface, COLOR.WHITE.value, self.translate_coord((span - arm, arm)), turn * self.scale)
        pygame.draw.circle(self.surface, COLOR.WHITE.value, self.translate_coord((span - arm, span - arm)), turn * self.scale)

        # Destination Indicator
        pygame.draw.rect(self.surface, destination_color[1], ((arm + turn) * self.scale + self.border, self.border / 4, lane * 2 * self.scale, self.border / 2))
        pygame.draw.rect(self.surface, destination_color[3], (span * self.scale + self.border * 1.5, (arm + turn) * self.scale + self.border, self.border / 2, lane * 2 * self.scale))
        pygame.draw.rect(self.surface, destination_color[5], ((arm + turn) * self.scale + self.border, span * self.scale + self.border + self.border / 4, lane * 2 * self.scale, self.border / 2))
        pygame.draw.rect(self.surface, destination_color[7], (self.border / 4, (arm + turn) * self.scale + self.border, self.border / 2, lane * 2 * self.scale))