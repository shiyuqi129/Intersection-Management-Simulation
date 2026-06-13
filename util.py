'''
Random util function that don't belong in other place
'''
from typing import Tuple
import math

def local2world(ox, oy, theta, x, y):
    '''
    (ox, oy): The observer's position in the world coordinate
    theta: The observer's facing angle (y-axis of the local coordinate)
    (x, y): A point in the local coordinate

    Output: (x, y) to the world coordinate
    '''
    return ox + x * math.sin(theta) + y * math.cos(theta), oy - x * math.cos(theta) + y * math.sin(theta)

def world2local(ox: float, oy: float, theta: float, x: float, y: float) -> Tuple[float, float]:
    '''
    (ox, oy): The observer's position in the local coordinate
    theta: The observer's facing angle (y-axis of the local coordinate)
    (x, y): A point in the world coordinate

    Output: (x, y) to the world coordinate
    '''
    x, y = x - ox, y - oy
    return x * math.sin(theta) - y * math.cos(theta), x * math.cos(theta) - y * math.sin(theta)