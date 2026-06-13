'''
main loop
'''

import pygame
import numpy as np

import random
from render import Renderer
from vehicle import Vehicle
from intersection import FourWayIntersection
from trajectory import FourWayIntersectionTrajectory
from traffic import Traffic
from conflict_zone_manager import TimingConflictGraph, N4_CONFLICTZONES, TimingConflictNode

from typing import List

pygame.init()

WIDTH, HEIGHT, FPS = 800, 600, 30
TPF = 1 # Physic tick per frame

window = pygame.display.set_mode((WIDTH, HEIGHT))

clock = pygame.time.Clock()

running = True
test_car = None
intersection = FourWayIntersection()
trajectory = FourWayIntersectionTrajectory(intersection)
renderer = Renderer(window, intersection.span)

arrival_rates = np.array([0.1, 0, 0.1, 0, 0.1, 0, 0.1, 0])
routing_probability = np.array([[1/3 if i%2==0 and j%2==1 and j!=i+1 else 0 for j in range(8)] for i in range(8)])

traffic = Traffic(arrival_rates, routing_probability)
conflict_graph = TimingConflictGraph(N4_CONFLICTZONES, intersection)

tick = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Add new vehicle
    new_vehicles = traffic.step_time_and_pop(1/(TPF*FPS))
    intersection.add_vehicles(new_vehicles)
    intersection.try_spawn()

    conflict_graph.add_vehicles(new_vehicles)

    dispatch: List[TimingConflictNode] = conflict_graph.schedule()

    for node in dispatch:
        if node.is_in_zone:
            continue
        vehicle, zone = node.vehicle, node.zone
        target = trajectory.conflict_zone_target_position(vehicle, vehicle.target_zone)
        if target - vehicle.distance_travelled <= 1:
            print("dispatch")
            conflict_graph.set_vehicle_in_zone(vehicle, zone)
            vehicle.last_zone = vehicle.target_zone
            vehicle.target_zone = zone

    for car in intersection.get_active_vehicles():
        if car.distance_travelled >= trajectory.conflict_zone_target_position(car, car.last_zone) + car.length / 2:
            conflict_graph.remove_node(car, car.last_zone)
            print("remove")
        if trajectory.set_vehicle_state_with_distance_travelled(car):
            conflict_graph.remove_vehicle(car)
            print("exit")
        action = "drive" #if car.distance_travelled + car.length/2 < intersection.arm_length else "stop"
        car.action("drive", car.target - car.distance_travelled, 5*(car.target - car.distance_travelled))
        car.current_speed = np.clip(car.current_speed + (car.current_acceleration - car.current_deceleration) / (TPF+FPS), 0, car.top_speed)
        car.distance_travelled += car.current_speed / (TPF*FPS)


    tick += 1
    if tick % TPF != 0:
        continue # skip rendering

    # Clear screen
    window.fill((255, 255, 255))

    # Draw stuff here
    renderer.draw_four_way_intersection(intersection)
    for car in intersection.get_active_vehicles():
        renderer.draw_vehicle(car)
    intersection.update_active_vehicles_list()

    # Show frame
    pygame.display.flip()

    clock.tick(FPS)

pygame.quit()