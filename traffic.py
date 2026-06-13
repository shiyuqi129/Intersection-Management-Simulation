'''
Handle traffic spawning
'''

import numpy as np
from numpy.typing import NDArray
from vehicle import Vehicle

from typing import List

class Traffic:
    def __init__(self, arrival_rates: NDArray[np.float64], routing_probability: NDArray[np.float64]) -> None:
        '''
        arrival_rates: Rate that vehicles arrives from each lanes. Shape: (lane)
        routing_probability: Matrix representation of probability of a vehicle from Lane i is going to Lane j. Shape: (lane, lane)
        '''

        if not arrival_rates.ndim == 1:
            raise ValueError("arrival_rates must be 1-dimensional")
        if not routing_probability.ndim == 2:
            raise ValueError("routing_probability must be 2-dimensional")
        if not arrival_rates.shape[0] == routing_probability.shape[0] == routing_probability.shape[1]:
            raise ValueError("The size of arrival_rates and routing_probability does not match.")
        if not np.allclose(routing_probability.diagonal(), 0):
            raise ValueError("The diagonal of routing_probability must be 0")
        if not np.all((routing_probability >= 0) & (routing_probability <= 1)):
            raise ValueError("Probability must be between 0 and 1")
        if not np.allclose(routing_probability[arrival_rates!=0].sum(axis=1), 1):
            raise ValueError("Probabilities along a row must add up to 1")
        
        self.arrival_rates = arrival_rates.copy()
        self.routing_probability = routing_probability.copy()
        self.routing_cdf = np.cumsum(self.routing_probability, axis = 1)

        self.next_vehicle_spawn_time = np.random.exponential(1 / self.arrival_rates)

    def random_destination(self, source: int):
        rand = np.random.random()
        return (self.routing_cdf[source] < rand).sum()

    def peek_next_spawn(self):
        '''
        Find the lane that is closest to spawning the next vehicle and the time till spawning.
        '''
        argmin = np.argmin(self.next_vehicle_spawn_time)
        return int(argmin), float(self.next_vehicle_spawn_time[argmin])
    
    def step_time_and_pop(self, dt: float) -> List[Vehicle]:
        '''
        Forward time by dt unit and return a list of vehicles that spawned in that time frame
        '''
        spawn_list = []
        mask = self.next_vehicle_spawn_time <= dt

        while np.any(mask):
            spawn_list.extend(np.flatnonzero(mask))
            self.next_vehicle_spawn_time[mask] += np.random.exponential(1/self.arrival_rates[mask])
            mask = self.next_vehicle_spawn_time <= dt

        self.next_vehicle_spawn_time -= dt

        return [Vehicle(source, self.random_destination(source)) for source in spawn_list]