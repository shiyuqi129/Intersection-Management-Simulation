'''
Handle conflict zone logic
'''

from typing import Dict, Tuple, List
from vehicle import Vehicle
from intersection import FourWayIntersection

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
'''
01
32
'''

# Constant for now
PASSING_TIME = 8.0
WAITING_TIME = 2.0

import networkx as nx

class TimingConflictNode:
    def __init__(self, vehicle: Vehicle, zone: int):
        self.vehicle = vehicle
        self.zone = zone
        self.is_in_zone = False
        self.trail_cleared = False
        self.passing_time = PASSING_TIME
    def __repr__(self) -> str:
        return f"{self.vehicle}, {self.zone}"

class TimingConflictGraph:
    def __init__(self, conflicts: Dict[Tuple[int, int], List[int]], intersection: FourWayIntersection):
        self.conflicts = conflicts
        self.G = nx.DiGraph()
        self.intersection = intersection
    def add_vehicle(self, vehicle: Vehicle):
        source = vehicle.source
        destination = vehicle.destination
        conflict_zones = self.conflicts[(source, destination)]
        new_nodes = [TimingConflictNode(vehicle, zone) for zone in conflict_zones]
        self.G.add_nodes_from(new_nodes)
        for i in range(len(new_nodes)-1):
            self.G.add_edge(new_nodes[i], new_nodes[i+1], type = 1)
        for node in self.G.nodes:
            if node in new_nodes:
                continue
            for new_node in new_nodes:
                if node.zone == new_node.zone:
                    if node.vehicle.source == vehicle.source:
                        self.G.add_edge(node, new_node, type=2)
                    else:
                        if not node.is_in_zone:
                            self.G.add_edge(node, new_node, type=3)
                            self.G.add_edge(new_node, node, type=3)
                        else:
                            self.G.add_edge(node, new_node, type=4) 
    def add_vehicles(self, vehicles: List[Vehicle]):
        for vehicle in vehicles:
            self.add_vehicle(vehicle)
    
    def remove_vehicle(self, vehicle: Vehicle):
        remove_node = []
        for node in self.G.nodes:
            if node.vehicle == vehicle:
                remove_node.append(node)
        self.G.remove_nodes_from(remove_node)
    
    def set_vehicle_in_zone(self, vehicle: Vehicle, zone: int):
        target = None
        for node in self.G.nodes:
            if node.vehicle == vehicle and node.zone == zone:
                node.is_in_zone = True
                target = node
                break
        else:
            return
        remove_list = list(self.G.in_edges(target, data=True))
        for edge in remove_list:
            if edge[2]["type"] == 3:
                assert(self.G.has_node(edge[1]))
                assert(self.G.has_node(edge[0]))
                self.G.edges[edge[1], edge[0]]["type"] = 4
            self.G.remove_edge(edge[0], edge[1])
    
    def remove_node(self, vehicle: Vehicle, zone: int):
        for node in self.G.nodes:
            if node.vehicle == vehicle and node.zone == zone:
                self.G.remove_node(node)
                return
            
    def schedule(self):
        if self.G.number_of_nodes() == 0:
            return []
        state_dict, time_dict = _schedule(self)
        dispatch = []
        for node, time in time_dict.items():
            for edge in self.G.in_edges(node, data = True):
                if edge[2]["type"] != 1 and state_dict[edge[0:2]] == "on":
                    break
            else:
                if time <= PASSING_TIME + WAITING_TIME + 0.5:
                    dispatch.append(node)
        return dispatch

def _verify(G: nx.DiGraph, state):
    Gp = nx.DiGraph()
    for edge in G.edges(data=True):
        if edge[2]["type"] == 1:
            Gp.add_node((edge[0], edge[1]))
    def edge_on(n1, n2):
        return G.has_edge(n1, n2) and state[(n1, n2)] == "on"
    for edge0 in Gp.nodes: # type 1 edges as nodes
        for edge1 in Gp.nodes:
            if edge0 == edge1:
                continue
            i00, i01 = edge0
            i10, i11 = edge1
            if edge_on(i00, i10) or edge_on(i00, i11) or edge_on(i01, i10) or edge_on(i01, i11):
                Gp.add_edge(edge0, edge1)
            if edge_on(i10, i00) or edge_on(i10, i01) or edge_on(i11, i00) or edge_on(i11, i01):
                Gp.add_edge(edge1, edge0)
    try:
        nx.find_cycle(Gp)
        return False
    except nx.NetworkXNoCycle:
        return True

def _schedule(conflict_graph: TimingConflictGraph):
    state = dict()
    slack = dict()
    s = dict()
    a = dict()

    G = conflict_graph.G.copy()
    for node in G.nodes:
        state[node] = "white"
        slack[node] = float("inf")
        a[node.vehicle] = conflict_graph.intersection.estimate_arrival_time(node.vehicle)
    for edge in G.edges(data = True):
        if edge[2]["type"] == 3:
            state[edge[0:2]] = "undecided"
        else:
            state[edge[0:2]] = "on"

    vehicle_order = [vehicle for vehicle, arrival_time in a.items()]
    vehicle_order.sort(key=lambda x: a[x])

    def update_time_slack():
        Gp = nx.DiGraph()
        Gp.add_nodes_from(G.nodes)
        Gp.add_edges_from([edge for edge in G.edges(data = True) if state[edge[0:2]] == "on"])
        _order = list(nx.topological_sort(Gp))
        topological_order = _order
        
        '''for node in _order:
            if node in topological_order:
                continue
            topological_order.append(node)
            while True:
                next_edge = None
                for edge in Gp.out_edges(node, data = True):
                    if edge[2]["type"] == 1:
                        next_edge = edge
                        break
                else:
                    break
                node = next_edge[1]
                topological_order.append(node)
        '''
        for node in topological_order:
            s[node] = a[node.vehicle]
            term1 = 0
            term2 = 0
            for edge in Gp.in_edges(node, data = True):
                # TODO : replace constant passing and waiting time with calculated ones
                term1 = max(term1, s.get(edge[0], 0) + edge[0].passing_time + WAITING_TIME)
                for edge2 in Gp.out_edges(edge[0], data = True):
                    if edge == edge2:
                        continue
                    if edge2[2]["type"] == 1:
                        term2 =  max(term2, s.get(edge2[1], 0) - WAITING_TIME + WAITING_TIME)
            s[node] = max(s[node], max(term1, term2))
        max_leaving_time = max(s[node] + node.passing_time for node in topological_order)
        for node in reversed(topological_order):
            slack[node] = max_leaving_time - s[node] - node.passing_time
            for edge in G.out_edges(node):
                slack[node] = min(slack[node], slack[edge[1]])

    def order(node):
        return vehicle_order.index(node.vehicle)

    def find_leaders(start, end):
        leaders = []
        black_list = []
        for node in G.nodes:
            if not start <= order(node) < end:
                continue
            if state[node] == "black":
                continue
            state[node] = "black"
            for edge in G.in_edges(node, data = True):
                if state[(edge[0], edge[1])] == "off":
                    continue
                if state[edge[0]] == "black":
                    continue
                if state[(edge[0], edge[1])] == "on":
                    state[node] = "white"
                    break
                state[node] = "gray"
            if(state[node] == "gray"):
                leaders.append(node)
            if(state[node] == "black"):
                black_list.append(node)
                state[node] = "white"
        for node in black_list:
            state[node] = "black"

    def find_candidates(leader_nodes: List[TimingConflictNode]):
        candidates_set = set()
        for node in leader_nodes:
            for edge in G.in_edges(node, data = True):
                if edge[2]["type"] == 3:
                    candidates_set.add(edge[0:2])
            for edge in G.out_edges(node, data = True):
                if edge[2]["type"] == 3:
                    candidates_set.add(edge[0:2])
        return list(candidates_set)

    def remove_type_three_edges(G: nx.DiGraph, start, end):
        for node in G.nodes:
            if order(node) >= start:
                state[node] = "white"
        for edge in G.edges(data=True):
            if edge[2]["type"] != 3:
                continue
            if order(edge[0]) >= end and order(edge[1]) >= end:
                state[edge[0:2]] = "dontcare"
            elif start <= order(edge[0]) < end and start <= order(edge[1]) < end:
                state[edge[0:2]] = "undecided"
            elif start <= order(edge[0]) < end <= order(edge[1]):
                state[edge[0:2]] = "on"
            elif start <= order(edge[1]) < end <= order(edge[0]):
                state[edge[0:2]] = "off"

        failed = False
        leader_nodes = find_leaders(start, end)
        while leader_nodes:
            candidate_edges = find_candidates(leader_nodes)
            max_cost = float("-inf")
            max_edge = None
            for edge in candidate_edges:
                cost = s[edge[0]] + edge[0].passing_time + WAITING_TIME - s[edge[1]] - slack[edge[1]]
                if cost > max_cost:
                    max_cost = cost
                    max_edge = edge
            if max_edge is None:
                find_leaders(start, end)
                continue

            max_edgep = (max_edge[1], max_edge[0])
            state[max_edge] = "off"
            state[max_edgep] = "on"
            if _verify(G, state) == False:
                state[max_edge] = "on"
                state[max_edgep] = "off"
                if _verify(G, state) == False:
                    failed = True
                    break
            leader_nodes = find_leaders(start, end)
            update_time_slack()
        if failed:
            mid = (start + end + 1) // 2
            remove_type_three_edges(G, start, mid)
            remove_type_three_edges(G, mid, end)

    update_time_slack()
    remove_type_three_edges(G, 0, len(a))
    return state, s