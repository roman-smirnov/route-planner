import queue
from itertools import count

tiebreaker = count()    # tiebreaker for cases where min heap has two elements with equal distance


def euclidean_distance(search_map, node1: int, node2: int):
    """
    Calculate the Euclidean distance between two pairs of intersection coordinate
    This heuristic is both admissible and consistent since it's a simplification of the given search problem
    :param search_map: The map which contains the coordinates for given intersection indices
    :param node1: index of the first intersection
    :param node2: index of the second intersection
    :return: Euclidean distance between the intersections
    """
    x1, y1 = search_map.intersections[node1]
    x2, y2 = search_map.intersections[node2]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_path(came_from: dict, start: int, goal: int):
    """
    Get the path from start to end node by backtracking where each node came from
    :param came_from: The dictionary which contains where each node came from
    :param start: The start node of the path
    :param goal: The end node of the path
    :return: A path from start to goal node
    """
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from.get(current, start)
    path.append(start)
    return path[::-1]


def shortest_path(search_map, start: int, goal: int):
    """
    Calculate the shortest path from start to end node using A* search
    :param search_map: The map which contains the intersections and roads on which to search
    :param start: The node to start the search from
    :param goal: The node to which to search a path
    :return: A path from start to goal node, or empty list if not found
    """
    min_heap = queue.PriorityQueue()
    came_from = dict()
    real_distance = dict()
    real_distance[start] = 0
    min_heap.put((euclidean_distance(search_map, start, goal), next(tiebreaker), start))
    while min_heap.not_empty:
        current = min_heap.get()[2]
        if current == goal:
            return get_path(came_from, start, goal)
        for neighbor in search_map.roads[current]:
            distance = euclidean_distance(search_map, current, neighbor) + real_distance[current]
            if distance < real_distance.get(neighbor, float('inf')):
                came_from[neighbor] = current
                real_distance[neighbor] = distance
            else:
                continue
            estimated_distance = euclidean_distance(search_map, neighbor, goal)
            min_heap.put((distance + estimated_distance, next(tiebreaker), neighbor))
    return []
