import random
import heapq
from collections import deque

# Directions (Up, Down, Left, Right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# Random movement algorithm (limited moves)
def random_move(start, goal, obstacles, rows, cols):
    path = []
    current = start

    for _ in range(1000):  # Limit to 1000 moves
        direction = random.choice(DIRECTIONS)
        new_pos = (current[0] + direction[0], current[1] + direction[1])

        if (
            0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and
            new_pos not in obstacles
        ):
            path.append(direction)
            current = new_pos

        if current == goal:
            return path

    return []  # If it takes too long, return empty path


# Breadth First Search (BFS) Algorithm
def bfs(start, goal, obstacles, rows, cols):
    queue = deque([(start, [])])  # (position, path)
    visited = set([start])

    while queue:
        current, path = queue.popleft()

        if current == goal:
            return path

        for direction in DIRECTIONS:
            new_pos = (current[0] + direction[0], current[1] + direction[1])

            if (0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and
                    new_pos not in obstacles and new_pos not in visited):
                queue.append((new_pos, path + [direction]))
                visited.add(new_pos)

    return []


# Depth First Search (DFS) Algorithm
def dfs(start, goal, obstacles, rows, cols):
    stack = [(start, [])]  # (position, path)
    visited = set([start])

    while stack:
        current, path = stack.pop()

        if current == goal:
            return path

        for direction in DIRECTIONS:
            new_pos = (current[0] + direction[0], current[1] + direction[1])

            if (0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and
                    new_pos not in obstacles and new_pos not in visited):
                stack.append((new_pos, path + [direction]))
                visited.add(new_pos)

    return []


# Iterative Deepening Search (IDS)
def ids(start, goal, obstacles, rows, cols, max_depth=100):
    def dls(node, depth, visited):
        stack = [(node, [], depth)]  

        while stack:
            current, path, remaining_depth = stack.pop()

            if current == goal:
                return path

            if remaining_depth == 0:
                continue 
            visited.add(current)

            for direction in DIRECTIONS:
                new_pos = (current[0] + direction[0], current[1] + direction[1])

                if (
                    0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and
                    new_pos not in obstacles and new_pos not in visited
                ):
                    stack.append((new_pos, path + [direction], remaining_depth - 1))

        return None  

    for depth in range(max_depth): 
        result = dls(start, depth, set())
        if result is not None:
            return result

    return [] 



# Uniform Cost Search (UCS)
def ucs(start, goal, obstacles, rows, cols):
    pq = [(0, start, [])]  # (cost, position, path)
    visited = {}

    while pq:
        cost, current, path = heapq.heappop(pq)

        if current in visited and visited[current] <= cost:
            continue
        visited[current] = cost

        if current == goal:
            return path

        for direction in DIRECTIONS:
            new_pos = (current[0] + direction[0], current[1] + direction[1])

            if (0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and new_pos not in obstacles):
                heapq.heappush(pq, (cost + 1, new_pos, path + [direction]))

    return []


# Greedy Best First Search Algorithm
def greedy_bfs(start, goal, obstacles, rows, cols):
    def heuristic(pos):
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])  # Manhattan distance

    pq = [(heuristic(start), start, [])]  # (heuristic, position, path)
    visited = set()

    while pq:
        _, current, path = heapq.heappop(pq)

        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return path

        for direction in DIRECTIONS:
            new_pos = (current[0] + direction[0], current[1] + direction[1])

            if (0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and new_pos not in obstacles and new_pos not in visited):
                heapq.heappush(pq, (heuristic(new_pos), new_pos, path + [direction]))

    return []


# A* Search Algorithm
def astar(start, goal, obstacles, rows, cols):
    def heuristic(pos):
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])  # Manhattan distance

    pq = [(heuristic(start), 0, start, [])]  # (f-score, cost, position, path)
    visited = {}

    while pq:
        f, cost, current, path = heapq.heappop(pq)

        if current in visited and visited[current] <= cost:
            continue
        visited[current] = cost

        if current == goal:
            return path

        for direction in DIRECTIONS:
            new_pos = (current[0] + direction[0], current[1] + direction[1])

            if (0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and new_pos not in obstacles):
                new_cost = cost + 1
                heapq.heappush(pq, (new_cost + heuristic(new_pos), new_cost, new_pos, path + [direction]))

    return []
