import heapq

class Node:
    def __init__(self, position, parent=None):
        self.position = position  # (x, y) coordinates
        self.parent = parent      # Parent node
        self.g = 0                # Cost from start to current node
        self.h = 0                # Heuristic cost to goal
        self.f = 0                # Total cost

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f  # For priority queue

def heuristic(a, b):
    # Using Manhattan distance as heuristic
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(start, goal, grid):
    open_list = []
    closed_set = set()
    start_node = Node(start)
    goal_node = Node(goal)

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        # Check if we reached the goal
        if current_node.position == goal_node.position:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Return reversed path

        closed_set.add(current_node.position)

        # Generate children nodes
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Up, Right, Down, Left
        for new_position in neighbors:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Check if the new position is within the grid bounds and not an obstacle
            if (0 <= node_position[0] < len(grid)) and (0 <= node_position[1] < len(grid[0])) and grid[node_position[0]][node_position[1]] == 1:
                neighbor_node = Node(node_position, current_node)

                if neighbor_node.position in closed_set:
                    continue

                # Calculate costs
                neighbor_node.g = current_node.g + 1
                neighbor_node.h = heuristic(neighbor_node.position, goal_node.position)
                neighbor_node.f = neighbor_node.g + neighbor_node.h

                # Check if this node is already in the open list
                if any(neighbor_node == open_node and neighbor_node.g > open_node.g for open_node in open_list):
                    continue

                heapq.heappush(open_list, neighbor_node)

    return None  # No path found

def print_grid(grid, path=None):
    for i, row in enumerate(grid):
        for j, col in enumerate(row):
            if path and (i, j) in path:
                print(" × ", end="")
            elif col == 1:
                print(" • ", end="")
            else:
                print(" $ ", end="")
        print()

def get_user_input():
    while True:
        try:
            start = tuple(map(int, input("Enter start position (x y): ").split()))
            goal = tuple(map(int, input("Enter goal position (x y): ").split()))
            return start, goal
        except ValueError:
            print("Invalid input. Please enter two integers separated by a space.")

def main():
    # Define the grid (1 = free space, 0 = obstacle)
    grid = [
        [1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1],
        [1, 1, 1, 0, 1],
        [1, 0, 1, 1, 1],
        [1, 1, 1, 1, 1]
    ]

    print("Grid (1 = free space, $ = obstacle):")
    print_grid(grid)

    start, goal = get_user_input()

    # Validate start and goal positions
    if grid[start[0]][start[1]] == 0 or grid[goal[0]][goal[1]] == 0:
        print("Start or goal position is an obstacle. Please choose valid positions.")
        return

    path = a_star_search(start, goal, grid)

    if path:
        print("Path found:")
        print(path)
        print_grid(grid, path)
    else:
        print("No path found.")

if __name__ == "__main__":
    main()