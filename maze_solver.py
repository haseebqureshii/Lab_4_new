from imports import np, cv2, heapq


def solve_maze_a_star(image_path, start, end):
    
    # Helper functions
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance
    
    def neighbors(node):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
        result = []
        for d in directions:
            neighbor = (node[0] + d[0], node[1] + d[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] == 255:
                result.append(neighbor)
        return result
    
    maze_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    _, binary_image = cv2.threshold(maze_image, 128, 255, cv2.THRESH_BINARY)
    grid = binary_image
    rows, cols = grid.shape
    
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost_so_far = {start: 0}
    
    while open_list:
        _, current = heapq.heappop(open_list)
        
        if current == end:
            break

        for neighbor in neighbors(current):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, end)
                heapq.heappush(open_list, (priority, neighbor))
                came_from[neighbor] = current
    
    path = []
    current = end
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    
    return path
