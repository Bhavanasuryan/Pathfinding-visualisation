import pygame
import heapq
import math

# Initialize pygame
pygame.init()

# Define constants
WIDTH = 800
HEIGHT = 800
GRID_SIZE = 20
ROWS = HEIGHT // GRID_SIZE
COLS = WIDTH // GRID_SIZE

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
GRAY = (169, 169, 169)
PURPLE = (128, 0, 128)
TURQUOISE = (64, 224, 208)

# Define grid and node class
class Node:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.x = col * GRID_SIZE
        self.y = row * GRID_SIZE
        self.color = WHITE
        self.neighbors = []
        self.previous = None
        self.g = float("inf")
        self.f = float("inf")

    def get_position(self):
        return self.row, self.col

    def is_wall(self):
        return self.color == BLACK

    def reset(self):
        self.color = WHITE
        self.previous = None
        self.g = float("inf")
        self.f = float("inf")

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, GRID_SIZE, GRID_SIZE))

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < ROWS - 1 and not grid[self.row + 1][self.col].is_wall():  # Down
            self.neighbors.append(grid[self.row + 1][self.col])
        if self.row > 0 and not grid[self.row - 1][self.col].is_wall():  # Up
            self.neighbors.append(grid[self.row - 1][self.col])
        if self.col < COLS - 1 and not grid[self.row][self.col + 1].is_wall():  # Right
            self.neighbors.append(grid[self.row][self.col + 1])
        if self.col > 0 and not grid[self.row][self.col - 1].is_wall():  # Left
            self.neighbors.append(grid[self.row][self.col - 1])

# Heuristic function (Manhattan Distance)
def heuristic(a, b):
    return abs(a.row - b.row) + abs(a.col - b.col)

# A* algorithm
def astar_algorithm(draw, grid, start, end):
    open_set = []
    closed_set = set()

    heapq.heappush(open_set, (start.f, start))  # Push the start node onto the priority queue

    start.g = 0
    start.f = heuristic(start, end)

    while open_set:
        current_node = heapq.heappop(open_set)[1]  # Get the node with the lowest f score

        if current_node == end:  # Path found
            reconstruct_path(draw, start, end)
            return True

        closed_set.add(current_node)

        for neighbor in current_node.neighbors:
            if neighbor in closed_set:
                continue

            tentative_g = current_node.g + 1

            if tentative_g < neighbor.g:
                neighbor.previous = current_node
                neighbor.g = tentative_g
                neighbor.f = neighbor.g + heuristic(neighbor, end)

                if neighbor not in open_set:
                    heapq.heappush(open_set, (neighbor.f, neighbor))

        draw()

    return False

# Dijkstra's algorithm
def dijkstra_algorithm(draw, grid, start, end):
    open_set = []
    closed_set = set()

    heapq.heappush(open_set, (start.f, start))

    start.g = 0
    start.f = heuristic(start, end)

    while open_set:
        current_node = heapq.heappop(open_set)[1]

        if current_node == end:
            reconstruct_path(draw, start, end)
            return True

        closed_set.add(current_node)

        for neighbor in current_node.neighbors:
            if neighbor in closed_set:
                continue

            tentative_g = current_node.g + 1

            if tentative_g < neighbor.g:
                neighbor.previous = current_node
                neighbor.g = tentative_g
                neighbor.f = neighbor.g

                if neighbor not in open_set:
                    heapq.heappush(open_set, (neighbor.f, neighbor))

        draw()

    return False

# BFS algorithm
def bfs_algorithm(draw, grid, start, end):
    queue = []
    visited = set()

    queue.append(start)
    visited.add(start)

    while queue:
        current_node = queue.pop(0)

        if current_node == end:
            reconstruct_path(draw, start, end)
            return True

        for neighbor in current_node.neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                neighbor.previous = current_node
                queue.append(neighbor)

        draw()

    return False

# Reconstruct the path from end to start
def reconstruct_path(draw, start, end):
    current_node = end
    while current_node != start:
        current_node.color = PURPLE
        current_node = current_node.previous
        draw()

# Draw the grid
def make_grid():
    grid = []
    for row in range(ROWS):
        grid.append([])
        for col in range(COLS):
            node = Node(row, col)
            grid[row].append(node)
    return grid

# Draw the window
def draw_window(win, grid):
    win.fill(WHITE)

    for row in grid:
        for node in row:
            node.draw(win)

    pygame.display.update()

# Main function
def main():
    win = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Pathfinding Algorithm Visualization")

    grid = make_grid()
    start_node = None
    end_node = None
    run = True
    algorithm_running = False
    start_set = False
    end_set = False

    while run:
        draw_window(win, grid)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if event.type == pygame.MOUSEBUTTONDOWN and not algorithm_running:
                row, col = pygame.mouse.get_pos()
                row //= GRID_SIZE
                col //= GRID_SIZE
                node = grid[row][col]

                if not start_set:
                    start_node = node
                    start_node.color = GREEN
                    start_set = True
                elif not end_set:
                    end_node = node
                    end_node.color = RED
                    end_set = True
                elif node != start_node and node != end_node:
                    node.color = BLACK

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:  # Reset the grid
                    start_set = False
                    end_set = False
                    for row in grid:
                        for node in row:
                            node.reset()
                elif event.key == pygame.K_SPACE:  # Start pathfinding
                    if start_node and end_node:
                        algorithm_running = True
                        for row in grid:
                            for node in row:
                                node.update_neighbors(grid)

                        astar_algorithm(lambda: draw_window(win, grid), grid, start_node, end_node)
                        # Uncomment the next line to run Dijkstra instead of A*
                        # dijkstra_algorithm(lambda: draw_window(win, grid), grid, start_node, end_node)
                        # Uncomment the next line to run BFS instead of A* or Dijkstra
                        # bfs_algorithm(lambda: draw_window(win, grid), grid, start_node, end_node)

    pygame.quit()

if __name__ == "__main__":
    main()
