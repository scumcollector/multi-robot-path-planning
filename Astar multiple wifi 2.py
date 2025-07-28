import pygame
import math
from queue import PriorityQueue
import socket

import serial #test2
import time #test2

from itertools import groupby

#display size config
WIDTH = 800 #800 for pc 600 for laptop
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Multi-Robot Path Planning")

#colors
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQOISE = (64, 224, 208)
PINK = (255, 192, 203)  # For robot 2 path


# Robot configurations
robots = [
    {'id': 0, 'start': None, 'end': None, 'color': ORANGE, 'path_color': PURPLE, 'ip': '10.29.142.98'},
    {'id': 1, 'start': None, 'end': None, 'color': BLUE, 'path_color': PINK, 'ip': '10.29.142.153'}
]

reservation_table = {}  # (row, col, time): robot_id

#represent each block in screen represents what
class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
        self.robot_id = None  # Track which robot this belongs to

    def get_pos(self):
        return self.row, self.col
    
    def is_closed(self):
        return self.color == RED
    
    def is_open(self):
        return self.color == GREEN
    
    def is_barrier(self):
        return self.color == BLACK
    
    def is_start(self):
        return self.color == ORANGE
    
    def is_end(self):
        return self.color == TURQOISE
    
    def reset(self):
        self.color = WHITE

    def make_closed(self):
        self.color = RED
    
    def make_open(self):
        self.color = GREEN
    
    def make_barrier(self):
        self.color = BLACK
    
    def make_start(self):
        self.color = ORANGE
    
    def make_end(self):
        self.color = TURQOISE   
        
    def make_path(self):
        self.color = PURPLE
    
    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def make_start(self, robot_id):
        self.color = robots[robot_id]['color']
        self.robot_id = robot_id

    def make_path(self, robot_id):
        self.color = robots[robot_id]['path_color']
        self.robot_id = robot_id

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): #it will check if the current row is at the last row AND if the row before it has any barrier.
            self.neighbors.append(grid[self.row + 1][self.col]) #down

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): #up
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): #right
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.row > 0 and not grid[self.row][self.col - 1].is_barrier(): #left 
            self.neighbors.append(grid[self.row][self.col - 1])
    #appending neighbours are just to see what path is available with no barriers or out of the grid that we set.

    def __lt__(self, other):
        return False
    

#calculate the manhattan distance with heuristic function 
def h(p1,p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

#this is basically the 'final answer' the blocks that are purple. thats why the solution will be animated backwards using came_from
def reconstruct_time_path(came_from, end, start, robot_id):
    # Find the key with the end node
    end_key = None
    for key in came_from:
        if key[0] == end:
            end_key = key
            break

    if end_key is None:
        print("Error: End node not found in came_from.")
        return []

    path = []
    current_key = end_key

    while current_key in came_from:
        node, time = current_key
        reservation_table[(node.row, node.col, time)] = robot_id  
        path.append(node.get_pos())
        current_key = came_from[current_key]

    # Reserve the start point
    reservation_table[(start.row, start.col, 0)] = robot_id  
    path.append(start.get_pos())
    path.reverse()

    return path


#the bread and butter of the project
def algorithm(draw, grid, start, end, robot_id, existing_paths, max_wait=3, wait_penalty=100):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, 0, start))  # (f_score, count, time, node)
    came_from = {}
    g_score = {(start, 0): 0}
    f_score = {(start, 0): h(start.get_pos(), end.get_pos())}

    open_set_hash = {(start, 0)}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        _, _, timen, current = open_set.get()
        open_set_hash.remove((current, timen))

        if current == end:
            # Reserve the path
            current_time = timen
            while (current, current_time) in came_from:
                reservation_table[(current.row, current.col, current_time)] = robot_id
                current, current_time = came_from[(current, current_time)]
            reservation_table[(start.row, start.col, 0)] = robot_id

            path_nodes = reconstruct_time_path(came_from, end, start, robot_id)
            existing_paths[robot_id] = path_nodes
            path_directions = convert_path_to_directions(path_nodes)

            print(f"Robot {robot_id + 1} path directions:", path_directions)
            send_to_arduino(robot_id, path_directions)

            for node in path_nodes:
                row, col = node
                grid[row][col].make_path(robot_id)
                draw()

            end.make_end()
            time.sleep(3)
            
            print(f"[Robot {robot_id+1}] Reached goal at time {timen}")
            
            # Reserve the goal position for future time steps to avoid other robots colliding into it
            for future_time in range(timen + 1, timen + 20):  # Reserve for 20 steps
                reservation_table[(end.row, end.col, future_time)] = robot_id

            return True
        
        


        for neighbor in current.neighbors:
            next_time = timen + 1
            key = (neighbor, next_time)

            if neighbor.is_barrier():
                continue

            # Prevent head-on swap
            prev_pos = (neighbor.row, neighbor.col, timen)
            next_pos = (current.row, current.col, next_time)
            if reservation_table.get(prev_pos) not in (None, robot_id) and \
               reservation_table.get(next_pos) not in (None, robot_id):
                continue

            # 🚫 Strict reservation collision prevention
            if reservation_table.get((neighbor.row, neighbor.col, next_time)) not in (None, robot_id):
                continue  # No penalty, just skip (forces reroute)

            # 🧠 Only allow movement if cell is not already blocked
            temp_g_score = g_score[(current, timen)] + 1

            if key not in g_score or temp_g_score < g_score[key]:
                came_from[key] = (current, timen)
                g_score[key] = temp_g_score
                f_score[key] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if key not in open_set_hash:
                    count += 1
                    open_set.put((f_score[key], count, next_time, neighbor))
                    open_set_hash.add(key)
                    neighbor.make_open()



        draw()
        if current != start:
            current.make_closed()

    print(f"[Robot {robot_id}] No path found.")
    return False



def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)

    return grid 

    
def draw_grid(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win, rows, width)
    pygame.display.update()


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos
    
    row = y // gap
    col = x // gap

    return row, col

# testing direction conversion in string
def convert_path_to_directions(path):
    # Same direction logic as before
    abs_directions = []
    for i in range(1, len(path)):
        prev_row, prev_col = path[i - 1]
        curr_row, curr_col = path[i]

        d_row = curr_row - prev_row
        d_col = curr_col - prev_col

        if d_col == -1:
            abs_directions.append('U')  # grid left
        elif d_col == 1:
            abs_directions.append('B')  # grid right
        elif d_row == -1:
            abs_directions.append('L')  # grid up
        elif d_row == 1:
            abs_directions.append('R')  # grid down

    facing_order = ['U', 'R', 'B', 'L']
    relative_moves = []
    current_facing = 'U'

    for abs_dir in abs_directions:
        if abs_dir == current_facing:
            relative_moves.append('U')
        else:
            i_face = facing_order.index(current_facing)
            i_abs = facing_order.index(abs_dir)
            turn = (i_abs - i_face) % 4

            if turn == 1:
                relative_moves.append('R')
            elif turn == 2:
                relative_moves.append('B')
            elif turn == 3:
                relative_moves.append('L')

            current_facing = abs_dir

    if relative_moves:
        relative_moves[0] = abs_directions[0]

    return ''.join(relative_moves)



#test 2
pending_executions = []

def send_to_arduino(robot_id, directions_string):
    ip = robots[robot_id]['ip']
    port = 80
    try:
        # Send direction
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5)
            s.connect((ip, port))
            message = f"ID:{robot_id + 1}:{directions_string}"
            s.sendall((message + '\n').encode())
            response = s.recv(1024).decode().strip()
            print(f"[Robot {robot_id + 1}] Direction Response: {response}")
            if "DIRECTION_OK" not in response:
                return

        # Ask for calibration
        calib = input(f"[Robot {robot_id + 1}] Run calibration? (y/n): ").lower()
        if calib == 'y':
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5)
                s.connect((ip, port))
                s.sendall(b"CALIBRATE\n")
                response = s.recv(1024).decode().strip()
                print(f"[Robot {robot_id + 1}] Calibration Response: {response}")
                if "CALIBRATION_DONE" not in response:
                    return

        # Add to pending execution list
        pending_executions.append((robot_id, ip))

    except Exception as e:
        print("")
        #print(f"[Robot {robot_id + 1}] Error: {e}")


def execute_all_pending():
    confirm = input("\nReady to start execution for all robots? (y/n): ").lower()
    if confirm != 'y':
        print("Execution aborted.")
        return

    for robot_id, ip in pending_executions:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5)
                s.connect((ip, 80))
                s.sendall(b"EXECUTE\n")
                response = s.recv(1024).decode().strip()
                print(f"[Robot {robot_id + 1}] Execution Response: {response}")
        except Exception as e:
            print(f"[Robot {robot_id + 1}] Execution Error: {e}")



def main(win, width):
    ROWS = 50
    grid = make_grid(ROWS, width)

    current_robot = 0  # Start with robot 0
    run = True

    while run:
        draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]: #left mouse button
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]

                # Set start/end for current robot
                if not robots[current_robot]['start'] and spot != robots[current_robot]['end']:
                    robots[current_robot]['start'] = spot
                    spot.make_start(current_robot)
                
                elif not robots[current_robot]['end'] and spot != robots[current_robot]['start']:
                    robots[current_robot]['end'] = spot
                    spot.make_end()
                
                elif spot != robots[current_robot]['end'] and spot != robots[current_robot]['start']:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]: #right mouse button
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                spot.reset()

                # Clear start/end if clicked
                for robot in robots:
                    if spot == robot['start']:
                        robot['start'] = None
                    if spot == robot['end']:
                        robot['end'] = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    # Check if all robots have start/end points
                    all_ready = all(robot['start'] and robot['end'] for robot in robots)
                    if all_ready:
                        reservation_table.clear() #new

                        for row in grid:
                            for spot in row:
                                spot.update_neighbors(grid)

                        # Run pathfinding for each robot
                        pending_executions.clear()
                        existing_paths = {}

                        for robot in sorted(robots, key=lambda r: r['id']): #new
                            algorithm(lambda: draw(win, grid, ROWS, width), 
                                     grid, robot['start'], robot['end'], robot['id'], existing_paths)
                        execute_all_pending()
                
                # Switch between robots with number keys
                if event.key == pygame.K_1:
                    current_robot = 0
                    print("Now placing Robot 1 (Orange)")

                if event.key == pygame.K_2:
                    current_robot = 1
                    print("Now placing Robot 2 (Blue)")

                if event.key == pygame.K_c:
                    # Clear everything
                    for robot in robots:
                        robot['start'] = None
                        robot['end'] = None
                    reservation_table.clear()
                    grid = make_grid(ROWS, width)


    pygame.quit()

main(WIN, WIDTH)