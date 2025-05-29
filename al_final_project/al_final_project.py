import pygame
import threading
import heapq
import math
import os
import time
import random
from gui.menu import Menu

# Giá trị pin tối đa
WIDTH, HEIGHT = 1000, 600
MAP_RATIO = 0.8
pin_max = 100.0
FPS = 60  # Tăng FPS để mượt hơn
SPEED = 0.5  # Tốc độ di chuyển (pixel/giây)
WAIT_TIME_STATION = 2.0  # Thời gian chờ tại trạm sạc (giây)
MAX_PATH_LINES = 15 
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Drone Pathfinding Visualizer")
font = pygame.font.SysFont(None, 28)
# Lớp đại diện cho mỗi ô trong lưới
class Cell:
    def __init__(self):
        self.parent_i = 0
        self.parent_j = 0
        self.f = float('inf')  # tổng chi phí f = g + h
        self.g = float('inf')  # chi phí từ điểm bắt đầu đến ô hiện tại
        self.h = 0  # ước lượng chi phí từ ô hiện tại đến đích
        self.battery = pin_max  # mức pin còn lại
        self.total_cost = 0
        self.height = 0

# Kiểm tra xem chỉ số ô có nằm trong lưới không
def is_valid(row, col, ROW, COL):
    return 0 <= row < ROW and 0 <= col < COL

# Kiểm tra ô có phải điểm đích không
def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Tính heuristic (h) theo khoảng cách Euclidean, đơn vị là mm
def calculate_h_value(row, col, dest, curr_height, dest_height):
    return math.hypot(100.0 * (dest_height - curr_height), math.hypot(400.0 * (row - dest[0]), 400.0 * (col - dest[1])))
class PathFinder(threading.Thread):
    def __init__(self, grid, src, dest, callback):
        super().__init__()
        self.grid = grid
        self.src = src
        self.dest = dest
        self.callback = callback
        self.ROW = len(grid)
        self.COL = len(grid[0])
        self.running = True
    def run(self):
        path = self.a_star_search()
        self.callback(path)
    # Truy vết đường đi từ đích về đầu, tính tổng chi phí di chuyển
    def trace_path(self, cell_details, dest):
        grid = self.grid
        path = []
        row, col = dest
        final_total_cost = 0

        # Lặp lại cho đến khi về đến điểm bắt đầu
        while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
            path.append((row, col, cell_details[row][col].height, cell_details[row][col].battery))
            temp_row = cell_details[row][col].parent_i
            temp_col = cell_details[row][col].parent_j
            row, col = temp_row, temp_col

        # Thêm điểm bắt đầu
        path.append((row, col, cell_details[row][col].height, cell_details[row][col].battery))
        path.reverse()
        path_with_cost = []
        for idx in range(len(path)):
            if idx == 0:
                path_with_cost.append((*path[idx], 0.0))
            else:
                r1, c1, *_ = path[idx]
                final_total_cost += cell_details[r1][c1].total_cost
                path_with_cost.append((*path[idx], round(final_total_cost, 2)))
        with open("output.txt", "w") as f:
            f.write("The Path is:\n")
            for i in path_with_cost:
                f.write(f" -> {i}\n")
            f.write(f"\nTotal cost of the Path is: {final_total_cost:.2f}\n")
        return path_with_cost
    # Thuật toán A* tìm đường có xét đến pin và địa hình
    def a_star_search(self):
        grid = self.grid
        src, dest = self.src, self.dest
        ROW, COL = self.ROW, self.COL
        if not is_valid(src[0], src[1], ROW, COL) or not is_valid(dest[0], dest[1], ROW, COL):
            print("Source or destination is invalid")
            return

        closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
        cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

        i, j = src
        cell_details[i][j].f = 0.0
        cell_details[i][j].g = 0.0
        cell_details[i][j].h = 0.0
        cell_details[i][j].battery = pin_max
        cell_details[i][j].parent_i = i
        cell_details[i][j].parent_j = j
        open_list = []
        heapq.heappush(open_list, (0.0, i, j, pin_max))
        found_dest = False

        directions = [(0, 1), (1, 0), (1, 1), (-1, 0),
                    (0, -1), (1, -1), (-1, 1), (-1, -1)]

        # Tìm tất cả điểm 0 (điểm sạc)
        list_of_zero_cells = [(r, c) for r in range(ROW) for c in range(COL) if grid[r][c] == 0]

        while open_list and self.running:
            f_val, i, j, battery = heapq.heappop(open_list)

            if closed_list[i][j]:
                continue
            closed_list[i][j] = True

            for dir in directions:
                new_i, new_j = i + dir[0], j + dir[1]

                if not is_valid(new_i, new_j, ROW, COL):
                    continue
                if closed_list[new_i][new_j]:
                    continue

                # Tính chi phí di chuyển: chéo hoặc thẳng
                move_cost = 400.0 * math.sqrt(2) if dir[0] != 0 and dir[1] != 0 else 400.0

                # Lấy độ cao (chi phí) ô hiện tại và ô mới
                curr_cell_cost = 0.0 if grid[i][j] == -1 else grid[i][j]
                new_cell_cost = 0.0 if grid[new_i][new_j] == -1 else grid[new_i][new_j]

                # Tính chi phí pin và điều chỉnh nếu lên/xuống dốc
                if curr_cell_cost == new_cell_cost:
                    battery_cost = 0.02 * move_cost
                    new_battery = battery - battery_cost
                elif curr_cell_cost > new_cell_cost:
                    movedown_cost = (curr_cell_cost - new_cell_cost) * 100.0
                    battery_cost = 0.02 * (move_cost + movedown_cost * 0.1)
                    move_cost += movedown_cost
                    new_battery = battery - battery_cost
                else:
                    moveup_cost = (new_cell_cost - curr_cell_cost) * 100.0
                    battery_cost = 0.02 * (move_cost + moveup_cost * 1.25)
                    move_cost += moveup_cost
                new_battery = battery - battery_cost
                if new_battery <= 0.0:
                    continue  # Bỏ qua nếu pin không đủ

                if new_cell_cost == 0.0 and not is_destination(new_i, new_j, dest):
                    new_battery = pin_max


                if is_destination(new_i, new_j, dest):
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    cell_details[new_i][new_j].total_cost = round(move_cost, 2)
                    found_dest = True
                    return self.trace_path(cell_details, dest)

                g_new = cell_details[i][j].g + move_cost

                # Tính h mới: nếu gần ô sạc hơn thì ưu tiên
                h_to_closest_zero = min(calculate_h_value(new_i, new_j, zero_cell, new_cell_cost, 0) for zero_cell in list_of_zero_cells)
                h_to_dest = calculate_h_value(new_i, new_j, dest, new_cell_cost, 0)
                #h_new = min(h_to_closest_zero, h_to_dest)

                # Nếu không đủ pin tới đích thì đi tìm trạm
                if new_battery < 0.02 * h_to_dest:
                    h_new = h_to_closest_zero
                else:
                    h_new = h_to_dest
                
                # Nếu có trạm gần đó thì tới đó luôn, tránh trường hợp gần đến đích lại phải đi tìm trạm
                if h_to_closest_zero < h_to_dest:
                    h_new = h_to_closest_zero
                else:
                    h_new = h_to_dest
                # Ưu tiên các ô có chi phí thấp hơn bằng cách giảm f theo priority_bias
                #priority_bias = 10.0 / (1 + new_cell_cost)
            
                f_new = g_new + h_new # - - priority_bias
                # Nếu ô mới tốt hơn (f nhỏ hơn), cập nhật thông tin
                if cell_details[new_i][new_j].f > f_new:
                    heapq.heappush(open_list, (f_new, new_i, new_j, new_battery))
                    cell_details[new_i][new_j].f = f_new
                    cell_details[new_i][new_j].g = g_new
                    cell_details[new_i][new_j].h = h_new
                    cell_details[new_i][new_j].battery = round(new_battery, 2)
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    cell_details[new_i][new_j].total_cost = round(move_cost, 2)
                    cell_details[new_i][new_j].height = grid[new_i][new_j]*100

        if not found_dest:
            # Tìm trạm sạc gần nhất đã duyệt được
            best_station = None
            min_h = float('inf')
            for i in range(ROW):
                for j in range(COL):
                    if closed_list[i][j] and grid[i][j] == 0:
                        h = calculate_h_value(i, j, dest, 0, 0)
                        if h < min_h:
                            min_h = h
                            best_station = (i, j)
            if best_station:
                print("Không đủ pin đến đích, dừng ở trạm sạc gần nhất:", best_station)
                return self.trace_path(cell_details, best_station)
            else:
                with open("output.txt", "w") as f:
                    f.write("Cannot find the destination cell\n")
            return []





# Hàm chính: đọc dữ liệu từ file và gọi thuật toán
def draw_map(grid, path, current_idx, src, dest, drone_progress=0.0, is_waiting=False, zoom_out=False, fade_index=0, paused=False, scroll_offset=0):
    ROW, COL = len(grid), len(grid[0])
    map_width = int(WIDTH * MAP_RATIO)
    cell_size = map_width // COL

    wall_img = pygame.image.load("assets/wall.png")
    ground_img = pygame.image.load("assets/ground.png")
    station_img = pygame.image.load("assets/station.png")
    start_img = pygame.image.load("assets/start_end.png")
    end_img = pygame.image.load("assets/start_end.png")
    drone_img = pygame.image.load("assets/drone.png")

    passed_cells = set((row, col) for row, col, *_ in path[:current_idx+1])
    offset = pygame.Vector2()
    if zoom_out:
        # Tính cell_size nhỏ nhất để vừa toàn bộ map vào màn hình
        map_width = int(WIDTH * 0.7)
        cell_size = min(map_width // COL, HEIGHT // ROW)
        offset.x = (WIDTH*MAP_RATIO - COL * cell_size) // 2
        offset.y = (HEIGHT - ROW * cell_size) // 2
    else:
        map_width = int(WIDTH * MAP_RATIO)
        cell_size = map_width // COL
        center_x = WIDTH * MAP_RATIO // 2
        center_y = HEIGHT // 2
        if path and current_idx < len(path):
            drone_pos = path[current_idx][:2]
        else:
            drone_pos = src
        offset.x = center_x - (drone_pos[1] * cell_size + cell_size // 3)
        offset.y = center_y - (drone_pos[0] * cell_size + cell_size // 3)
        max_x = 0
        max_y = 0
        min_x = WIDTH * MAP_RATIO - COL * cell_size
        min_y = HEIGHT - ROW * cell_size
        offset.x = max(min(offset.x, max_x), min_x)
        offset.y = max(min(offset.y, max_y), min_y)
    for i in range(ROW):
        for j in range(COL):
            x = j * cell_size
            y = i * cell_size
            val = grid[i][j]
            if (i, j) == src:
                img = pygame.transform.scale(start_img, (cell_size, cell_size))
            elif (i, j) == dest:
                img = pygame.transform.scale(end_img, (cell_size, cell_size))
            elif val == 0:
                img = pygame.transform.scale(station_img, (cell_size, cell_size))
            elif val == 1:
                img = pygame.transform.scale(ground_img, (cell_size, cell_size))
            else:
                img = pygame.transform.scale(wall_img, (cell_size, cell_size))
            screen.blit(img, (x+offset.x, y+offset.y))
            
            if (i, j) in passed_cells:
                if zoom_out:
                    # Tìm vị trí của ô này trong path
                    try:
                        idx = [k for k, p in enumerate(path) if p[0] == i and p[1] == j][0]
                    except IndexError:
                        idx = -1
                    if 0 <= idx < fade_index:
                        # Đã đến lượt fade sang xanh
                        green_surface = pygame.Surface((cell_size, cell_size), pygame.SRCALPHA)
                        green_surface.fill((0, 255, 100, 150))  # Xanh nhạt, alpha 120
                        screen.blit(green_surface, (x+offset.x, y+offset.y))
                    else:
                        # Chưa đến lượt, vẫn đen nhạt
                        black_surface = pygame.Surface((cell_size, cell_size), pygame.SRCALPHA)
                        black_surface.fill((0, 0, 0, 150))
                        screen.blit(black_surface, (x+offset.x, y+offset.y))
                else:
                    black_surface = pygame.Surface((cell_size, cell_size), pygame.SRCALPHA)
                    black_surface.fill((0, 0, 0, 150))
                    screen.blit(black_surface, (x+offset.x, y+offset.y))
    # Vẽ drone (nếu chưa đến đích)
    # if not zoom_out:
        # Tính vị trí drone mượt giữa 2 ô
    angle = 0
    if path and current_idx < len(path) - 1:
        (r1, c1, *_), (r2, c2, *_) = path[current_idx], path[current_idx+1]
        drone_row = r1 + (r2 - r1) * drone_progress
        drone_col = c1 + (c2 - c1) * drone_progress
        # Tính góc hướng di chuyển (theo radian)
        dx = c2 - c1
        dy = r2 - r1
        angle = math.degrees(math.atan2(-dy, dx))  # Đảo dấu dy vì pygame y-axis ngược
    elif path:
        drone_row, drone_col = path[-1][0], path[-1][1]
    else:
        drone_row, drone_col = src
    drone_x = drone_col * cell_size
    drone_y = drone_row * cell_size
    # Hiệu ứng drone khi sạc
    drone_scaled = pygame.transform.scale(drone_img, (cell_size, cell_size))
    drone_rotated = pygame.transform.rotate(drone_scaled, angle)
    if is_waiting:
        # Hiệu ứng rung nhẹ khi sạc
        shake = math.sin(time.time() * 10) * 2  # Rung với tần số 10 Hz, biên độ 2 pixel
        drone_rect = pygame.Rect(drone_x + offset.x + shake, drone_y + offset.y + shake, cell_size, cell_size)
        drone_rotated = drone_scaled
    else:
        drone_rect = drone_rotated.get_rect(center=(drone_x + offset.x + cell_size // 2, drone_y + offset.y + cell_size // 2))
    screen.blit(drone_rotated, drone_rect)


    # Giao diện bên phải
    pygame.draw.rect(screen, (0, 0, 0), (map_width, 0, WIDTH - map_width, HEIGHT))
    if not zoom_out:
        r, c, val, battery, cost = path[current_idx]
        # Độ cao là giá trị ô hiện tại (val)
        altitude = val if isinstance(val, (int, float)) else 0
        # Chọn màu theo độ cao
        altitude_color = (0, 200, 0) if altitude <= 100 else (220, 0, 0)
        draw_text(f"Battery: {battery:.2f}%", map_width + 20, 40)
        draw_text(f"Cost: {cost}", map_width + 20, 80)
        draw_text(f"Position: ({r}, {c})", map_width + 20, 120)
        draw_text(f"Altitude: {altitude}", map_width + 20, 160, altitude_color)
        if is_waiting:
            draw_text("Charging...", map_width + 20, 200)
        else:
            draw_text("Moving...", map_width + 20, 200)
        pause_rect = pygame.Rect(map_width + 5, HEIGHT - 100, 90, 40)
        menu_rect = pygame.Rect(map_width + 105, HEIGHT - 100, 90, 40)
        pygame.draw.rect(screen, (100, 100, 255), pause_rect)
        pygame.draw.rect(screen, (255, 180, 60), menu_rect)
        draw_text("Pause" if not paused else "Resume", pause_rect.x + 12, pause_rect.y + 8, (0,0,0))
        draw_text("Menu", menu_rect.x + 20, menu_rect.y + 8, (0,0,0))
    else:
        menu_rect = pygame.Rect(map_width + ((map_width*0.3)//2), HEIGHT - 100, 120, 40)
        pygame.draw.rect(screen, (255, 180, 60), menu_rect)
        draw_text("Menu", menu_rect.x + 30, menu_rect.y + 8, (0,0,0))
        draw_text("Path:", map_width + 20, 40)
        total_lines = fade_index
        # Chỉ vẽ các dòng trong khoảng scroll_offset
        for idx in range(scroll_offset, min(scroll_offset + MAX_PATH_LINES, total_lines)):
            if idx < len(path):
                r, c, h, *_ = path[idx]
                draw_text(f"{idx+1}: (x: {r}, y: {c}, height: {h})", map_width + 20, 60 + (idx - scroll_offset)*25, (0,255,100))
        # Vẽ thanh trượt nếu cần
        if total_lines > MAX_PATH_LINES:
            bar_height = int(MAX_PATH_LINES / total_lines * 375)
            bar_y = 60 + int(scroll_offset / total_lines * 375)
            bar_rect = pygame.Rect(WIDTH-30, bar_y, 10, bar_height)
            pygame.draw.rect(screen, (180,180,180), (WIDTH-30, 60, 10, 375))
            pygame.draw.rect(screen, (100,100,255), bar_rect)
        
def draw_text(text, x, y, color=(255,140,0)):
    img = font.render(text, True, color)
    screen.blit(img, (x, y))
def main():
    # choose = input("Choose map (1-3): ").strip()

    # input_folder = "input"
    
    # # Chuyển map chọn thành index tương ứng với dãy file
    # try:
    #     map_number = int(choose)
    #     if map_number not in [1, 2, 3]:
    #         raise ValueError
    # except ValueError:
    #     print("Please select a number from 1 to 3")
    #     return
    # map_number = int(choose)
    # # Tính toán chỉ số bắt đầu và kết thúc của các file input tương ứng
    # start_index = (map_number - 1) * 4 + 1
    # end_index = map_number * 4

    # # Code chọn file ngẫu nhiên
    # # Tạo danh sách tên file phù hợp
    # input_files = [f"input{i}.txt" for i in range(start_index, end_index + 1)]
    # input_files = [f for f in input_files if os.path.exists(os.path.join(input_folder, f))]

    # # if not input_files:
    # #     print("No matching input file found.")
    # #     return

    # # # Chọn file ngẫu nhiên
    # filename = random.choice(input_files)

    # '''
    # # Code chọn file từ bàn phím
    # # filename = f"input{choose}.txt"
    # # input_path = os.path.join(input_folder, filename)

    # # if not os.path.exists(input_path):
    # #     print(f"File {filename} does not exist.")
    # #     return
    # '''
    
    # input_path = os.path.join(input_folder, filename)
    # print(f"\n--- Processing file: {filename} ---")

    CELL_SIZE=100
    path = []
    done = [False]


    clock = pygame.time.Clock()
    
    step = 0
    drone_progress = 0.0
    is_waiting = False
    wait_time = 0.0

    running = True
    zoom_out = False
    fade_index = 0
    fade_progress = 0.0
    FADE_TIME_PER_CELL = 1  # Thời gian đổi màu mỗi ô (giây)
    paused = False
    scroll_offset = 0 
    global in_menu, selected_map
    in_menu = True
    selected_map = None
    def on_choose_map(filename):
        global in_menu, selected_map
        selected_map = filename
        in_menu = False
        
    def on_done(p):
        print(f"Path found: {p}")
        path.extend(p)
        done[0] = True
    while running:
        delta_time = clock.tick(FPS) / 1000.0
        screen.fill((0, 0, 0))
        # Define map_width for button position calculations
        
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if not zoom_out:
                    map_width = int(WIDTH * MAP_RATIO)
                    if map_width + 5 <= mx <= map_width + 95 and HEIGHT - 100 <= my <= HEIGHT - 60:
                        paused = not paused  # Đảo trạng thái pause
                    if map_width + 105 <= mx <= map_width + 195 and HEIGHT - 100 <= my <= HEIGHT - 60:
                        in_menu = True  # Quay lại menu
                else:
                    if event.button == 4:  # Lăn chuột lên
                        scroll_offset = max(0, scroll_offset - 1)
                    elif event.button == 5:  # Lăn chuột xuống
                        max_offset = max(0, fade_index - MAX_PATH_LINES)
                        scroll_offset = min(max_offset, scroll_offset + 1)
                    map_width = int(WIDTH * 0.7)
                    if map_width + ((map_width*0.3)//2) <= mx <= map_width + (WIDTH*0.3//2) +120 and HEIGHT - 100 <= my <= HEIGHT - 60:
                        zoom_out = False
                        is_waiting = False
                        pause = False
                        in_menu = True  # Quay lại menu
            if event.type == pygame.QUIT:
                pf.running = False
                running = False
        if in_menu:
            menu = Menu(screen, on_choose_map=on_choose_map)
            menu.run()
        elif not in_menu and selected_map:
            # Đọc file map
            print(selected_map)
            input_folder = os.path.join(os.path.dirname(__file__), "input")
            input_path = os.path.join(input_folder, selected_map)
            with open(input_path, "r") as f:
                lines = [line.strip() for line in f.readlines()]

            ROW = int(lines[0])
            grid = []
            src = dest = None

            for i in range(1, ROW + 1):
                row = []
                tokens = lines[i].split()
                for j, token in enumerate(tokens):
                    if token == 'A':
                        src = (i - 1, j)
                        row.append(-1)
                    elif token == 'B':
                        dest = (i - 1, j)
                        row.append(-1)
                    else:
                        row.append(int(token))
                grid.append(row)

            if src is None or dest is None:
                print("Error: Start (A) or Destination (B) does not exist in file")
                in_menu = True
                continue

            # Reset các biến liên quan đến thuật toán
            path.clear()
            done[0] = False
            step = 0
            drone_progress = 0.0
            is_waiting = False
            wait_time = 0.0
            zoom_out = False
            fade_index = 0
            fade_progress = 0.0

            # Khởi động thuật toán tìm đường
            pf = PathFinder(grid, src, dest, on_done)
            pf.start()

            # Đặt lại selected_map để không load lại nhiều lần
            selected_map = None
        else:
            if not paused:
                if done[0] and path and step < len(path) - 1:
                    row, col, *_ = path[step]
                    # Nếu đang ở trạm sạc và chuẩn bị bắt đầu di chuyển
                    if grid[row][col] == 0 and drone_progress == 0.0:
                        if not is_waiting:
                            is_waiting = True
                            wait_time = 0.0
                        if is_waiting:
                            wait_time += delta_time
                            if wait_time >= WAIT_TIME_STATION:
                                is_waiting = False  # Hết chờ, bắt đầu di chuyển
                                drone_progress += SPEED * delta_time
                        else:
                            print("ok")
                            drone_progress += SPEED * delta_time
                            if drone_progress >= 1.0:
                                drone_progress = 0.0
                                step += 1
                    else:
                        # Các ô thường hoặc đang di chuyển giữa các ô
                        drone_progress += SPEED * delta_time
                        if drone_progress >= 1.0:
                            drone_progress = 0.0
                            step += 1
                    if done[0] and path and step >= len(path) - 1:
                        zoom_out = True
            if zoom_out:
                if fade_index < len(path):
                    fade_progress += delta_time
                    if fade_progress >= FADE_TIME_PER_CELL:
                        fade_progress = 0.0
                        fade_index += 1
            draw_map(grid, path, step, src, dest, drone_progress, is_waiting, zoom_out, fade_index if zoom_out else 0, paused, scroll_offset=scroll_offset)
            pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()