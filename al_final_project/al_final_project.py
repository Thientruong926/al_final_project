import math
import heapq
import os
import random

pin_max = 100.0
s = set() #lưu vị trí trạm

class Cell:
    def __init__(self):
        self.parent_i = 0
        self.parent_j = 0
        self.f = float('inf')
        self.g = float('inf')
        self.h = 0
        self.battery = pin_max
        self.total_cost = 0

def init(grid, ROW, COL):
    for i in range(ROW):
        for j in range(COL):
            if grid[i][j] == 0:
                s.add((i, j))

def is_valid(row, col, ROW, COL): #kiểm tra ô có nằm trong lưới không
    return 0 <= row < ROW and 0 <= col < COL

def is_destination(row, col, dest): #ô có phải đích không
    return row == dest[0] and col == dest[1]

def calculate_h_value(row, col, height, dest): #ước lượng khoảng cách đến đích bằng công thức Euclidean
    return math.hypot(100*height,(math.hypot(400*(row - dest[0]), 400*(col - dest[1]))))

def trace_path(cell_details, dest, grid): #truy vết đường đi từ đích về đầu bằng cách lần ngược theo parent_i, parent_j, và in ra chi phí
    path = []
    row, col = dest
    final_total_cost = 0

    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col, cell_details[row][col].battery, cell_details[row][col].total_cost, cell_details[row][col].h))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        final_total_cost += cell_details[row][col].total_cost
        row, col = temp_row, temp_col

    path.append((row, col, cell_details[row][col].battery, cell_details[row][col].total_cost, cell_details[row][col].h))
    path.reverse()

    print("The Path is:")
    for i in path:
        print(f" -> {i}")
    print(f"\nTotal cost of the Path is: {final_total_cost:.2f}\n")

# A* Search
def a_star_search(grid, src, dest, ROW, COL):
    if not is_valid(src[0], src[1], ROW, COL) or not is_valid(dest[0], dest[1], ROW, COL):
        print("Source or destination is invalid")
        return

    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    i, j = src
    cell_details[i][j].g = 0.0
    cell_details[i][j].h = calculate_h_value(i, j, 0, dest)
    cell_details[i][j].f = cell_details[i][j].g + cell_details[i][j].h
    cell_details[i][j].battery = pin_max
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    open_list = []
    heapq.heappush(open_list, (0.0, i, j, pin_max))
    found_dest = False

    directions = [(0, 1), (1, 0), (1, 1), (-1, 0),
                  (0, -1), (1, -1), (-1, 1), (-1, -1)]

    while open_list:
        f_val, i, j, battery = heapq.heappop(open_list)
        
        if closed_list[i][j]:
            continue
        closed_list[i][j] = True
        estimated_battery_to_dest = 0.02 * cell_details[i][j].h

        #code tìm trạm từ vị trí đang xét
        while cell_details[i][j].battery < estimated_battery_to_dest:
            print(f"Battery from ({i},{j}) not enough to reach destination. Seeking charging station.")
            if not s:
                print("No charging stations available.")
                trace_path(cell_details, (i, j), grid)
                return
            
            stations.sort(key=lambda pos: math.hypot(grid[i][j] * 100, math.hypot(400 * (pos[0]-i), 400 * (pos[1]-j))))
            nearest_station = stations[0]
            print(f"Redirecting to charging station at {nearest_station}")
            result = a_star_search_station(grid, (i, j), nearest_station, ROW, COL, cell_details)
            if result is not None:
                i, j = result
                print(cell_details[i][j].battery)
                estimated_battery_to_dest = 0.02 * cell_details[i][j].h
            else:
                trace_path(cell_details, dest, grid)
                print("drone can't find station.")
                return

        for dir in directions:
            new_i, new_j = i + dir[0], j + dir[1]

            if not is_valid(new_i, new_j, ROW, COL):
                continue

            move_cost = round(400*math.sqrt(2), 2) if dir[0] != 0 and dir[1] != 0 else 400

            curr_cell_cost = 0 if grid[i][j] == -1 else grid[i][j]
            new_cell_cost = 0 if grid[new_i][new_j] == -1 else grid[new_i][new_j]

            if curr_cell_cost == new_cell_cost:
                battery_cost = 0.02*move_cost
                new_battery = battery - battery_cost
            elif curr_cell_cost>new_cell_cost:
                movedown_cost = (curr_cell_cost - new_cell_cost) * 100
                battery_cost = 0.02*(move_cost + movedown_cost*0.1)
                move_cost += movedown_cost
                new_battery = battery - battery_cost
            else:
                moveup_cost = (new_cell_cost - curr_cell_cost) * 100
                battery_cost = 0.02*(move_cost + moveup_cost*1.25)
                move_cost += moveup_cost
                new_battery = battery - battery_cost

            # chưa xét trường hợp pin <= 0
            if new_battery <= 0:
                print("The drone ran out of battery.")
                trace_path(cell_details, (i, j), grid)
                return

            # trên đường đi, drone vô tình đi vô trạm
            if new_cell_cost == 0 and not is_destination(new_i, new_j, dest):
                new_battery = pin_max


            if is_destination(new_i, new_j, dest):
                cell_details[new_i][new_j].parent_i = i
                cell_details[new_i][new_j].parent_j = j
                cell_details[new_i][new_j].battery = round(new_battery, 2)
                cell_details[new_i][new_j].total_cost = move_cost
                trace_path(cell_details, dest, grid)
                found_dest = True
                return

            h_new = calculate_h_value(new_i, new_j, new_cell_cost, dest)
            g_new = cell_details[i][j].g + move_cost
            f_new = g_new + h_new
            if cell_details[new_i][new_j].f > f_new:
                heapq.heappush(open_list, (f_new, new_i, new_j, round(new_battery, 2)))
                cell_details[new_i][new_j].f = f_new
                cell_details[new_i][new_j].g = g_new
                cell_details[new_i][new_j].h = round(h_new, 2)
                cell_details[new_i][new_j].battery = round(new_battery, 2)
                cell_details[new_i][new_j].parent_i = i
                cell_details[new_i][new_j].parent_j = j
                cell_details[new_i][new_j].total_cost = move_cost

    if not found_dest:
        print("Cannot find the destination cell\n")

#code tìm trạm
def a_star_search_station(grid, src, dest, ROW, COL, cell_details):
    if not is_valid(src[0], src[1], ROW, COL) or not is_valid(dest[0], dest[1], ROW, COL):
        print("Source or station is invalid")
        return

    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]

    i, j = src
    cell_details[i][j].g = 0.0
    cell_details[i][j].h = calculate_h_value(i, j, 0, dest)
    cell_details[i][j].f = cell_details[i][j].g + cell_details[i][j].h

    open_list = []
    heapq.heappush(open_list, (0.0, i, j, pin_max))

    directions = [(0, 1), (1, 0), (1, 1), (-1, 0),
                  (0, -1), (1, -1), (-1, 1), (-1, -1)]

    while open_list:

        f_val, i, j, battery = heapq.heappop(open_list)

        if closed_list[i][j]:
            continue
        closed_list[i][j] = True

        for dir in directions:
            new_i, new_j = i + dir[0], j + dir[1]

            if not is_valid(new_i, new_j, ROW, COL):
                continue

            move_cost = round(400*math.sqrt(2), 2) if dir[0] != 0 and dir[1] != 0 else 400

            curr_cell_cost = 0 if grid[i][j] == -1 else grid[i][j]
            new_cell_cost = 0 if grid[new_i][new_j] == -1 else grid[new_i][new_j]

            if curr_cell_cost == new_cell_cost:
                battery_cost = 0.02*move_cost
                new_battery = battery - battery_cost
            elif curr_cell_cost>new_cell_cost:
                movedown_cost = (curr_cell_cost - new_cell_cost) * 100
                battery_cost = 0.02*(move_cost + movedown_cost*0.1)
                move_cost += movedown_cost
                new_battery = battery - battery_cost
            else:
                moveup_cost = (new_cell_cost - curr_cell_cost) * 100
                battery_cost = 0.02*(move_cost + moveup_cost*1.25)
                move_cost += moveup_cost
                new_battery = battery - battery_cost

            if new_battery <= 0:
                return

            h_new = calculate_h_value(new_i, new_j, new_cell_cost, dest)
            g_new = cell_details[i][j].g + move_cost
            f_new = g_new + h_new

            if is_destination(new_i, new_j, dest):
                cell_details[new_i][new_j].parent_i = i
                cell_details[new_i][new_j].parent_j = j
                cell_details[new_i][new_j].battery = pin_max
                cell_details[new_i][new_j].total_cost = move_cost
                cell_details[new_i][new_j].f = f_new
                cell_details[new_i][new_j].g = g_new
                cell_details[new_i][new_j].h = round(h_new, 2)
                return (new_i, new_j)

            if cell_details[new_i][new_j].f > f_new:
                heapq.heappush(open_list, (f_new, new_i, new_j, round(new_battery, 2)))
                cell_details[new_i][new_j].f = f_new
                cell_details[new_i][new_j].g = g_new
                cell_details[new_i][new_j].h = round(h_new, 2)
                cell_details[new_i][new_j].battery = round(new_battery, 2)
                cell_details[new_i][new_j].parent_i = i
                cell_details[new_i][new_j].parent_j = j
                cell_details[new_i][new_j].total_cost = move_cost

def main():
    choose = input("Choose map (1-3): ").strip()

    input_folder = "input"
    
    # Chuyển map chọn thành index tương ứng với dãy file
    '''
    try:
        map_number = int(choose)
        if map_number not in [1, 2, 3]:
            raise ValueError
    except ValueError:
        print("Please select a number from 1 to 3")
        return
    '''
    map_number = int(choose)
    # Tính toán chỉ số bắt đầu và kết thúc của các file input tương ứng
    start_index = (map_number - 1) * 4 + 1
    end_index = map_number * 4

    # Code chọn file ngẫu nhiên
    '''
    # Tạo danh sách tên file phù hợp
    input_files = [f"input{i}.txt" for i in range(start_index, end_index + 1)]
    input_files = [f for f in input_files if os.path.exists(os.path.join(input_folder, f))]

    if not input_files:
        print("No matching input file found.")
        return

    # Chọn file ngẫu nhiên
    filename = random.choice(input_files)
    '''

    # Code chọn file từ bàn phím
    filename = f"input{choose}.txt"
    input_path = os.path.join(input_folder, filename)

    if not os.path.exists(input_path):
        print(f"File {filename} does not exist.")
        return
    
    input_path = os.path.join(input_folder, filename)
    print(f"\n--- Processing file: {filename} ---")

    # Đọc và xử lý file
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
        return

    COL = len(grid[0])
    init()
    a_star_search(grid, src, dest, ROW, COL)

if __name__ == "__main__":
    main()