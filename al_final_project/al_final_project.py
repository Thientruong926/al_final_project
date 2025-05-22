import math
import heapq
import os
import random

# Giá trị pin tối đa
pin_max = 100.0

# Lớp đại diện cho mỗi ô trong lưới
class Cell:
    def __init__(self):
        self.parent_i = 0  # chỉ số hàng của ô cha
        self.parent_j = 0  # chỉ số cột của ô cha
        self.f = float('inf')  # tổng chi phí f = g + h
        self.g = float('inf')  # chi phí từ điểm bắt đầu đến ô hiện tại
        self.h = 0  # ước lượng chi phí từ ô hiện tại đến đích
        self.battery = pin_max  # mức pin còn lại

# Kiểm tra xem chỉ số ô có nằm trong lưới không
def is_valid(row, col, ROW, COL):
    return 0 <= row < ROW and 0 <= col < COL

# Kiểm tra ô có phải điểm đích không
def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Hàm ước lượng khoảng cách đến đích bằng khoảng cách Euclidean (có scale 400)
def calculate_h_value(row, col, dest):
    return math.hypot(400.0 * (row - dest[0]), 400.0 * (col - dest[1]))

# Truy vết đường đi từ đích về đầu, tính tổng chi phí di chuyển

def trace_path(cell_details, dest, grid):
    path = []
    row, col = dest
    total_cost = 0.0

    # Lặp lại cho đến khi về đến điểm bắt đầu
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col, cell_details[row][col].battery))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j

        dx = abs(row - temp_row)
        dy = abs(col - temp_col)
        step_cost = math.sqrt(2) if dx == 1 and dy == 1 else 1.0

        # Lấy giá trị ô hiện tại (nếu là kí tự thì xem như chi phí = 1)
        if isinstance(grid[row][col], str):
            cell_value = 1
        else:
            cell_value = grid[row][col]
            if cell_value == 0:
                cell_value = 1

        total_cost += cell_value * step_cost

        row, col = temp_row, temp_col

    # Thêm điểm bắt đầu
    path.append((row, col, cell_details[row][col].battery))
    path.reverse()

    with open("output.txt", "w") as f:
        f.write("The Path is:\n")
        for i in path:
            f.write(f" -> {i}\n")
        f.write(f"\nTotal cost of the Path is: {total_cost:.2f}\n")

# Thuật toán A* tìm đường có xét đến pin và địa hình
def a_star_search(grid, src, dest, ROW, COL):
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

    while open_list:
        f_val, i, j, battery = heapq.heappop(open_list)

        if closed_list[i][j]:
            continue
        closed_list[i][j] = True

        for dir in directions:
            new_i, new_j = i + dir[0], j + dir[1]

            if not is_valid(new_i, new_j, ROW, COL):
                continue

            # Né các ô trọng số > 3
            if grid[new_i][new_j] > 3:
                continue

            move_cost = round(400.0 * math.sqrt(2), 2) if dir[0] != 0 and dir[1] != 0 else 400.0

            curr_cell_cost = 0.0 if grid[i][j] == -1 else grid[i][j]
            new_cell_cost = 0.0 if grid[new_i][new_j] == -1 else grid[new_i][new_j]

            #Tính toán điều kiện thực tế
            if curr_cell_cost == new_cell_cost:
                battery_cost = 0.02 * move_cost
                new_battery = battery - battery_cost
            elif curr_cell_cost > new_cell_cost:
                movedown_cost = (curr_cell_cost - new_cell_cost) * 100.0
                battery_cost = 0.02 * (move_cost + (movedown_cost * 0.1))
                move_cost += movedown_cost
                new_battery = battery - battery_cost
            else:
                moveup_cost = (new_cell_cost - curr_cell_cost) * 100.0
                battery_cost = 0.02 * (move_cost + (moveup_cost * 1.25))
                move_cost += moveup_cost
                new_battery = battery - battery_cost

            if new_battery <= 0.0:
                trace_path(cell_details, (i, j), grid)
                with open("output.txt", "w") as f:
                    f.write("The drone ran out of battery\n")
                continue

            if new_cell_cost == 0.0 and not is_destination(new_i, new_j, dest):
                new_battery = pin_max

            if is_destination(new_i, new_j, dest):
                cell_details[new_i][new_j].parent_i = i
                cell_details[new_i][new_j].parent_j = j
                trace_path(cell_details, dest, grid)
                found_dest = True
                return

            g_new = cell_details[i][j].g + move_cost

            '''
            # Nếu pin nhỏ hơn hoặc bằng 80 thì ưu tiên tìm điểm 0, nếu không thì tìm B
            if new_battery <= 100.0 and list_of_zero_cells:
                # Tính khoảng cách gần nhất đến điểm sạc
                h_to_closest_zero = min(calculate_h_value(new_i, new_j, zero_cell) for zero_cell in list_of_zero_cells)
                h_to_dest = calculate_h_value(new_i, new_j, dest)

                # Nếu điểm sạc gần hơn đích thì ưu tiên sạc
                if h_to_closest_zero < h_to_dest:
                    h_new = h_to_closest_zero
                else:
                    h_new = h_to_dest
            else:
                h_new = calculate_h_value(new_i, new_j, dest)
            '''
            # Tính khoảng cách gần nhất đến điểm sạc
            h_to_closest_zero = min(calculate_h_value(new_i, new_j, zero_cell) for zero_cell in list_of_zero_cells)
            h_to_dest = calculate_h_value(new_i, new_j, dest)

            # Nếu điểm sạc gần hơn đích thì ưu tiên sạc
            if h_to_closest_zero < h_to_dest:
                h_new = h_to_closest_zero
            else:
                h_new = h_to_dest

            f_new = g_new + h_new

            if cell_details[new_i][new_j].f > f_new:
                heapq.heappush(open_list, (f_new, new_i, new_j, new_battery))
                cell_details[new_i][new_j].f = f_new
                cell_details[new_i][new_j].g = g_new
                cell_details[new_i][new_j].h = h_new
                cell_details[new_i][new_j].battery = round(new_battery, 2)
                cell_details[new_i][new_j].parent_i = i
                cell_details[new_i][new_j].parent_j = j

    if not found_dest:
        with open("output.txt", "w") as f:
            f.write("Cannot find the destination cell\n")

# Hàm chính: đọc dữ liệu từ file và gọi thuật toán

def main():
    choose = input("Choose map (1-3): ").strip()

    input_folder = "input"
    
    # Chuyển map chọn thành index tương ứng với dãy file
    try:
        map_number = int(choose)
        if map_number not in [1, 2, 3]:
            raise ValueError
    except ValueError:
        print("Please select a number from 1 to 3")
        return
    map_number = int(choose)
    # Tính toán chỉ số bắt đầu và kết thúc của các file input tương ứng
    start_index = (map_number - 1) * 4 + 1
    end_index = map_number * 4

    # Code chọn file ngẫu nhiên
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
    '''
    
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
    a_star_search(grid, src, dest, ROW, COL)

if __name__ == "__main__":
    main()