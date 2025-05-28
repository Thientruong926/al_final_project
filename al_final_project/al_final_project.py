import math
import heapq
import os
import random

pin_max = 100.0  # Mức pin tối đa cho drone

# Lớp Cell dùng để lưu thông tin chi tiết của mỗi ô trong lưới
class Cell:
    def __init__(self):
        self.parent_i = 0  # Tọa độ hàng của ô cha (để truy vết đường đi)
        self.parent_j = 0  # Tọa độ cột của ô cha
        self.f = float('inf')  # Tổng chi phí f = g + h
        self.g = float('inf')  # Chi phí thực tế từ điểm xuất phát
        self.h = 0  # Heuristic ước lượng đến đích
        self.battery = pin_max  # Mức pin còn lại tại ô này
        self.total_cost = 0

# Hàm kiểm tra ô có nằm trong phạm vi lưới hay không
def is_valid(row, col, ROW, COL):
    return 0 <= row < ROW and 0 <= col < COL

# Kiểm tra ô hiện tại có phải là đích không
def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Tính heuristic (h) theo khoảng cách Euclidean, đơn vị là mm
def calculate_h_value(row, col, dest, curr_height, dest_height):
    return math.hypot(100.0 * (dest_height - curr_height), math.hypot(400.0 * (row - dest[0]), 400.0 * (col - dest[1])))

# Truy vết đường đi từ đích về nguồn và ghi ra file
def trace_path(cell_details, dest):
    path = []
    row, col = dest
    final_total_cost = 0

    # Truy ngược lại từ đích về nguồn
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col, cell_details[row][col].battery))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        final_total_cost += cell_details[row][col].total_cost

        row, col = temp_row, temp_col

    path.append((row, col, cell_details[row][col].battery))  # Thêm điểm bắt đầu
    path.reverse()  # Đảo ngược lại để in theo thứ tự đi

    with open("output.txt", "w") as f:
        f.write("The Path is:\n")
        for i in path:
            f.write(f" -> {i}\n")
        f.write(f"\nTotal cost of the Path is: {final_total_cost:.2f}\n")

# Thuật toán A* để tìm đường đi tối ưu từ src đến dest
def a_star_search(grid, src, dest, ROW, COL):
    if not is_valid(src[0], src[1], ROW, COL) or not is_valid(dest[0], dest[1], ROW, COL):
        print("Source or destination is invalid")
        return

    # Danh sách các ô đã đóng (đã xét)
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    # Khởi tạo thông tin chi tiết của từng ô
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]
    # Tọa độ xuất phát
    i, j = src

    # Cài đặt thông tin cho ô xuất phát
    cell_details[i][j].f = 0.0
    cell_details[i][j].g = 0.0
    cell_details[i][j].h = 0.0
    cell_details[i][j].battery = pin_max
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    open_list = []
    heapq.heappush(open_list, (0.0, i, j, pin_max))  # Thêm điểm bắt đầu vào danh sách mở
    found_dest = False

    # Các hướng đi: phải, xuống, chéo phải xuống, lên, trái, chéo trái xuống, chéo phải lên, chéo trái lên
    directions = [(0, 1), (1, 0), (1, 1), (-1, 0), (0, -1), (1, -1), (-1, 1), (-1, -1)]

    # Tập hợp các ô có giá trị 0 để ưu tiên sạc
    list_of_zero_cells = [(r, c) for r in range(ROW) for c in range(COL) if grid[r][c] == 0]

    # Bắt đầu duyệt danh sách mở
    while open_list:
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
                new_battery = pin_max  # Sạc đầy khi đi qua ô 0

            if is_destination(new_i, new_j, dest):
                cell_details[new_i][new_j].parent_i = i
                cell_details[new_i][new_j].parent_j = j
                cell_details[new_i][new_j].total_cost = move_cost
                trace_path(cell_details, dest)
                found_dest = True
                return

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
            
            # Nếu có trạm gần đó thì đi tới đó luôn, tránh trường hợp gần đến đích hết pin lại phải đi tìm trạm
            if h_to_closest_zero > h_to_dest:
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
                cell_details[new_i][new_j].total_cost = move_cost

    # Nếu không tìm thấy đích
    if not found_dest:
        with open("output.txt", "w") as f:
            f.write("Cannot find the destination cell\n")

# Hàm chính để chạy chương trình
def main():
    choose = input("Choose map (1-12): ").strip()
    input_folder = "input"
    map_number = int(choose)
    filename = f"input{choose}.txt"
    input_path = os.path.join(input_folder, filename)

    if not os.path.exists(input_path):
        print(f"File {filename} does not exist.")
        return

    print(f"\n--- Processing file: {filename} ---")

    with open(input_path, "r") as f:
        lines = [line.strip() for line in f.readlines()]

    ROW = int(lines[0])
    grid = []
    src = dest = None

    # Đọc lưới và xác định điểm A (nguồn) và B (đích)
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

# Điểm bắt đầu chương trình
if __name__ == "__main__":
    main()