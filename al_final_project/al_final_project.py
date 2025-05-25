import math
import heapq
import os

PIN_MAX = 100.0  # Mức pin tối đa cho drone

# Lớp Cell lưu thông tin chi tiết của từng ô trong quá trình tìm đường
class Cell:
    def __init__(self):
        self.parent_i = 0  # Dòng của ô cha
        self.parent_j = 0  # Cột của ô cha
        self.f = float('inf')  # Tổng chi phí f = g + h
        self.g = float('inf')  # Chi phí từ điểm bắt đầu đến ô hiện tại
        self.h = 0  # Heuristic - chi phí ước lượng đến đích
        self.battery = PIN_MAX  # Mức pin còn lại tại ô này

# Kiểm tra ô có hợp lệ (nằm trong phạm vi lưới)
def is_valid(row, col, ROW, COL):
    return 0 <= row < ROW and 0 <= col < COL

# Kiểm tra ô hiện tại có phải là điểm đích không
def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Tính heuristic (ước lượng khoảng cách 3D đến đích)
def calculate_h_value(row, col, dest, curr_height, dest_height):
    if any(x is None or (isinstance(x, float) and math.isnan(x)) for x in [curr_height, dest_height]):
        return float('inf')
    dx = 400.0 * (row - dest[0])  # Khoảng cách theo hàng
    dy = 400.0 * (col - dest[1])  # Khoảng cách theo cột
    dz = 100.0 * (dest_height - curr_height)  # Độ cao chênh lệch
    return math.hypot(dz, math.hypot(dx, dy))  # Tính khoảng cách 3D

# Truy vết và ghi lại đường đi sau khi tìm được
def trace_path(cell_details, dest, grid):
    path = []
    row, col = dest
    total_cost = 0.0

    # Lặp ngược lại từ đích về nguồn qua các ô cha
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col, cell_details[row][col].battery))
        prev_row = cell_details[row][col].parent_i
        prev_col = cell_details[row][col].parent_j

        dx = abs(row - prev_row)
        dy = abs(col - prev_col)
        step_cost = math.sqrt(2) if dx and dy else 1.0  # Chéo = căn bậc 2, ngang/dọc = 1
        cell_value = 1 if isinstance(grid[row][col], str) else (grid[row][col] or 1)
        total_cost += cell_value * step_cost

        row, col = prev_row, prev_col

    # Thêm ô bắt đầu vào đường đi
    path.append((row, col, cell_details[row][col].battery))
    path.reverse()

    # Ghi kết quả ra file
    with open("output.txt", "w") as f:
        f.write("The Path is:\n")
        for node in path:
            f.write(f" -> {node}\n")
        f.write(f"\nTotal cost of the Path is: {total_cost:.2f}\n")

# Hàm chính thực hiện thuật toán A*
def a_star_search(grid, src, dest, ROW, COL):
    # Kiểm tra hợp lệ của điểm bắt đầu và kết thúc
    if not is_valid(*src, ROW, COL) or not is_valid(*dest, ROW, COL):
        print("Source or destination is invalid")
        return

    # Danh sách đánh dấu ô đã xét
    closed = [[False for _ in range(COL)] for _ in range(ROW)]
    # Ma trận lưu chi tiết thông tin mỗi ô
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    # Khởi tạo ô bắt đầu
    si, sj = src
    cell_details[si][sj].f = 0.0
    cell_details[si][sj].g = 0.0
    cell_details[si][sj].h = 0.0
    cell_details[si][sj].battery = PIN_MAX
    cell_details[si][sj].parent_i = si
    cell_details[si][sj].parent_j = sj

    # Hàng đợi ưu tiên lưu các ô đang xét theo f
    open_list = [(0.0, si, sj, PIN_MAX)]
    # Các hướng đi: 8 hướng (ngang, dọc, chéo)
    directions = [(0, 1), (1, 0), (1, 1), (-1, 0), (0, -1), (1, -1), (-1, 1), (-1, -1)]
    # Danh sách các ô có thể sạc pin (giá trị 0)
    recharge_cells = [(r, c) for r in range(ROW) for c in range(COL) if grid[r][c] == 0]

    while open_list:
        f, i, j, battery = heapq.heappop(open_list)  # Lấy ô có f nhỏ nhất

        if closed[i][j]:  # Bỏ qua nếu ô đã xét
            continue
        closed[i][j] = True  # Đánh dấu ô đã xét

        for d in directions:
            ni, nj = i + d[0], j + d[1]
            if not is_valid(ni, nj, ROW, COL) or grid[ni][nj] > 4:
                continue  # Bỏ qua ô không hợp lệ hoặc vật cản (giá trị lớn hơn 3)

            # Tính chi phí di chuyển giữa hai ô
            move_cost = 400.0 * math.sqrt(2) if d[0] and d[1] else 400.0  # chéo thì √2, ngang/dọc thì 400
            curr_h = 0.0 if grid[i][j] == -1 else grid[i][j]
            new_h = 0.0 if grid[ni][nj] == -1 else grid[ni][nj]

            if curr_h == new_h:  # Không đổi độ cao
                cost = 0.02 * move_cost
                new_battery = battery - cost
            elif curr_h > new_h:  # Xuống dốc
                down = (curr_h - new_h) * 100.0
                cost = 0.02 * (move_cost + 0.1 * down)
                move_cost += down
                new_battery = battery - cost
            else:  # Lên dốc
                up = (new_h - curr_h) * 100.0
                cost = 0.02 * (move_cost + 1.25 * up)
                move_cost += up
                new_battery = battery - cost

            if new_battery <= 0:  # Không đủ pin
                continue

            # Nếu ô là ô sạc và không phải đích, thì nạp đầy pin
            if new_h == 0.0 and not is_destination(ni, nj, dest):
                new_battery = PIN_MAX

            # Nếu đã tới đích
            if is_destination(ni, nj, dest):
                cell_details[ni][nj].parent_i = i
                cell_details[ni][nj].parent_j = j
                trace_path(cell_details, dest, grid)
                return

            # Tính lại g, h, f mới
            g_new = cell_details[i][j].g + move_cost
            h_to_recharge = min(
                (calculate_h_value(ni, nj, rcell, new_h, 0.0) for rcell in recharge_cells),
                default=float('inf')
            )
            h_to_dest = calculate_h_value(ni, nj, dest, new_h, 0.0)
            h_new = min(h_to_recharge, h_to_dest)  # Heuristic ưu tiên sạc nếu gần hơn
            f_new = g_new + h_new

            # Nếu đường đi mới tốt hơn, cập nhật lại
            if cell_details[ni][nj].f > f_new:
                heapq.heappush(open_list, (f_new, ni, nj, new_battery))
                cell_details[ni][nj].f = f_new
                cell_details[ni][nj].g = g_new
                cell_details[ni][nj].h = h_new
                cell_details[ni][nj].battery = round(new_battery, 2)
                cell_details[ni][nj].parent_i = i
                cell_details[ni][nj].parent_j = j

    # Nếu không tìm được đường đến đích
    with open("output.txt", "w") as f:
        f.write("Cannot find the destination cell\n")

# Hàm chính để chạy chương trình
def main():
    map_id = input("Choose map (1-12): ").strip()
    path = os.path.join("input", f"input{map_id}.txt")

    if not os.path.exists(path):
        print(f"File input{map_id}.txt does not exist.")
        return

    print(f"\n--- Processing file: input{map_id}.txt ---")

    # Đọc dữ liệu từ file
    with open(path, "r") as f:
        lines = [line.strip() for line in f]

    ROW = int(lines[0])  # Số hàng trong bản đồ
    grid = []
    src = dest = None

    # Phân tích bản đồ, tìm vị trí A và B
    for i in range(1, ROW + 1):
        tokens = lines[i].split()
        row = []
        for j, val in enumerate(tokens):
            if val == 'A':
                src = (i - 1, j)
                row.append(-1)  # Gán -1 cho ô A
            elif val == 'B':
                dest = (i - 1, j)
                row.append(-1)  # Gán -1 cho ô B
            else:
                row.append(int(val))  # Giá trị độ cao
        grid.append(row)

    if src is None or dest is None:
        print("Error: Start (A) or Destination (B) not found.")
        return

    COL = len(grid[0])  # Số cột
    a_star_search(grid, src, dest, ROW, COL)

if __name__ == "__main__":
    main()
