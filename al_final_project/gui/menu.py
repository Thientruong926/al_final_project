from ntpath import join
import os
import random
import pygame
import sys


WIDTH, HEIGHT = 1000, 600
WHITE = (255, 255, 255)
GRAY = (200, 200, 200)
BLUE = (0, 120, 255)
BLACK = (0, 0, 0)

class Button:
    def __init__(self, text, x, y, w, h, callback, color = (255,255,0)):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.callback = callback
        self.color = color

    def draw(self, screen, font, active=False):
        pygame.draw.rect(screen, self.color, self.rect)
        pygame.draw.rect(screen, BLACK, self.rect, 2)
        txt = font.render(self.text, True, BLACK)
        screen.blit(txt, (self.rect.x + (self.rect.w - txt.get_width()) // 2,
                          self.rect.y + (self.rect.h - txt.get_height()) // 2))

    def is_hover(self, pos):
        return self.rect.collidepoint(pos)

class Menu:
    def __init__(self, screen, on_choose_map = None):
        self.screen = screen
        self.font = pygame.font.SysFont(None, 36)
        self.title_font = pygame.font.SysFont(None, 60)
        self.special_font = pygame.font.Font(join("src","Roboto-Medium.ttf"), 20)
        self.state = "main"
        self.selected_map = None
        self.instructions_text = [
            "INSTRUCTIONS:",
            "- Choose Map: Chọn bản đồ có sẵn.",
            "- Random Map: Sinh bản đồ ngẫu nhiên với kích thước.",
            "- Instructions: Xem hướng dẫn sử dụng.",
            "- Quit: Thoát chương trình.",
            "- Back: Quay lại bước vừa rồi.",
            "- Pause: Dừng chạy demo",
            "- Resume: Tiếp tục chạy demo",
            "- Menu: Quay về màn hình menu"
        ]
        self.buttons = []
        self.create_main_buttons()
        self.on_choose_map = on_choose_map
        self.should_quit = False 

    def create_main_buttons(self):
        self.buttons = [
            Button("Choose Map", 400, 180, 200, 50, self.choose_map),
            Button("Random Map", 400, 250, 200, 50, self.random_map),
            Button("Instructions", 400, 320, 200, 50, self.instructions),
            Button("Quit", 400, 390, 200, 50, self.quit)
        ]

    def choose_map(self):
        self.state = "choose_map"
        self.buttons = []

        # Đọc danh sách file trong thư mục input
        input_folder = os.path.join(os.path.dirname(__file__), "..", "input")
        files = [f for f in os.listdir(input_folder) if f.endswith(".txt")]
        files.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))

        # Chia thành 4 cột
        col_count = 4
        btn_w, btn_h = 180, 50
        margin_x, margin_y = 30, 20
        start_x = 100
        start_y = 200
        x, y = 0, 0
        for idx, fname in enumerate(files):
            col = idx % col_count
            row = idx // col_count
            x = start_x + col * (btn_w + margin_x)
            y = start_y + row * (btn_h + margin_y)
            btn_text = "Map " + fname[5:-4]
            self.buttons.append(Button(btn_text, x, y, btn_w, btn_h, lambda f=fname: self.select_map(f), (127, 255, 0)))
        self.buttons.append(Button("Back", 400, y + 150 , 200, 50, self.back, (255, 114, 86)))

    def select_map(self, map_filename):
        self.selected_map = map_filename
        if self.on_choose_map:
            self.on_choose_map(map_filename)
        self.should_quit = True

    def random_map(self):
        self.state = "random_map"
        sizes = ["9x9", "12x12", "15x15"]
        choice = [1, 2, 3]
        self.buttons = []
        for i, sz in enumerate(sizes):
            print(f"{i}:{sz}")
            self.buttons.append(Button(sz, 400, 200 + i*60, 200, 50, lambda sz=choice[i]: self.select_random(sz)))
        self.buttons.append(Button("Back", 400, 400 , 200, 50, self.back, (255, 114, 86)))

    def select_random(self, choice):
        print(choice)
        start_index = (choice - 1) * 4 + 1
        end_index = choice * 4
        input_files = [f"input{i}.txt" for i in range(start_index, end_index + 1)]
        filename = random.choice(input_files)
        self.selected_map = filename
        if self.on_choose_map:
            self.on_choose_map(filename)
        self.should_quit = True

    def instructions(self):
        self.state = "instructions"
        self.buttons = []
        self.buttons.append(Button("Back", 400, 500 , 200, 50, self.back, (255, 114, 86)))

    def back(self):
        self.state = "main"
        self.create_main_buttons()
        

    def quit(self):
        pygame.quit()
        sys.exit()

    def run(self):
        running = True
        while running:
            self.screen.fill((0,0,0))
            if self.state == "main":
                title = self.title_font.render("MAIN MENU", True, (255, 255, 0))
                self.screen.blit(title, (WIDTH//2 - title.get_width()//2, 100))
            elif self.state == "choose_map":
                title = self.title_font.render("CHOOSE MAP", True, (0, 255, 255))
                self.screen.blit(title, (WIDTH//2 - title.get_width()//2, 100))
            elif self.state == "random_map":
                title = self.title_font.render("RANDOM MAP", True, (0, 255, 255))
                self.screen.blit(title, (WIDTH//2 - title.get_width()//2, 100))
            elif self.state == "instructions":
                title = self.title_font.render("INSTRUCTIONS", True, (0, 255, 255))
                self.screen.blit(title, (WIDTH//2 - title.get_width()//2, 100))
                y = 150
                for line in self.instructions_text:
                    txt = self.special_font.render(line, True, (255,255,0))
                    self.screen.blit(txt, (100, y))
                    y += 35
            for btn in self.buttons:
                btn.draw(self.screen, self.font)
            pygame.display.flip()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.quit()
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    for btn in self.buttons:
                        if btn.is_hover(event.pos):
                            btn.callback()
            if self.should_quit:
                running = False