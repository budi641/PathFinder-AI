import pygame

class Game:
    def __init__(self, grid, start, goal, path, screen_width=1280, screen_height=720, grid_width=600, grid_height=600):
        pygame.init()
        self.grid = grid
        self.start = start
        self.goal = goal
        self.path = path
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.cell_size = self.grid_width // self.grid.size
        self.grid_offset_x = (self.screen_width - self.grid_width) // 2
        self.grid_offset_y = (self.screen_height - self.grid_height) // 2

        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Pathfinding Visualization")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, self.cell_size // 2)

    def draw_grid(self, current_index, collected_items):
        for x in range(self.grid.size):
            for y in range(self.grid.size):
                cell_rect = pygame.Rect(
                    self.grid_offset_x + x * self.cell_size,
                    self.grid_offset_y + y * self.cell_size,
                    self.cell_size,
                    self.cell_size,
                )

                # Determine cell color
                if (x, y) == (self.path[current_index].x, self.path[current_index].y):
                    color = (0, 255, 0)  # Green for the current position
                elif any((step.x, step.y) == (x, y) for step in self.path[:current_index]):
                    color = (200, 200, 200)  # Grey for visited cells
                else:
                    color = (255, 255, 255)  # White for unvisited cells

                pygame.draw.rect(self.screen, color, cell_rect)
                pygame.draw.rect(self.screen, (0, 0, 0), cell_rect, 1)  # Black border

                # Draw obstacles
                if (x, y) in self.grid.obstacles:
                    pygame.draw.polygon(
                        self.screen,
                        (255, 0, 0),
                        [
                            (cell_rect.left + self.cell_size // 2, cell_rect.top),
                            (cell_rect.left, cell_rect.bottom),
                            (cell_rect.right, cell_rect.bottom),
                        ],
                    )

                # Draw collectibles
                if (x, y) in self.grid.collectibles and (x, y) not in collected_items:
                    pygame.draw.circle(
                        self.screen,
                        (0, 0, 255),
                        cell_rect.center,
                        self.cell_size // 4,
                    )

                # Draw start and goal positions
                if (x, y) == (self.start.x, self.start.y):
                    text = self.font.render("S", True, (0, 0, 0))
                    self.screen.blit(
                        text,
                        text.get_rect(center=cell_rect.center),
                    )
                elif (x, y) == (self.goal.x, self.goal.y):
                    text = self.font.render("G", True, (0, 0, 0))
                    self.screen.blit(
                        text,
                        text.get_rect(center=cell_rect.center),
                    )


    def run(self):
        current_index = 0  # Start at the first step in the path
        collected_items = set()  # Track collected items

        running = True
        while running:
            self.screen.fill((255, 255, 255))
            self.draw_grid(current_index, collected_items)
            pygame.display.flip()
            self.clock.tick(60)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RIGHT and current_index < len(self.path) - 1:
                        current_index += 1
                        # Collect items at the new position
                        x, y = self.path[current_index].x, self.path[current_index].y
                        if (x, y) in self.grid.collectibles:
                            collected_items.add((x, y))
                    elif event.key == pygame.K_LEFT and current_index > 0:
                        # Move back and potentially uncollect an item
                        x, y = self.path[current_index].x, self.path[current_index].y
                        if (x, y) in collected_items:
                            collected_items.remove((x, y))
                        current_index -= 1

        pygame.quit()

