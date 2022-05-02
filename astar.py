import random

import pygame

# colors
FILL = (255, 255, 255)
WALL = (0, 0, 0)
PATH = (238, 130, 238)
NODES = (255, 0, 0)
CLOSED_SET = (230, 230, 250)
OPEN_SET = (188, 245, 188)
HIGHLIGHT = (186, 85, 211)
# pygame settings
FPS = 20
windowX = 800
windowY = 600
WIDTH = 600
HEIGHT = 600
rows = 40
cols = 40
width = WIDTH // cols
height = HEIGHT // rows
SCREEN = pygame.display.set_mode((windowX, windowY))
pygame.display.set_caption('A* Pathfinding Algorithm')
# a* variables
grid = []
openSet = []
closedSet = []
path = []
costs = [0.5, 1]
wall_probability = 0.2
num_of_targets = 4


def heuristic(a, b):
    #L1 distance since we don't have diagonal moves
    dist = 0
    distX = abs(a.i - b.i)
    distY = abs(a.j - b.j)
    for i in range(distX):
        dist += costs[0]
    for j in range(distY):
        dist += costs[1]
    return dist


def a_star(targetNode):
    solution = True
    solved = False
    if len(openSet) > 0:
        winner = 0
        for i in range(len(openSet)):
            if openSet[i].f < openSet[winner].f:
                winner = i
            # if there is a tie according to heuristic, then
            # prefer to explore options with the longer
            # known paths (closer to goal)
            if openSet[i].f == openSet[winner].f:
                if openSet[i].g > openSet[winner].g:
                    winner = i

        current = openSet[winner]

        if current is targetNode:
            solved = True
            path.insert(0, targetNode)
            return solved, solution

        openSet.remove(current)
        closedSet.append(current)
        neighbours = current.neighbours
        update_cost(current)

        for neighbour in neighbours:
            if neighbour not in closedSet and not neighbour.isWall:
                newPath = current.g + neighbour.cost
                if newPath < neighbour.g or neighbour not in openSet:
                    neighbour.g = newPath
                    neighbour.h = heuristic(neighbour, targetNode)
                    neighbour.f = neighbour.g + neighbour.h
                    neighbour.parent = current
                    if neighbour not in openSet:
                        openSet.append(neighbour)

    else:
        solution = False
        return solved, solution

    # find the path
    if solution:
        path.clear()
        temp = current
        path.append(temp)
        while temp.parent:
            path.append(temp.parent)
            temp = temp.parent

    return solved, solution


class Cell:

    def __init__(self, i, j):
        self.i = i
        self.j = j
        self.f = 0
        self.g = 0
        self.h = 0
        self.width = WIDTH // cols
        self.height = HEIGHT // rows
        self.x = i * self.width
        self.y = j * self.height
        self.neighbours = []
        self.isWall = False
        self.parent = None
        self.isTraversed = False
        self.cost = 0

        if random.random() < wall_probability:
            self.isWall = True

    def addNeighbours(self):
        i = self.i
        j = self.j
        if i < cols - 1:
            self.neighbours.append(grid[i + 1][j])
        if i > 0:
            self.neighbours.append(grid[i - 1][j])
        if j < rows - 1:
            self.neighbours.append(grid[i][j + 1])
        if j > 0:
            self.neighbours.append(grid[i][j - 1])

    def show(self, color):
        if self.isWall:
            pygame.draw.rect(SCREEN, WALL,
                             (self.x, self.y, self.width, self.height))
        else:
            pygame.draw.rect(SCREEN, color,
                             (self.x, self.y, self.width, self.height))


def update_cost(cell):
    neighbours = cell.neighbours
    for neighbour in neighbours:
        if neighbour not in closedSet:
            if neighbour.j == cell.j:
                neighbour.cost = costs[0]
            elif neighbour.i == cell.i:
                neighbour.cost = costs[1]


def calculate_cost():
    cost = 0
    for cell in reversed(path):
        cost += cell.cost
    return cost


def generate_targets():
    targets = []
    for i in range(num_of_targets):
        targets.append(
            grid[random.randint(1, cols - 1)][random.randint(1, rows - 1)])
    return targets


def setup():
    for i in range(cols):
        grid.append([])

    for i in range(cols):
        for j in range(rows):
            new_cell = Cell(i, j)
            grid[i].append(new_cell)

    for i in range(cols):
        for j in range(rows):
            grid[i][j].addNeighbours()

    startNode = grid[0][0]
    openSet.append(startNode)
    Nodes = [startNode]
    for target in generate_targets():
        Nodes.append(target)

    for node in Nodes:
        node.isWall = False

    return Nodes


def next_target(startTarget, targets):
    exists_next_target = False
    startTarget.isTraversed = True
    min_dist = heuristic(grid[0][0], grid[cols - 1][rows - 1])
    nextTarget = startTarget
    for target in targets[1:]:
        if target != startTarget and not target.isTraversed:
            exists_next_target = True
            if heuristic(startTarget, target) < min_dist:
                min_dist = heuristic(startTarget, target)
                nextTarget = target
    return nextTarget, exists_next_target


def draw_grid():
    for i in range(cols + 1):
        pygame.draw.line(SCREEN, WALL, (0, i * width), (WIDTH, i * width))
        for j in range(rows + 1):
            pygame.draw.line(SCREEN, WALL, (j * height, 0),
                             (j * height, HEIGHT))


def draw_menu(score, nodes, solution, expansions):
    border_radius = 2
    gap = 10
    font_size = 2 * gap
    line_gap = 2 * font_size
    count = 1

    # menu window
    x1 = WIDTH + gap
    y1 = gap
    x2 = windowX - gap
    y2 = windowY - 10
    pygame.draw.rect(SCREEN, WALL, (x1, y1, x2 - x1, y2 - y1))
    pygame.draw.rect(SCREEN, FILL, (x1 + border_radius, y1 + border_radius,
                                    x2 - x1 - 2 * border_radius,
                                    y2 - y1 - 2 * border_radius))
    # text init
    pygame.font.init()
    my_font = pygame.font.SysFont('arial', font_size)
    text_score_label = my_font.render('SCORE', True, WALL)
    text_nodes_label = my_font.render('TARGETS', True, WALL)
    text_score = my_font.render(str(score), True, HIGHLIGHT)
    text_expansions_label = my_font.render('EXPANSIONS', True, WALL)
    text_expansions = my_font.render(str(expansions), True, HIGHLIGHT)
    text_rect_label_1 = text_score_label.get_rect(
        topleft=(x1 + font_size, y1 + count*line_gap))
    count += 1
    text_rect_score = text_score.get_rect(
        topleft=(x1 + font_size, y1 + count*line_gap))
    count += 1
    text_rect_label_2 = text_expansions_label.get_rect(
        topleft=(x1 + font_size, y1 + count*line_gap))
    count += 1
    text_rect_expansions = text_score.get_rect(
        topleft=(x1 + font_size, y1 + count*line_gap))
    count += 1
    text_rect_label_3 = text_nodes_label.get_rect(
        topleft=(x1 + font_size, y1 + count*line_gap))
    count += 1

    for node in nodes[1:]:

        nodes_str = [node.i, node.j]
        if node.isTraversed:
            text_nodes = my_font.render(str(nodes_str), True, HIGHLIGHT)
        else:
            text_nodes = my_font.render(str(nodes_str), True, NODES)
        text_rect_nodes = text_nodes_label.get_rect(
            topleft=(x1 + font_size, y1 + count*line_gap))
        count += 1
        SCREEN.blit(text_nodes, text_rect_nodes)
    count += 1

    solution_str = ''
    if all(node.isTraversed for node in nodes):
        solution_str = 'Solved!'
    else:
        if not solution:
            solution_str = 'No solution!'

    text_solution_label = my_font.render(solution_str, True, WALL)
    text_rect_label_4 = text_solution_label.get_rect(
        topleft=(x1 + font_size, y1 + count*line_gap))
    SCREEN.blit(text_score_label, text_rect_label_1)
    SCREEN.blit(text_score, text_rect_score)
    SCREEN.blit(text_expansions_label, text_rect_label_2)
    SCREEN.blit(text_expansions, text_rect_expansions)
    SCREEN.blit(text_nodes_label, text_rect_label_3)
    SCREEN.blit(text_solution_label, text_rect_label_4)


def draw_buttons(label):
    gap = 10
    font_size = 2 * gap
    x1 = WIDTH + gap
    y1 = gap
    x2 = windowX - gap
    y2 = windowY - 10
    pygame.font.init()
    my_font = pygame.font.SysFont('arial', font_size)
    buttonx1 = x1 + font_size
    buttony1 = y2 - y1 - 2 * font_size
    buttonx2 = x2 - font_size
    buttony2 = y2 - y1
    start_button = pygame.draw.rect(SCREEN, WALL, (
        buttonx1, buttony1, buttonx2 - buttonx1, buttony2 - buttony1))

    text_start_label = my_font.render(label, True, FILL)
    text_start = text_start_label.get_rect(
        center=(start_button.centerx, start_button.centery))
    SCREEN.blit(text_start_label, text_start)

    buttons = [start_button]
    return buttons


def draw(nodes):
    SCREEN.fill(FILL)
    for row in grid:
        for cell in row:
            cell.show(FILL)
            if cell in nodes:
                cell.show(NODES)

    for cell in closedSet:
        if cell not in nodes:
            cell.show(CLOSED_SET)

    for cell in openSet:
        if cell not in nodes:
            cell.show(OPEN_SET)

    for cell in path:
        if cell not in nodes:
            cell.show(PATH)
        else:
            if cell is not nodes[0]:
                pygame.draw.rect(SCREEN, HIGHLIGHT,
                                 (cell.x, cell.y, cell.width, cell.height))


def is_over(rect, pos):
    return True if rect.collidepoint(pos[0], pos[1]) else False


def check_buttons(buttons, pos):
    if is_over(buttons[0], pos):
        return 1
    # return False


def restart():
    path.clear()
    openSet.clear()
    closedSet.clear()
    grid.clear()
    # return nodes


def main():
    nodes = setup()
    startNode = nodes[0]
    targetNode, exists_next_target = next_target(startNode, nodes)
    solution = True
    nextNode = False

    clock = pygame.time.Clock()
    run = True
    start = False
    count = 1
    label = 'START'
    buttons = []
    num_of_expansions = 0

    while run:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.display.quit()
                pygame.quit()
                quit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                pressed = check_buttons(buttons, pos)
                if pressed == 1:
                    if label == 'RESTART':
                        restart()
                        main()
                    if count % 2 == 0:
                        count += 1
                        label = 'START'
                        start = False
                    else:
                        count += 1
                        label = 'PAUSE'
                        start = True

        if start:
            if exists_next_target:
                if not solution:
                    exists_next_target = False
                if nextNode:
                    openSet.clear()
                    openSet.append(targetNode)
                    targetNode, exists_next_target = next_target(targetNode,
                                                                 nodes)

                nextNode, solution = a_star(targetNode)
                num_of_expansions += 1

        draw(nodes)
        draw_grid()
        draw_menu(calculate_cost(), nodes, solution, num_of_expansions)
        if not exists_next_target:
            label = 'RESTART'
        buttons = draw_buttons(label)

        pygame.display.update()
    pygame.display.quit()
    pygame.quit()


if __name__ == "__main__":
    main()
