import pygame
pygame.init()
global phases
global sced
global cost
# in sced, 0 is the first phase and it will keep on going until all phases are reached
# [[[agent0], [agent1], [agent2], [agent3]....], [agent0, agent1, agent2, agent3....], ...]
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
LBLUE = (135, 206, 0)
WIDTH = 575
HEIGHT = 575
line_spacing = 25
win = pygame.display.set_mode((500, 500))
pygame.display.set_caption("Path Planning Demo")



class BuildPlate:
    def __init__(self, position, height, width, n,  covered_points=[]):
        self.position = position
        self.covered_points = covered_points
        self.height = height
        self.width = width
        self.num = n

    def __eq__(self, other):
        return self.covered_points == other.covered_points

    def resize_build_plate(self, position, height, width, covered_points=[]):
        self.position = position
        self.covered_points = covered_points
        self.height = height
        self.width = width

    def get_covered_points(self):
        return self.covered_points

    def get_position(self):
        return self.position

    def get_height(self):
        return self.height

    def get_width(self):
        return self.width

    def getNum(self):
        return self.num

    def setNum(self, n):
        self.num = n

class MovingPlatform:
    def __init__(self, position, platformID, color):
        self.position = position
        self.platformID = platformID
        self.prev_position = (0, 0)
        self.color = color

    def __eq__(self, other):
        return self.platformID == other.platformID

    def set_position(self, new_position):
        self.prev_position = self.position
        self.position = new_position

    def get_position(self):
        return self.position

    def get_id(self):
        return self.platformID

    def get_color(self):
        return self.color


platform_zero = MovingPlatform((1, 13), 0, GREEN)
platform_one = MovingPlatform((1, 14), 1, GREEN)
platform_two = MovingPlatform((1, 15), 2, GREEN)
platform_three = MovingPlatform((1, 16), 3, GREEN)
arm_list = []
platform_list = [platform_zero, platform_one, platform_two, platform_three]
build_plate_list = []


def draw_grid():
    for row in range(25):
        pygame.draw.line(win, WHITE, (0, row * 25), (WIDTH, row * 25))
        pygame.draw.line(win, WHITE, (row * 25, 0), (row * 25, HEIGHT))


def draw_bots(platform_list=[]):
    bot_size = 18
    half_size = 9
    for bots in platform_list:
        pygame.draw.rect(win, bots.get_color(), (((bots.get_position()[0] + 1) * 25) - half_size,
                                               (((bots.get_position()[1] + 1) * 25) - half_size), bot_size, bot_size))


def draw_build_plate(build_plate_list=[]):
    for plates in build_plate_list:
        if plates.getNum() == 1:
            pygame.draw.rect(win, LBLUE, (((plates.get_position()[0] + 1) * 25), ((plates.get_position()[1] + 1) * 25),
                                        plates.get_height(), (plates.get_width())))
            plates.setNum(0)

        if plates.getNum() == 0:
            pygame.draw.rect(win, BLUE, (((plates.get_position()[0] + 1) * 25), ((plates.get_position()[1] + 1) * 25),
                                        plates.get_height(), (plates.get_width())))


def draw_objects():
    global arm_list
    global platform_list
    global build_plate_list
    draw_bots(platform_list)
    draw_build_plate(build_plate_list)


def get_max(outer_planner):
    outer_max = 0
    for lines in outer_planner:
        inner_max = -1
        for line in lines:
            if len(line) > inner_max:
                inner_max = len(line)
        outer_max = outer_max + inner_max
    return outer_max


def extend_elements(outer_planner):
    max = -1
    for lines in outer_planner:
        for line in lines:
            if len(line) > max:
                max = len(line)
    for lines in outer_planner:
        for line in lines:
            while len(line) != max:
                current_len = len(line)
                adder = line[current_len - 1]
                line.append(adder)

    return outer_planner


def main():
    global build_plate_list
    global platform_list
    global cost

    draw_grid()
    draw_objects()
    pygame.time.delay(100)
    pygame.display.update()
    global phases
    global sced
    phases = int(input("How many phases?"))
    outer_planner = []
    outer_o_planner = []
    current_phase = 1
    while current_phase <= phases:
        p1 = "phase"
        p2 = ".txt"
        file_name = p1 + str(current_phase) + p2
        #print(file_name)
        planner = []
        o_planner = []
        with open(file_name, 'r+') as f:
            phase = f.readline()
            #print(phase)
            num_agents = int(f.readline())
            #print(num_agents)
            for it in range(num_agents):
                agent_id = f.readline()
                num_of_steps = int(f.readline())
                planner_inner = []
                for it2 in range(num_of_steps):
                    row = f.readline().split()
                    row = [int(i) for i in row]
                    planner_inner.append(row)
                planner.append(planner_inner)
            empty_space = f.readline()
            obstacle = f.readline()
            num_obstacles = int(f.readline())
            #print(num_obstacles)
            for it4 in range(num_obstacles):
                bamp = f.readline().split()
                bamp = [int(i) for i in bamp]
                if bamp[0] >= 0:
                    o_planner.append(bamp)

        outer_planner.append(planner)
        current_phase = current_phase + 1
        outer_o_planner.append(o_planner)
    outer_planner = extend_elements(outer_planner)
    start_loop = 0
    end_loop = len(outer_planner)
    next_run = 0
    pygame.draw.rect(win, BLACK, (0, 0, 500, 500))
    draw_grid()
    draw_objects()
    pygame.time.delay(100)
    pygame.display.update()
    while start_loop < end_loop:
        current_agents = len(outer_planner[start_loop])
        #next_run = int(input(next_run))

        #if next_run == 1:
        for itr, (a, b, c, d) in enumerate(zip(outer_planner[start_loop][0], outer_planner[start_loop][1], outer_planner[start_loop][2], outer_planner[start_loop][3])):
            pygame.event.get()
            #print(a)
            #print(b)
            #print(c)
            #print(d)

            #print((str(a[0]) + " " + str(a[1])))
            #print((str(b[0]) + " " + str(b[1])))
            #print((str(c[0]) + " " + str(c[1])))
            #print((str(d[0]) + " " + str(d[1])))
            platform_list[0].set_position((a[0], a[1]))
            platform_list[1].set_position((b[0], b[1]))
            platform_list[2].set_position((c[0], c[1]))
            platform_list[3].set_position((d[0], d[1]))
            pygame.draw.rect(win, BLACK, (0, 0, 500, 500))
            draw_grid()
            draw_objects()
            pygame.time.delay(10)
            pygame.display.update()
        for objects in outer_o_planner[start_loop]:
            print(objects)
            if len(objects) > 0:
                build_plate_list.append(BuildPlate((objects[0] - .5, objects[1] - .5), 25, 25, 1, []))
        print(" ")
        pygame.draw.rect(win, BLACK, (0, 0, 500, 500))
        draw_grid()
        draw_objects()
        pygame.time.delay(80)
        pygame.display.update()
        start_loop = start_loop + 1

    pygame.draw.rect(win, BLACK, (0, 0, 500, 500))
    draw_grid()
    draw_objects()
    pygame.time.delay(100)
    pygame.display.update()
    cost = get_max(outer_planner)
    print("-----------------------------")
    print("max cost: " + str(cost))
    print("-----------------------------")


    """
    for lines in outer_planner:
        for line in lines:
            print(line)
            print(" ")
    
    for lines in outer_planner:
        for line in lines:
            print(line)
            print(" ")
    
    outer_planner = extend_elements(outer_planner)

    for lines in outer_planner:
        for line in lines:
            print(line)
            print(len(line))
            print(" ")

    for line in outer_o_planner:
        print(line)
        print(" ")
    """
    active = True
    while active:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                active = False
    pygame.quit()


if __name__ == "__main__":
    main()