from Tkinter import Tk, Canvas, Toplevel


from stripstream.pddl.examples.continuous_tamp.continuous_tamp_utils import SUCTION_WIDTH, SUCTION_HEIGHT, STEM_WIDTH,  STEM_HEIGHT

PIXEL_BUFFER = 10
ENV_HEIGHT = 1.


class ContinuousTMPViewer(object):

    def __init__(self, env_region, regions=[], tl_x=0, tl_y=0, width=500, height=250, title='Grid', background='tan'):
        self.tk = Tk()

        self.tk.withdraw()
        self.top = Toplevel(self.tk)
        self.top.wm_title(title)
        self.top.protocol('WM_DELETE_WINDOW', self.top.destroy)

        self.env_region = env_region
        self.regions = regions
        self.tl_x = tl_x
        self.tl_y = tl_y
        self.width = width
        self.height = height
        self.canvas = Canvas(self.top, width=self.width,
                             height=self.height, background=background)
        self.canvas.pack()

        self.move_frame(self.tl_x, self.tl_y)

        self.dist_to_pixel = (
            self.width - 2 * PIXEL_BUFFER) / (self.env_region.w)
        self.dist_width = self.width / self.dist_to_pixel
        self.dist_height = self.height / self.dist_to_pixel
        self.ground_height = self.height - self.dist_to_pixel * ENV_HEIGHT
        self.robot_dist = self.dist_height / 2.

        self.robot = []
        self.blocks = []
        self.holding = None
        self.draw_environment()

    def center(self):
        self.top.update_idletasks()
        w = self.top.winfo_screenwidth()
        h = self.top.winfo_screenheight()
        size = tuple(int(_)
                     for _ in self.top.geometry().split('+')[0].split('x'))
        x = w / 2 - size[0] / 2
        y = h / 2 - size[1] / 2
        self.top.geometry("%dx%d+%d+%d" % (size + (x, y)))

    def move_frame(self, x, y):
        self.top.update_idletasks()
        size = tuple(int(_)
                     for _ in self.top.geometry().split('+')[0].split('x'))
        self.top.geometry("%dx%d+%d+%d" % (size + (x, y)))

    def t_x(self, x):
        return self.dist_to_pixel * (x + self.dist_width / 2.)

    def t_y(self, y):
        return self.ground_height - self.dist_to_pixel * y

    def draw_block(self, block, x):
        self.blocks.append(self.canvas.create_rectangle(self.t_x(x - block.w / 2.), self.t_y(0),
                                                        self.t_x(
                                                            x + block.w / 2.), self.t_y(block.h),
                                                        fill=block.color, outline='black', width=2))

    def draw_holding(self, block, x):
        self.holding = self.canvas.create_rectangle(self.t_x(x - block.w / 2.), self.t_y(self.robot_dist - SUCTION_HEIGHT / 2 - block.h),
                                                    self.t_x(
                                                        x + block.w / 2.), self.t_y(self.robot_dist - SUCTION_HEIGHT / 2),
                                                    fill=block.color, outline='black', width=2)

    def draw_region(self, region):
        self.environment.append(self.canvas.create_rectangle(self.t_x(region.x - region.w / 2.), self.ground_height,
                                                             self.t_x(
                                                                 region.x + region.w / 2.), self.height,
                                                             fill='red', outline='black', width=2))

    def draw_environment(self, table_color='lightgrey', bin_color='grey'):
        self.environment = [
            self.canvas.create_rectangle(self.t_x(-self.env_region.w / 2), self.ground_height,
                                         self.t_x(self.env_region.w /
                                                  2), self.height,
                                         fill=table_color, outline='black', width=2)
        ]
        for region in self.regions:
            self.draw_region(region)

    def draw_robot(self, x, color='yellow'):
        self.robot = [
            self.canvas.create_rectangle(self.t_x(x - SUCTION_WIDTH / 2.), self.t_y(self.robot_dist - SUCTION_HEIGHT / 2.),
                                         self.t_x(
                x + SUCTION_WIDTH / 2.), self.t_y(self.robot_dist + SUCTION_HEIGHT / 2.),
                fill=color, outline='black', width=2),
            self.canvas.create_rectangle(self.t_x(x - STEM_WIDTH / 2.), self.t_y(self.robot_dist + SUCTION_HEIGHT / 2.),
                                         self.t_x(
                x + STEM_WIDTH / 2.), self.t_y(self.robot_dist + SUCTION_HEIGHT / 2. + STEM_HEIGHT),
                fill=color, outline='black', width=2),
        ]

    def clear_state(self):
        for block in self.blocks:
            self.canvas.delete(block)
        for part in self.robot:
            self.canvas.delete(part)
        if self.holding is not None:
            self.canvas.delete(self.holding)

    def clear_all(self):
        self.canvas.delete('all')

    def save(self, filename):

        from PIL import ImageGrab
        ImageGrab.grab((0, 0, self.width, self.height)).save(filename + '.jpg')
