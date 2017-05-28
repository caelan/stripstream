from Tkinter import Tk, Canvas, Toplevel


MAX_ROWS = 5
MAX_COLS = 10


class CountableTMPViewer(object):

    def __init__(self, rows, cols, width=500, height=500, side=25, block_buffer=10, title='Grid', background='tan'):
        tk = Tk()
        tk.withdraw()
        top = Toplevel(tk)
        top.wm_title(title)
        top.protocol('WM_DELETE_WINDOW', top.destroy)

        self.width = width
        self.height = height
        self.rows = min(rows, MAX_ROWS)
        self.cols = min(cols, MAX_COLS)
        self.canvas = Canvas(top, width=self.width,
                             height=self.height, background=background)
        self.canvas.pack()
        self.side = side
        self.block_buffer = block_buffer
        self.cells = {}
        self.robot = []
        self.draw_environment()

    def transform_r(self, r):
        return self.table_y1 + r * (self.side + 2 * self.block_buffer) + 2 * self.block_buffer + self.side / 2

    def transform_c(self, c):

        return self.table_x1 + c * (self.side + 2 * self.block_buffer) + 2 * self.block_buffer + self.side / 2

    def draw_environment(self, table_color='lightgrey', bin_color='grey'):
        table_width = self.cols * \
            (self.side + 2 * self.block_buffer) + 2 * self.block_buffer
        table_height = self.rows * \
            (self.side + 2 * self.block_buffer) + 2 * self.block_buffer

        border_buffer = 50
        self.table_x1 = border_buffer
        self.table_y1 = self.height - table_height - border_buffer

        bin_width = 20
        self.environment = [
            self.canvas.create_rectangle(self.table_x1, self.table_y1,
                                         self.table_x1 + table_width, self.table_y1 + table_height,
                                         fill=table_color, outline='black', width=2),
            self.canvas.create_rectangle(self.table_x1 - bin_width, self.table_y1,
                                         self.table_x1, self.table_y1 + table_height,
                                         fill=bin_color, outline='black', width=2),
            self.canvas.create_rectangle(self.table_x1 + table_width, self.table_y1,
                                         self.table_x1 + table_width + bin_width, self.table_y1 + table_height,
                                         fill=bin_color, outline='black', width=2),
            self.canvas.create_rectangle(self.table_x1, self.table_y1 + table_height,
                                         self.table_x1 + table_width, self.table_y1 + table_height + bin_width,
                                         fill=bin_color, outline='black', width=2),
            self.canvas.create_rectangle(self.table_x1 - bin_width, self.table_y1 + table_height,
                                         self.table_x1 + table_width + bin_width, self.table_y1 + table_height + bin_width,
                                         fill=bin_color, outline='black', width=2),
        ]

        pose_radius = 2
        for r in range(self.rows):
            for c in range(self.cols):
                x = self.transform_c(c)
                y = self.transform_r(r)
                self.environment.append(self.canvas.create_oval(x - pose_radius, y - pose_radius,
                                                                x + pose_radius, y + pose_radius, fill='black'))

    def draw_robot(self, r, c, color='white'):
        grasp_buffer = 5
        finger_length = self.side + grasp_buffer
        finger_width = 10
        gripper_length = 20
        gripper_width = self.side + 2 * self.block_buffer + finger_width
        stem_length = 50
        stem_width = 20

        x = self.transform_c(c)
        y = self.transform_r(r) - self.side / 2 - \
            gripper_length / 2 - grasp_buffer
        finger_x = gripper_width / 2 - finger_width / 2
        self.robot = [
            self.canvas.create_rectangle(x + finger_x - finger_width / 2., y,
                                         x + finger_x + finger_width / 2., y + finger_length,
                                         fill=color, outline='black', width=2),
            self.canvas.create_rectangle(x - finger_x - finger_width / 2., y,
                                         x - finger_x + finger_width / 2., y + finger_length,
                                         fill=color, outline='black', width=2),
            self.canvas.create_rectangle(x - stem_width / 2., y - stem_length,
                                         x + stem_width / 2., y,
                                         fill=color, outline='black', width=2),
            self.canvas.create_rectangle(x - gripper_width / 2., y - gripper_length / 2.,
                                         x + gripper_width / 2., y + gripper_length / 2.,
                                         fill=color, outline='black', width=2),
        ]

    def draw_block(self, r, c, color='red'):
        x = self.transform_c(c)
        y = self.transform_r(r)
        self.cells[(x, y)] = self.canvas.create_rectangle(x - self.side / 2., y - self.side / 2.,
                                                          x + self.side / 2., y + self.side / 2.,
                                                          fill=color, outline='black', width=2)

    def clear(self):
        self.canvas.delete('all')

    def save(self, filename):

        from PIL import ImageGrab
        ImageGrab.grab((0, 0, self.width, self.height)).save(filename + '.jpg')
