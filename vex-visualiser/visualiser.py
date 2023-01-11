from matplotlib import pyplot as plt
import math
import random

class PlotOdometry:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        plt.subplots_adjust(left=0.25)
        self.ax.set_aspect('equal', adjustable='box')
        plt.ion()

        self.pos_text = plt.gcf().text(0.02, 0.5, f"pos:", fontsize=12)
        self.angle_text = plt.gcf().text(0.02, 0.4, f"theta:", fontsize=12)

    def plot(self, x, y, theta, pointer_length=0.5):
        plt.gca().clear()
        self.ax.axis([0, 12, 0, 12])
        self.ax.grid()

        theta = 90 - theta   # up is "zero"
        self.ax.plot(x, y, marker="o")
        x1 = x + pointer_length * math.cos(math.radians(theta))
        y1 = y + pointer_length * math.sin(math.radians(theta))
        self.ax.plot([x, x1],[y, y1], 'k-', lw=1)

        self.pos_text.set_text(f"pos: {x}, {y}")
        self.angle_text.set_text(f"theta: {theta}")

        plt.show()
        self.fig.canvas.flush_events()