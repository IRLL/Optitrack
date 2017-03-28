import matplotlib.patches as patches
import matplotlib.pyplot as plt

from matplotlib.path import Path

class PathPlotter:
    def __init__(self, base_path):
        self.base_path = base_path
        self.robot_path = []
        self.robot_verts = []
        self.robot_codes = []

        self.fig = plt.figure()
        self.base_subplot = self.fig.add_subplot(111)
        self.robot_subplot = self.fig.add_subplot(111)

        verts = self.base_path
        codes = [Path.MOVETO]
        for i in range(len(verts) - 1):
            codes.append(Path.LINETO)

        # Close path
        verts.append((0.0, 0.0))
        codes.append(Path.CLOSEPOLY)

        self.mpl_base_path = Path(verts, codes)
        self.base_patch = patches.PathPatch(self.mpl_base_path, facecolor='orange', lw=2)
        self.base_patch.set_fill(False)
        self.base_subplot.add_patch(self.base_patch)

        xs = [i[0] for i in verts]
        ys = [i[1] for i in verts]

        self.base_subplot.set_xlim(min(xs) - 0.5, max(xs) + 0.5)
        self.base_subplot.set_ylim(min(ys) - 0.5, max(ys) + 0.5)
        self.robot_subplot.set_xlim(min(xs) - 0.5, max(xs) + 0.5)
        self.robot_subplot.set_ylim(min(ys) - 0.5, max(ys) + 0.5)

    def draw_robot_path(self):
        plt.draw()
        plt.pause(0.01)

    def draw_base_path(self):
        plt.plot()
        #plt.draw()
        #plt.pause(1.0)

    def add_robot_location(self, location):
        self.robot_path.append(location)
        self.robot_verts.append(location)

        if len(self.robot_path) == 1:
            self.robot_codes.append(Path.MOVETO)
            return
        else:
            self.robot_codes.append(Path.LINETO)

        self.robot_subplot.cla()
        self.mpl_robot_path = Path(self.robot_verts, self.robot_codes)
        self.robot_patch = patches.PathPatch(self.mpl_robot_path, edgecolor='green', facecolor='orange', lw=2)
        self.robot_patch.set_fill(False)
        self.base_subplot.add_patch(self.base_patch)
        self.robot_subplot.add_patch(self.robot_patch)
