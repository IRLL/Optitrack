import copy
import warnings

import matplotlib.cbook
import matplotlib.patches as patches
import matplotlib.pyplot as plt

from matplotlib.path import Path

warnings.filterwarnings("ignore", category=matplotlib.cbook.MatplotlibDeprecationWarning)
warnings.filterwarnings("ignore", category=matplotlib.cbook.mplDeprecation)

"""
def fxn():
    warnings.warn("deprecated", matplotlib.cbook.mplDeprecation)

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    fxn()
"""

class PathPlotter:
    def __init__(self):
        self.fig = plt.figure()
        self.base_subplot = self.fig.add_subplot(1, 1, 1)
        #self.robot_subplot = self.fig.add_subplot(1, 1, 1)

    def input_base_path(self, path):
        self.base_path = copy.copy(path)

        if len(self.base_path) == 0:
            return

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

        self.base_subplot.cla()
        self.base_subplot.add_patch(self.base_patch)

        xs = [i[0] for i in verts]
        ys = [i[1] for i in verts]

        self.base_subplot.set_xlim(min(xs) - 0.5, max(xs) + 0.5)
        self.base_subplot.set_ylim(min(ys) - 0.5, max(ys) + 0.5)
        #self.robot_subplot.set_xlim(min(xs) - 0.5, max(xs) + 0.5)
        #self.robot_subplot.set_ylim(min(ys) - 0.5, max(ys) + 0.5)

    def input_robot_path(self, path):
        self.robot_path = copy.copy(path)

        if len(self.robot_path) == 0:
            return

        verts = self.robot_path
        codes = [Path.MOVETO]

        for i in range(len(verts) - 1):
            codes.append(Path.LINETO)

        self.mpl_robot_path = Path(verts, codes)

        self.robot_patch = patches.PathPatch(self.mpl_robot_path, edgecolor='green', facecolor='orange', lw=2)
        self.robot_patch.set_fill(False)

        #self.robot_subplot.cla()
        #self.robot_subplot.add_patch(self.robot_patch)
        self.base_subplot.add_patch(self.robot_patch)

    def input_closest_point_on_path(self, pt):
        verts = [pt, self.robot_path[-1]]
        codes = [Path.MOVETO, Path.LINETO]

        self.mpl_difference_path = Path(verts, codes)

        self.diff_patch = patches.PathPatch(self.mpl_difference_path, edgecolor='red', facecolor='orange', lw=2)
        self.diff_patch.set_fill(False)

        self.base_subplot.add_patch(self.diff_patch)

    def draw(self, block=False):
        if block:
            plt.show()
        else:
            plt.draw()
            plt.pause(0.01)
