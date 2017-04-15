import copy

from ..path.path_plotter import PathPlotter
from ..optitrack_bridge.optitrack_bridge import OptitrackBridge
from ..path.path_utilities import *

class Path2D:
    """
    Stores a 2d path of points as a list. The path can be defined using global
    coordinates or as coordinates defined relative to an origin.

    Attributes
    ----------
    path : list of 2d points
    rigid_bodies : list of RigidBody
    """
    def __init__(self):
        self.path = []
        self.origin_rb = None

    def set_origin_name(self, name):
        if name:
            self.origin_name = name
            self.opti_utils = OptitrackBridge([name])
            self.origin_rb = self.opti_utils.get_rigid_body(name)

    def open_path(self, filename):
        outfile = open(filename, 'r')

        origin_name = outfile.readline()

        if origin_name != "''":
            self.set_origin_name(origin_name[1:-2])

        for line in outfile.readlines():
            pts = line.split(",")
            self.path.append( [float(pts[0]), float(pts[1])] )

        outfile.close()

    def plot(self):
        plotter = PathPlotter()
        plotter.input_base_path(self.path)
        plotter.draw(block=True)

    def add_point(self, pt):
        if self.origin_rb:
            pos = self.origin_rb.get_position()[0:2]
            pt[0] -= pos[0]
            pt[1] -= pos[1]

        self.path.append(pt)

    def get_path(self):
        return copy.copy(self.path)

    def __repr__(self):
        string = ""
        string += "Num Points: " + str(len(self.path)) + "\n"
        for i, p in enumerate(self.path):
            string += str(i) + ". " + str(p) + "\n"
        return string

    def __str__(self):
        string = ""
        string += "Num Points: " + str(len(self.path)) + "\n"
        for i, p in enumerate(self.path):
            string += str(i) + ". " + str(p) + "\n"
        return string

    def __iter__(self):
        for p in self.path:
            yield p

    def __getitem__(self, val):
        return self.path[val]

    def __len__(self):
        return len(self.path)

    def append(self, val):
        if not isinstance(val, tuple) or not len(val) == 2:
            raise ValueError("input must be a tuple of length 2")

        self.path.append(val)
