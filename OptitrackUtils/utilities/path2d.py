import copy

from path_plotter import *
from optitrack_utilities import OptitrackUtilities
from path_utilities import *

class Path2D:
    def __init__(self, filename=None, origin_name=None):
        self.path = []
        self.filename = filename
        self.origin_name = origin_name
        self.origin_rb = None

        if self.filename:
            self.open_path(filename)

        if self.origin_name:
            self.set_origin_name(self.origin_name)

    def set_origin_name(self, name):
        if name:
            self.origin_name = name
            self.opti_utils = OptitrackUtilities([name])
            self.origin_rb = self.opti_utils.get_rigid_body(name)

    def open_path(self, filename):
        print "open_path"
        print "filename",
        print filename
        outfile = open(filename, 'r')

        origin_name = outfile.readline()
        print "origin_name",
        print origin_name

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
