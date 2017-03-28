from path_plotter import *
from path_utilities import *

class Path2D:
    def __init__(self, max_segment_length, origin=None):
        self.path = []
        self.origin = origin
        self.max_segment_length = max_segment_length

    def add_point(self, pt):
        # Check if added line seg is longer than max segment length
        if len(self.path) > 0:
            seg_length = euclid_distance(self.path[-1], pt)
            print "seg_length: ",
            print seg_length
            if seg_length > self.max_segment_length:
                mid = bisect_line(self.path[-1], pt)
                self.add_point(mid)
                self.add_point(pt)
            else:
                self.path.append(pt)
        else:
            self.path.append(pt)

    def save_path(self, filename):
        outfile = open(filename, 'w')

        for p in self.path:
            line = str(p[0]) + "," + str(p[1]) + "\n"
            outfile.write(line)

        outfile.close()

    def draw_base_path(self):
        self.p = PathPlotter(self.path)
        self.p.draw_base_path()

    def draw_robot_location(self, location):
        self.p.add_robot_location(location)
        self.p.draw_robot_path()

    def open_path(self, filename):
        outfile = open(filename, 'r')

        for line in outfile.readlines():
            pts = line.split(",")
            self.add_point( (float(pts[0]), float(pts[1])) )

        outfile.close()

    def find_distance_from_path(self, location):
        _, distance = distance_from_path(location, self.path)
        return distance

    def get_closest_path_point(self, location):
        closest, _ = distance_from_path(location, self.path)
        return closest

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
