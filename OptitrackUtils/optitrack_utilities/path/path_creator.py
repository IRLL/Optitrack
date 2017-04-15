import copy

from ..path.path_utilities import *

class PathCreator:
    def __init__(self, origin_rb = None, max_segment_length = 0.5, min_segment_length = 0.02):
        self.path = []
        self.origin_rb = origin_rb
        self.max_segment_length = max_segment_length
        self.min_segment_length = min_segment_length

    def get_path(self):
        return copy.copy(self.path)

    def add_point_helper(self, pt):
        if len(self.path) > 0:
            seg_length = euclid_distance(self.path[-1], pt)

            # If point to be added is too close to previous point: ignore.
            if seg_length < self.min_segment_length:
                return

            # If new segment is greater than max_segment_length recursively
            # split segment.
            if seg_length > self.max_segment_length:
                mid = bisect_line(self.path[-1], pt)
                self.add_point_helper(mid)
                self.add_point_helper(pt)
            else:
                self.path.append(pt)
        else:
            self.path.append(pt)

    def add_point(self, pt):
        if self.origin_rb:
            origin_coords = self.origin_rb.get_position()
            pt[0] -= origin_coords[0]
            pt[1] -= origin_coords[1]

        self.add_point_helper(pt)

    def finalize_path(self):
        self.add_point_helper(self.path[0])
        self.path = self.path[0:-1]

    def save_path(self, filename):
        self.finalize_path()

        outfile = open(filename, 'w')

        if self.origin_rb:
            origin_rb_name = "'" + self.origin_rb.name + "'"
        else:
            origin_rb_name = "''"

        outfile.write(origin_rb_name + "\n")

        for p in self.path:
            line = str(p[0]) + "," + str(p[1]) + "\n"
            outfile.write(line)

        outfile.close()

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
