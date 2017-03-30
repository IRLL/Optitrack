from path_utilities import *

class ENGR120_Grader:
    def __init__(self, base_path, robot_path):
        self.base_path = base_path
        self.robot_path = robot_path
        self.errors = []

    def grade(self):
        for pt in self.robot_path:
            _, distance = get_closest_point_and_distance_from_path(pt, self.base_path)
            self.errors.append(distance * 100.0)

        print
        print "********************************"
        print "Avg (Absolute) Error (cm): ",
        print sum([abs(i) for i in self.errors]) / len(self.errors)
        print "Standard Deviation (cm): ",
        print (sum([i ** 2 for i in self.errors]) / len(self.errors)) ** 0.5
        print "********************************"
        print
