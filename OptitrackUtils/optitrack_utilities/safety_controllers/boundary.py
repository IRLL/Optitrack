from math import fabs

class Boundary:
    def __init__(self, shape, center, dims):
        self.shape = shape
        self.center = center
        self.dims = dims

    def within_bounds(self, pos):
        return True

class BoxBoundary(Boundary):
    def __init__(self, center, dims):
        Boundary.__init__(self, "box", center, dims)

    def within_bounds(self, pos):
        if fabs(pos[0] - self.center[0]) > (self.dims[0] / 2.0):
            return False
        if fabs(pos[1] - self.center[1]) > (self.dims[1] / 2.0):
            return False
        if fabs(pos[2] - self.center[2]) > (self.dims[2] / 2.0):
            return False

        return True

class SphereBoundary(Boundary):
    def __init__(self, center, dims):
        Boundary.__init__(self, "sphere", center, dims)

    def within_bounds(self, pos):
        if fabs(pos[0] - self.center[0]) > self.dims[0]:
            return False
        if fabs(pos[1] - self.center[1]) > self.dims[0]:
            return False
        if fabs(pos[2] - self.center[2]) > self.dims[0]:
            return False

        return True

