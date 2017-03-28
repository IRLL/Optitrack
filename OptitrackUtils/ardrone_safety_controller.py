#!/usr/bin/env python

class ArdroneSafetyController:
    def __init__(self, rigid_body, boundaries = [[-1, 1], [-1, 1], [-1, 1]]):
        self.rigid_body = rigid_body
        self.min_x = boundaries[0][0]
        self.max_x = boundaries[0][1]
        self.min_y = boundaries[1][0]
        self.max_y = boundaries[1][1]
        self.min_z = boundaries[2][0]
        self.max_z = boundaries[2][1]

        self.command = None

    def set_command(self, command):
        self.command = command

    def run(self):


