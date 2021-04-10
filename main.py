from sympy import *
import numpy as np

class Link:
    def __init__(self, link_type, link_num, link_length, link_twist, link_offset, joint_angle):
        self.link_type = link_type
        self.link_num = link_num
        self.link_length = link_length
        self.link_twist = link_twist
        self.link_offset = link_offset
        self.joint_angle = joint_angle
    
    def compute_num_DH(self):
        A = Matrix([[cos(self.joint_angle), -sin(self.joint_angle)*cos(self.link_twist) , sin(self.joint_angle)*sin(self.link_twist), self.link_length * cos(self.joint_angle)],  
                    [sin(self.joint_angle), cos(self.joint_angle)*cos(self.link_twist) , -cos(self.joint_angle)*sin(self.link_twist), self.link_length * sin(self.joint_angle)],
                    [0, sin(self.link_twist), cos(self.link_twist), self.link_offset],
                    [0, 0, 0, 1]])
        return A
    
if __name__ == "__main__":
    #Link(link_type, link_num, link_length, link_twist, link_offset, joint_angle)
    #use var('x') for symbols
    test = Link("prismatic", 1, 0, 0, var('a_5') + var('d_3'), 0)
    pprint(test.compute_num_DH())
