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
        var('a_%d c_%d s_%d d_%d' % (self.link_num, self.link_num, self.link_num, self.link_num))
    
    def compute_num_DH():
        return A
    
    def compute_alg_DH(self):
        if self.link_type == "revolute":
            A = Matrix([[eval('c_%d'%self.link_num), -eval('s_%d'%self.link_num)*np.cos(self.link_twist), eval('s_%d'%self.link_num)*np.sin(self.link_twist), eval('a_%d*c_%d' % (self.link_num, self.link_num))], 
                        [eval('s_%d'%self.link_num), eval('c_%d'%self.link_num)*np.cos(self.link_twist), -eval('c_%d'%self.link_num)*np.sin(self.link_twist), eval('a_%d*s_%d' % (self.link_num, self.link_num))], 
                        [0, np.sin(self.link_twist), np.cos(self.link_twist), self.link_offset],
                        [0, 0, 0, 1]])
            return A
        if self.link_type == "prismatic":
            return A


if __name__ == "__main__":
    test = Link("revolute", 1, 0, -np.pi/2, var('L_1'), 0)
    pprint(test.compute_alg_DH())
