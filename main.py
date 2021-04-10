from sympy import *

class Link:
    def __init__(self, link_type, link_num, link_length, link_twist, link_offset, joint_angle):
        self.link_type = link_type
        self.link_num = link_num
        self.link_length = link_length
        self.link_twist = link_twist
        self.link_offset = link_offset
        self.joint_angle = joint_angle
    
    def compute_num_DH(self, round=False):
        A = Matrix([[cos(self.joint_angle), -sin(self.joint_angle)*cos(self.link_twist) , sin(self.joint_angle)*sin(self.link_twist), self.link_length * cos(self.joint_angle)],  
                    [sin(self.joint_angle), cos(self.joint_angle)*cos(self.link_twist) , -cos(self.joint_angle)*sin(self.link_twist), self.link_length * sin(self.joint_angle)],
                    [0, sin(self.link_twist), cos(self.link_twist), self.link_offset],
                    [0, 0, 0, 1]])
        if round: return self.round_matrix(A)
        return A

    def round_matrix(self, A):
        for i in range(A.shape[0]):
            for j in range(A.shape[1]):
                if (isinstance(A[i,j], Float) and A[i,j] < 0.0000001):
                    A[i,j] = 0
        return A
    
if __name__ == "__main__":
    #Link(link_type, link_num(i), link_length(a), link_twist(alpha), link_offset(d), joint_angle(theta))
    #use var('x') for symbols
    theta = '\u03B8'
    link1 = Link("revolute", 1, 0, -pi/2, symbols('L_1'), symbols('%s_1' %(theta)))
    link2 = Link("prismatic", 2, symbols('L_2'), 0, 0, symbols('%s_2' %(theta)))
    link3 = Link("prismatic", 3, symbols('L_3'), 0, 0, symbols('%s_3' %(theta)))


    A1 = link1.compute_num_DH(round=False)
    A2 = link2.compute_num_DH(round=False)
    A3 = link3.compute_num_DH(round=False)

    pprint(A1)
    pprint(A2)
    pprint(A3)
    pprint(A1*A2)
    pprint(trigsimp(A1*A2*A3))

