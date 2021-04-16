from link import Link
from velocity_kinematics import *
from sympy import *


def compute_kinetic(link):
    return

def compute_potential(link, A, m): #sin(theta_1)*mgL_1 + sin(theta_1 + theta_2)*mgL_2 where theta is the joint angle from the ground/x_0 plane. A is the transformation matrix from link i to base
    if link.link_num == 1 or A.col(3)[0:3] == [0, 0, 1]: #checking if the Z axis of link i is parallel to Z_0
        return symbols('m_%d', link.link_num) * symbols('g') * link.link_length
   


def total_Kp(links, M): # M is a 1xN matrix with the mas of each link
    return

def total_Ke(links):
    return
