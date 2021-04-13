from sympy import *

class Link:
    def __init__(self, link_type, link_num, link_length, link_twist, link_offset, joint_angle):
        self.link_type = link_type
        self.link_num = link_num
        self.link_length = link_length
        self.link_twist = link_twist
        self.link_offset = link_offset
        self.joint_angle = joint_angle
    
        self.A = Matrix([[cos(self.joint_angle), -sin(self.joint_angle)*cos(self.link_twist) , sin(self.joint_angle)*sin(self.link_twist), self.link_length * cos(self.joint_angle)],  
                    [sin(self.joint_angle), cos(self.joint_angle)*cos(self.link_twist) , -cos(self.joint_angle)*sin(self.link_twist), self.link_length * sin(self.joint_angle)],
                    [0, sin(self.link_twist), cos(self.link_twist), self.link_offset],
                    [0, 0, 0, 1]])
    def getZ(self):
        return self.A.col(2)[0:3]

    def getOrigin(self):
        return self.A.col(3)[0:3]

def compute_angVel(links):
    if links[0].link_type == "revolute":  
        Jw = [[0, 0, 1]]
    else:
        Jw = [[0, 0, 0]]
    for i in range(1, len(links)):
        if(links[i].link_type == "revolute"):
            if(isParallelZ(links[i-1], links[i])):
                Jw.append(Jw[-1])
            else:
                Jw.append(links[i-1].getZ())
        else:
            Jw.append([0, 0, 0])
    return Matrix(Jw).T

def compute_linVel(links):
    A_n = eye(4)
    for i in range(0, len(links)):
        A_n = A_n*links[i].A
    o_n = trigsimp(A_n).col(3)[0:3]

    if links[0].link_type == "revolute":
        Jv = trigsimp(Matrix([0, 0, 1]).cross(Matrix(o_n)))
    else:
        Jv =  Matrix(links[0].getZ)
    
    A_x = eye(4)
    for i in range(1, len(links)):
        A_x = A_x * links[i-1].A
        if links[i].link_type == "revolute":
            col = Matrix(A_x.col(2)[0:3]).cross(Matrix(o_n) - Matrix(A_x.col(3)[0:3]))
            Jv = Jv.col_insert(i, col)
        else:
            Jv = Jv.col_insert(1, A_x.col(2)[0:3])
    return trigsimp(Jv)

def isParallelZ(link_1, link_2):
    z_1 = link_1.getZ()
    z_2 = link_2.getZ()
    cross = Matrix(z_1).T.cross(Matrix(z_2).T)
    if sqrt(cross[0]**2 + cross[1]**2 + cross[2]**2) == 0:
        return True
    return False

if __name__ == "__main__":
    #Link(link_type, link_num(i), link_length(a), link_twist(alpha), link_offset(d), joint_angle(theta))
    #use symmbols('x') for symbols
    theta = '\u03B8'
    links = []
    links.append( Link("revolute", 1, 0, -pi/2, symbols('L_1'), symbols('%s_1' %(theta))) )
    links.append( Link("revolute", 2, symbols('L_2'), 0, 0, symbols('%s_2' %(theta))) )
    links.append( Link("revolute", 3, symbols('L_3'), 0, 0, symbols('%s_3' %(theta))) )

    A1 = links[0].A
    A2 = links[1].A
    A3 = links[2].A

    pprint(A1)
    pprint(A2)
    pprint(A3)
    #pprint(A1*A2)
    #pprint(trigsimp(A1*A2*A3))
    #pprint(compute_angVel(links))
    pprint(compute_linVel(links))
