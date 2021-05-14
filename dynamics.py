from link import Link
from velocity_kinematics import *
from sympy import *


def compute_kinetic(J,links, DOF, q):
    DOF = len(links)
    q_dot = q.diff()
    D = zeros(DOF, DOF)
    k = 0

    #Sepereate jacobian
    Jv = J[:3,:DOF]
    Jw = J[3:6,:DOF]
    Jv_list = [] #Linear velocities
    Jw_list = [] #Angular velocities

    R_list = [links[0].A[:3, :3]] #Rotational matricies
    
    I_list = []
    for i in range(DOF):
        I_list.append(Matrix([[Symbol("I_%d,x"%(i+1)), 0, 0],
                              [0, Symbol("I_%d,y"%(i+1)), 0],
                              [0, 0, Symbol("I_%d,z"%(i+1))]]))
        if i > 0:
            R_list.append(simplify(R_list[-1] * links[i].A[:3, :3]))

    print("\n######### ROTATION MATRICIES #########\n")
    pprint(R_list)
    print("\n######### INERTIA TENSOR MATRICIES #########\n")
    pprint(I_list)
    #Substitute independent terms and columns
    for i in range(DOF - 1):
        v = Jv 
        w = Jw
        for j in range(i + 1, DOF):
            v = v.col_del(j); v = v.col_insert(j, zeros(3, 1))
            w = w.col_del(j); w = w.col_insert(j, zeros(3, 1))
            try:
                v = v.subs(links[j].link_offset, 0)
            except Exception as e:
                print(e)
            try:
                v = v.subs(links[j].joint_angle, 0)
            except Exception as e:
                print(e)
            try:
                w = w.subs(links[j].link_offset, 0)
            except Exception as e:
                print(e)
            try:
                w = w.subs(links[j].joint_angle, 0)
            except Exception as e:
                print(e)
            try:
                v = v.subs(links[j].link_length, 0)
                w = w.subs(links[j].link_length, 0)
            except Exception as e:
                print(e)
        Jv_list.append(v)
        Jw_list.append(w)

    Jv_list.append(Jv)
    Jw_list.append(Jw)

    print("\n######### LINEAR VELOCITIES (Jv)  #########\n")
    pprint(Jw_list)
    print("\n######### ANGULAR VELOCITIES (Jw)  #########\n")
    pprint(Jv_list)
    
    #Summing the kinetic energy of each link together
    for i in range(DOF):
        lin_vel = links[i].m * Jv_list[i].T * Jv_list[i]
        ang_vel = Jw_list[i].T * R_list[i] * I_list[i] * R_list[i].T * Jw_list[i]
        D += lin_vel + ang_vel
     
    print("\n######### INERTIA MATRIX (D)  #########\n")
    pprint(simplify(D))
    print("\n######### TOTAL KINETIC ENERGY (Ke)  #########\n")
    k = simplify( (1/2) * (q_dot.T * D * q_dot)  )
    pprint(k)

    return D, k

def compute_cristoffel(D, DOF, q):
    t = symbols("t")
    C = zeros(DOF, DOF)
    for k in range(DOF):
        for j in range(DOF):
            for i in range(DOF):
                C[k, j] += (1/2) * (D[k, j].diff(q[i]) + D[k, i].diff(q[j]) - D[i, j].diff(q[k])) * q[i].diff(t)
    C = simplify(C)
    pprint(C)
    return C

def compute_potential(link, A, m): #sin(theta_1)*mgL_1 + sin(theta_1 + theta_2)*mgL_2 where theta is the joint angle from the ground/x_0 plane. A is the transformation matrix from link i to base
    if link.link_num == 1 or A.col(3)[0:3] == [0, 0, 1]: #checking if the Z axis of link i is parallel to Z_0
        return symbols('m_%d', link.link_num) * symbols('g') * link.link_length
   
