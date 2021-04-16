from sympy import *
from velocity_kinematics import *
from dynamics import *
from link import Link


if __name__ == "__main__":
    #Link(link_type, link_num(i), link_length(a), link_twist(alpha), link_offset(d), joint_angle(theta))
    #use symmbols('x') for symbols
    theta = '\u03B8'
    links = []
    links.append( Link("revolute", 1, 0, -pi/2, symbols('L_1'), symbols('%s_1' %(theta))) )
    links.append( Link("revolute", 2, symbols('L_2'), 0, 0, symbols('%s_2' %(theta))) )
    links.append( Link("revolute", 3, symbols('L_3'), 0, 0, symbols('%s_3' %(theta))) )

    T = eye(4)
    Jv = compute_linVel(links)
    Jw = compute_angVel(links)
    J = Jv 
    for link in links: #print all the transformation matricies for each link
        print("A_%d\n"%link.link_num)
        pprint(link.A)
        
    print("The forward kinematics: \n")
    for link in links: 
        T = T * link.A
        for i in range(1, link.link_num + 1):
            print("A_%d"%i, end=" ")
        print("\n")
        pprint(trigsimp(T))
    
    print("Angular velocity Jw: \n")
    pprint(compute_angVel(links))
    print("Linear velocity Jv: \n")
    pprint(compute_linVel(links))
    print("The complete Jacobian J(q): \n") 
    for i in range(1, 4):
        J = J.row_insert(3 + i, Jw.row(i-1))
    pprint(J)

    print("Singularities: \n")
    #det = compute_singularities(compute_linVel(links))
    #pprint(trigsimp(det))
