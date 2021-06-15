from sympy import *
from sympy.physics.vector import *
from velocity_kinematics import *
from dynamics import *
from link import Link
from latex import print_simple, print_latex

if __name__ == "__main__":
    #Link(link_type, link_num(i), link_length(a), link_twist(alpha), link_offset(d), joint_angle(theta))
    #use symmbols('x') for symbols
    #Always give angles in terms of pi
    
    #Configuration
    theta = '\u03B8'
    q = []      #joint variables
    DOF = 0     #Degrees of freedom
    links = []  #Link objects representing each link of the robot and its DH parameters

    #Forward kinematics
    T = eye(4)  #Transformation matrix from base to end effector

    #Velocity kinematics
    Jv = 0  #Linear component of the jacobian
    Jw = 0  #Angular component of the jacobian
    J = 0   #The complete jacobian [Jv Jw] (6xDOF)

    #Dynamics
    D = 0   #Inertia matrix
    C = 0   #Coriolis/centrifugal matrix
    Ke = 0  #Total kinetic energy
  
    #Example configuration of a 3DOF RRP manipulator 
    links.append( Link(link_type="revolute", link_num=1, link_length=0, link_twist=-pi/2, link_offset=symbols('L_1'), joint_angle=symbols('%s_1' %(theta)) ) )
    links.append( Link(link_type="revolute", link_num=2, link_length=symbols('L_2'), link_twist=pi/2, link_offset=0, joint_angle=symbols('%s_2' %(theta))) )
    links.append( Link(link_type="prismatic", link_num=3, link_length=0, link_twist=0, link_offset=symbols('L_3'), joint_angle=0) )
        
    DOF = len(links)

    #Set the masses if needed
    links[0].m = symbols("m_1")
    links[1].m = symbols("m_2")
    #links[2].m = symbols("m_3")

    #Fill q                                                           
    for i in range(DOF):                                                        
        if links[i].link_type == "prismatic":                                    
            q.append(links[i].link_offset)                                      
        else:                                                                   
            q.append(links[i].joint_angle) 
   
    q = Matrix(q)
    
    T = eye(4)

    print("\n\n######## TRANSFORMATION MATRICIES #########\n\n")
    for link in links:
        print("A_%d\n"%link.link_num)
        pprint(link.A)
        print_latex(link.A, links)
        
    print("\n\n######## FORWARD KINEMATICS  #########\n\n")
    for link in links: 
        T = T * link.A
        for i in range(1, link.link_num + 1):
            print("A_%d"%i, end=" ")
        print("\n")
        pprint(trigsimp(T))
        print_latex(trigsimp(T), links)
  
    Jv = compute_linVel(links)
    Jw = compute_angVel(links)
    J = Jv 

    print("############ ANGULAR VELOCITY #################\n\n")
    pprint(Jw)
    print_latex(Jw, links)

    print("############ LINEAR VELOCITY #################\n\n")
    pprint(Jv)
    print_latex(Jv, links)
    
    print("############ JACOBIAN #################\n\n")
    for i in range(1, 4):
        J = J.row_insert(3 + i, Jw.row(i-1))
    pprint(J)
    print_latex(J, links)

    print("############ Determinant of Jv  #################\n\n")
    #det = compute_singularities(Jv)
    #pprint(det)
    
    #Dynamics
    D, Ke = compute_kinetic(J, links, DOF, q)
    C = compute_cristoffel(D, DOF, q)

