from sympy import *
from sympy.physics.vector import *
from velocity_kinematics import *
from dynamics import *
from link import Link


def inverse_kinematcs(T, links, x_val, y_val, z_val):
    DOF = len(links)
    unknowns = []
    system = []
    for link in links:
        if link.link_type == "prismatic":
            unknowns.append(link.link_offset)
        else:
            unknowns.append(link.joint_angle)
    x, y, z = symbols("x y z")

    system.append( Eq(x, T.col(3)[0]) )
    system.append( Eq(y, T.col(3)[1]) )
    system.append( Eq(z, T.col(3)[2]) )

    result = solve(system, unknowns, dict=True)
    pprint(result)

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
    g = 0   #gravity vector
    Ke = 0  #Total kinetic energy
    Pe = 0  #Total potential energy
  
    links.append( Link("revolute", 1, 0, pi/2, symbols('L_1'), dynamicsymbols('%s_1' %(theta))) )
    links.append( Link("revolute", 2, symbols('L_2'), 0, 0, dynamicsymbols('%s_2' %(theta))) )
    links.append( Link("revolute", 3, symbols('L_3'), 0, 0, dynamicsymbols('%s_3' %(theta))) )

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
    Jv = compute_linVel(links)
    Jw = compute_angVel(links)
    J = Jv 

    print("\n\n######## TRANSFORMATION MATRICIES #########I\n\n")
    for link in links: #print all the transformation matricies for each link
        print("A_%d\n"%link.link_num)
        pprint(link.A)
        
    print("\n\n######## FORWARD KINEMATICS  #########I\n\n")
    for link in links: 
        T = T * link.A
        for i in range(1, link.link_num + 1):
            print("A_%d"%i, end=" ")
        print("\n")
        pprint(trigsimp(T))
   

    print("############ ANGULAR VELOCITY #################\n\n")
    pprint(Jw)

    print("############ LINEAR VELOCITY #################\n\n")
    pprint(Jv)
    
    print("############ JACOBIAN #################\n\n")
    for i in range(1, 4):
        J = J.row_insert(3 + i, Jw.row(i-1))
    pprint(J)

    print("############ SINGULARITIES  #################\n\n")
    #det = compute_singularities(compute_linVel(links))
    #pprint(trigsimp(det))

    print("Ke: \n");
    D, Ke = compute_kinetic(J, links, DOF, q)
    C = compute_cristoffel(D, DOF, q)

