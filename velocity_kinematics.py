from sympy import *
from link import Link
from latex import print_simple, print_latex

def compute_angVel(links):                                                      
    print("\n####### Jw_1 #########\n")
    if links[0].link_type == "revolute":                                        
        Jw = [[0, 0, 1]]                                                        
        pprint(symbols("Z_0="))
    else:                                                                       
        Jw = [[0, 0, 0]]
    pprint(Jw[-1])
    for i in range(1, len(links)):                                              
        print("\n####### Jw_%d #########\n"%(i+1))
        if(links[i].link_type == "revolute"):                                   
            if(isParallelZ(links[i-1], links[i])):                              
                pprint(symbols("Z_%d="%i))
                Jw.append(Jw[-1])                                               
                pprint(Jw[-1])
                print_simple(Matrix(Jw[-1]), links)
                print_latex(Matrix(Jw[-1]), links)
            else:
                pprint(symbols("Z_%d="%i))
                Jw.append(links[i-1].getZ())
                pprint(Jw[-1])
                print_simple(Matrix(Jw[-1]), links)
                print_latex(Matrix(Jw[-1]), links)
        else:                                                                   
            Jw.append([0, 0, 0])                                                
            pprint(Jw[-1])
    return Matrix(Jw).T                                                         
                                                                                
def compute_linVel(links):
    print("\n* ---> cross product\n")
    n = len(links)
    symb_o_n = symbols("o_%d"%n)
    A_n = eye(4)                                                                
    for i in range(0, len(links)):                                              
        A_n = A_n*links[i].A                                                    
    o_n = trigsimp(A_n).col(3)[0:3]
    pprint(symbols("o_%d"%n))
    pprint(Matrix(o_n))
    print_simple(Matrix(o_n), links)
    print_latex(Matrix(o_n), links)
                                                                                
    print("\n######## Jv_1 #########\n")
    Z_0, o_0 = symbols("Z_0 o_0")
    if links[0].link_type == "revolute":
        pprint(Z_0 * (o_0 - symb_o_n) )
        Jv = trigsimp(Matrix([0, 0, 1]).cross(Matrix(o_n)))
    else:
        pprint(Z_0)
        Jv =  Matrix(links[0].getZ())                                             
    pprint(Jv)
    print_simple(Jv, links)
    print_latex(Jv, links)
                                                                                
    A_x = eye(4)                                                                
    for i in range(1, len(links)):                                              
        print("\n######## Jv_%d #########\n"%(i+1))
        z, o = symbols("Z_%d o_%d"%(i, i))
        A_x = A_x * links[i-1].A                                                
        if links[i].link_type == "revolute":                                    
            pprint(z * (o - symb_o_n)) 
            col = Matrix(A_x.col(2)[0:3]).cross(Matrix(o_n) - Matrix(A_x.col(3)[0:3]))
            Jv = Jv.col_insert(i, col)                                          
        else:                                                                   
            Jv = Jv.col_insert(i, Matrix(A_x.col(2)[0:3]))
            pprint(z)
        pprint(Jv.col(i))
        print_simple(Jv.col(i), links)
        print_latex(Jv.col(i), links)
    return trigsimp(Jv)                                                         
                                                                                
def compute_singularities(Jv):
    det = simplify(Jv.det())
    return det                                                  
                                                                                
                                                                                
def isParallelZ(link_1, link_2):                                                
    z_1 = link_1.getZ()                                                         
    z_2 = link_2.getZ()                                                         
    cross = Matrix(z_1).T.cross(Matrix(z_2).T)                                  
    if sqrt(cross[0]**2 + cross[1]**2 + cross[2]**2) == 0:                      
        return True                                                             
    return False                                                                

