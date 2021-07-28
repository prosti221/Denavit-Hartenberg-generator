##Computes the forward kinematic equations, velocity kinematics and dynamics of a configuration based on the Denavit-Hartenberg convention.

#This takes the following inputs:
    *1. N number of links in configuration
    *2. The DH parameters of the link: link length, link twist, link offset and joint angle.
    *3. Mass and inertia tensors for the dynamics

The output is the product of N homogeneous transformation matricies A_0 * A_1 ... A_N.
You can calculate the velocity kinematics of the configuration using the fucntions compute_angVel() and compute_linVel().
The output will be two 3xn matricies: J_v and J_w that together forms the 6xn jacobian.
The functions compute_kinetic(), compute_cristoffel() and compute_g() can be used to get the equations of motion in the form of D(q) * v + C(q, v) * v + G(q) = t




