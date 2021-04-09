Computes the forward kinematic equations based on the Denavit-Hartenberg convention.

This takes the following inputs:
    1. N number of links in configuration
    2. The DH parameters of the link: link length, link twist, link offset and joint angle.

The output is the product of N homogeneous transformation matricies A_0 * A_1 ... A_N. The result can be presented numerically or algebraically.

