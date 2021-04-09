import numpy as np
from sympy import *

link_num = 2
var('a_%d c_%d s_%d d_%d' % (link_num, link_num, link_num, link_num))
A = Matrix([[eval('a_%d' % link_num), 2, 2], [2, 3, 4]])

pprint(A)
