from sympy import symbols, cos, sin, Matrix,pprint
import numpy as np


r = symbols('r') #roll
p = symbols('p') #pitch
y = symbols('y') # yaw

roll = Matrix([
    [cos(r), -sin(r), 0],
    [sin(r),  cos(r), 0],
    [0,      0,       1]
])

# Pitch matrix (rotation about y-axis)
pitch = Matrix([
    [ cos(p), 0, sin(p)],
    [ 0,      1, 0    ],
    [-sin(p), 0, cos(p)]
])

# Yaw matrix (rotation about x-axis)
yaw = Matrix([
    [1,      0,       0],
    [0, cos(y), -sin(y)],
    [0, sin(y),  cos(y)]
])

final_rot = yaw * pitch*roll;
final_rot = final_rot
variables = Matrix([r])

J = final_rot.jacobian(variables)
pprint(J)