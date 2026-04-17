#inverse kinematics 
import numpy as np 
from numpy import pi,sqrt,square as sq ,sin,cos,arctan2,arccos,abs,rad2deg as c2d

desrired_matrix = np.array([
 [-6.1232340e-17, -6.1232340e-17,  1.0000000e+00,  2.3250000e+01],
 [ 1.8369702e-16, -1.0000000e+00, -6.1232340e-17, -6.0500000e+01],
 [ 1.0000000e+00,  1.8369702e-16,  6.1232340e-17,  2.0500000e+01],
 [ 0.0000000e+00,  0.0000000e+00,  0.0000000e+00,  1.0000000e+00]])

wrist_pos = np.array([
    [desrired_matrix[0,3]-desrired_matrix[0,2]*32.5],
    [desrired_matrix[1,3]-desrired_matrix[1,2]*32.5],
    [desrired_matrix[2,3]-desrired_matrix[2,2]*32.5]
])

theeta1 = arctan2(wrist_pos[1],wrist_pos[0])

r1 = sqrt(sq(wrist_pos[0])+sq(wrist_pos[1]))

r2 = wrist_pos[2] - 20.5

pi2 = arctan2(r2,r1)

r3 = sqrt(sq(r1)+sq(r2))

ca = 32.5

pi3 = arccos(sq(r3)+sq(28)-sq(ca),2*r3*28)

#elbow up 

theeta3 = pi3 - pi

pi1 = arctan2(ca*sin(abs(theeta3)),28+ca*cos(theeta3))

theeta2 = pi2 - pi1

print(c2d(theeta1),c2d(theeta2),c2d(theeta3))