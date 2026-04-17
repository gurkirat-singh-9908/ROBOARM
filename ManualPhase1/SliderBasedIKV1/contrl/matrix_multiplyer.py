#use this to solve the matrix calculations for rotation matrixes 
import sympy as sp 
from sympy import pi

theeta = sp.symbols('t1 t2 t3 t4 t5 t6')
link = sp.symbols('a0 a1 a2 a3 a4 a5')

def dh_transform(val):
    theta, alpha, r, d = val
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), r * sp.cos(theta)],
        [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), r * sp.sin(theta)],
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1]
    ])

#dh table has been updated for inverse 

dh_matrix = [
        [theeta[0]+(5.710*pi)/180,          -pi/2,                  0,                            link[0]],
        [theeta[1]+(5.739*pi)/180,           0,                    -link[1],                      0],
        [theeta[2]-(15.25*pi)/180,          pi/2,                  0,                            0],
        [theeta[3]-pi/2,                   -pi/2,                  0,                          -(link[2]+link[3])],
        [theeta[4]-pi/2,                   -pi/2,                  0,                            0]              ,
        [theeta[5]+0,                        0,                     0,                            link[-2]+link[-1]]
    ]

def matrix_multiplyer(start_frame, end_frame):
    T = sp.eye(4)  # Identity matrix for initial transformation
    for i in range(start_frame, end_frame):
        T = T * dh_transform(dh_matrix[i])
    return T


def main():
    #after you make your matrix ask chat gpt to convert it into the mathematical form  
    T_03 = matrix_multiplyer(0, 3)

    T_36 = matrix_multiplyer(3, 6)
    print("\n")
    #sp.pprint(T_03, use_unicode=True)
    sp.pprint(T_36, use_unicode=True)

if __name__ == "__main__":
    main()
