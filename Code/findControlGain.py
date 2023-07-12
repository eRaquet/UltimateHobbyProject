import numpy as np
import scipy.linalg as sp

#simulation constants
R_beam = 0.06
R_wheel = 0.041
R_acc = 0.024
M_beam = 0.997
g = 9.81
I_beam = 0.007483920614471
St_acc = 0

#state values
states = np.zeros(7)

#dynamics matrices
A = np.array([[0, 1, 0, 0], [0, 0, (M_beam*R_beam**2*g + M_beam*R_beam*R_wheel*g)/(M_beam*R_beam**2 + 2*M_beam*R_beam*R_wheel + M_beam*R_wheel**2 + I_beam), 0], [0, 0, 0, 1], [0, 0, M_beam*R_beam*g/(M_beam*R_beam**2 + 2*M_beam*R_beam*R_wheel + M_beam*R_wheel**2 + I_beam), 0]])
B = np.array([[0], [R_wheel*I_beam/(M_beam*R_beam**2 + 2*M_beam*R_beam*R_wheel + M_beam*R_wheel**2 + I_beam)], [0], [M_beam*(-R_beam*R_wheel - R_wheel**2)/(M_beam*R_beam**2 + 2*M_beam*R_beam*R_wheel + M_beam*R_wheel**2 + I_beam)]])

#state cost weights (total cost = Q @ states + R @ St_acc)
Q = np.eye(4)
Q[0][0], Q[1][1], Q[2][2], Q[3][3] = 30000.0, 0.03, 60.0, 0.5
R = np.eye(1)

#controler setup
X = sp.solve_continuous_are(A, B, Q, R)
K = np.dot(np.linalg.inv(R), (np.dot(B.T, X)))[0]

#controller works as follows (St_acc = -K @ states)

print(K)