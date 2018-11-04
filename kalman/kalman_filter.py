
import numpy as np
import random
import matplotlib.pyplot as plt
import csv

dt = 0.01
H = [
    [1]
]
F = [
    [1]
]
Q = [
    [0.001]
]
R = [
    [22.9]
]
P = R

# truth = Truth()

# x = truth.measure()
x = [
    [0]
]

t = 0
t_values = []
# truth_values = []
estimated_values = []
measured_values = []

def iteration(measurement):
    global x, P, t, t_values, truth_values, estimated_values, measured_values
    print("-------------------")
    z = [
        [measurement]
    ]
    print("z:\n{}".format(z))
    x_predicted = np.matmul(F, x)
    print("x_predicted:\n{}".format(x_predicted))
    P_predicted = np.matmul(np.matmul(F, P), np.linalg.inv(F)) + Q
    print("P_predicted:\n{}".format(P_predicted))
    innovation = z - np.matmul(H, x_predicted)
    print("innovation:\n{}".format(innovation))
    innovation_covariance = R + np.matmul(np.matmul(H, P_predicted), np.transpose(H))
    print("innovation_covariance:\n{}".format(innovation_covariance))
    K = np.matmul(np.matmul(P_predicted, np.transpose(H)), np.linalg.inv(innovation_covariance))
    print("K:\n{}".format(K))
    x = x_predicted + np.matmul(K, innovation)
    print("x:\n{}".format(x))
    P = np.matmul(np.matmul(np.identity(1) - np.matmul(K, H), P_predicted), np.matmul(K, np.matmul(R, np.transpose(K))))
    print("P:\n{}".format(P))
    post_fit_residual = z - np.matmul(H, x)
    print("post_fit_residual:\n{}".format(post_fit_residual))

    t_values.append(t)
    # truth_values.append(actual[0])
    estimated_values.append(x[0])
    measured_values.append(z[0])

    t += dt
    # truth.update(dt)

f = open('upright_with_motors.csv', 'r')
r = csv.DictReader(f)
for i, row in enumerate(r):
    iteration(float(row['error']))

plt.plot(t_values, estimated_values, 'g', t_values, measured_values, 'b')
plt.show()
