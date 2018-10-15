
import numpy as np
import random
import matplotlib.pyplot as plt

dt = 0.01
H = [
    [1, 0],
    [0, 1]
]
F = [
    [1, dt],
    [0, 1]
]
Q = [
    [0.001, 0],
    [0, 0.001]
]
R = [
    [1.5, 0],
    [0, 0.2]
]
P = R

class Truth:
    def __init__(self):
        self.position = 0
        self.velocity = 10

        self.position_variance = 1.5
        self.velocity_variance = 0.2

    def actual(self):
        return [
            [self.position],
            [self.velocity]
        ]

    def measure(self):
        return [
            [self.position + random.normalvariate(0, self.position_variance)],
            [self.velocity + random.normalvariate(0, self.velocity_variance)]
        ]

    def update(self, dt):
        self.position += self.velocity * dt

truth = Truth()

x = truth.measure()

t = 0
t_values = []
truth_values = []
estimated_values = []
measured_values = []

def iteration(truth):
    global x, P, t, t_values, truth_values, estimated_values, measured_values
    print("-------------------")
    actual = truth.actual()
    print("truth:\n{}".format(actual))
    z = truth.measure()
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
    P = np.matmul(np.matmul(np.identity(2) - np.matmul(K, H), P_predicted), np.matmul(K, np.matmul(R, np.transpose(K))))
    print("P:\n{}".format(P))
    post_fit_residual = z - np.matmul(H, x)
    print("post_fit_residual:\n{}".format(post_fit_residual))

    t_values.append(t)
    truth_values.append(actual[0])
    estimated_values.append(x[0])
    measured_values.append(z[0])

    t += dt
    truth.update(dt)

for i in range(100):
    iteration(truth)

plt.plot(t_values, truth_values, 'r', t_values, estimated_values, 'g', t_values, measured_values, 'b')
plt.show()
