import numpy as np

def quaternProd(a, b):
    # ab = np.zeros((len(a), 4))
    # ab[:, 0] = a[:, 0]*b[:, 0] - a[:, 1]*b[:, 1] - a[:, 2]*b[:, 2] - a[:, 3]*b[:, 3]
    # ab[:, 1] = a[:, 0]*b[:, 1] + a[:, 1]*b[:, 0] + a[:, 2]*b[:, 3] - a[:, 3]*b[:, 2]
    # ab[:, 2] = a[:, 0]*b[:, 2] - a[:, 1]*b[:, 3] + a[:, 2]*b[:, 0] + a[:, 3]*b[:, 1]
    # ab[:, 3] = a[:, 0]*b[:, 3] + a[:, 1]*b[:, 2] - a[:, 2]*b[:, 1] + a[:, 3]*b[:, 0]
    # return ab
    ab = np.zeros(4)
    ab[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    ab[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
    ab[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
    ab[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    return ab

def quaternConj(q):
    # qConj = np.zeros_like(q)
    # qConj[:, 0] = q[:, 0]
    # qConj[:, 1:] = -q[:, 1:]
    # return qConj
    qConj = np.zeros(4)
    qConj[0] = q[0]
    qConj[1] = -q[1]
    qConj[2] = -q[2]
    qConj[3] = -q[3]
    return qConj

class MadgwickAHRS:
    def __init__(self, SamplePeriod=1/256, Quaternion=[1, 0, 0, 0], Beta=1):
        self.SamplePeriod = SamplePeriod
        self.Quaternion = np.array(Quaternion)
        self.Beta = Beta

    def Update(self, Gyroscope, Accelerometer, Magnetometer):
        q = self.Quaternion.copy()

        if np.linalg.norm(Accelerometer) == 0:
            return

        Accelerometer = Accelerometer / np.linalg.norm(Accelerometer)

        if np.linalg.norm(Magnetometer) == 0:
            return

        Magnetometer = Magnetometer / np.linalg.norm(Magnetometer)

        h = quaternProd(q, quaternProd(np.array([0, Magnetometer[0], Magnetometer[1], Magnetometer[2]]), np.array(quaternConj(q))))
        b = [0, np.linalg.norm([h[1], h[2]]), 0, h[3]]

        F = np.array([
        2 * (q[1] * q[3] - q[0] * q[2]) - Accelerometer[0],
        2 * (q[0] * q[1] + q[2] * q[3]) - Accelerometer[1],
        2 * (0.5 - q[1] ** 2 - q[2] ** 2) - Accelerometer[2],
        2 * b[1] * (0.5 - q[2] ** 2 - q[3] ** 2) + 2 * b[3] * (q[1] * q[3] - q[0] * q[2]) - Magnetometer[0],
        2 * b[1] * (q[1] * q[2] - q[0] * q[3]) + 2 * b[3] * (q[0] * q[1] + q[2] * q[3]) - Magnetometer[1],
        2 * b[1] * (q[0] * q[2] + q[1] * q[3]) + 2 * b[3] * (0.5 - q[1] ** 2 - q[2] ** 2) - Magnetometer[2]
        ])

        J = np.array([
        [-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]],
        [2 * q[1], 2 * q[0], 2 * q[3], 2 * q[2]],
        [0, -4 * q[1], -4 * q[2], 0],
        [-2 * b[3] * q[2], 2 * b[3] * q[3], -4 * b[1] * q[2] - 2 * b[3] * q[0], -4 * b[1] * q[3] + 2 * b[3] * q[1]],
        [-2 * b[1] * q[3] + 2 * b[3] * q[1], 2 * b[1] * q[2] + 2 * b[3] * q[0], 2 * b[1] * q[1] + 2 * b[3] * q[3],
         -2 * b[1] * q[0] + 2 * b[3] * q[2]],
        [2 * b[1] * q[2], 2 * b[1] * q[3] - 4 * b[3] * q[1], 2 * b[1] * q[0] - 4 * b[3] * q[2], 2 * b[1] * q[1]]
        ])

        # step = np.dot(np.linalg.inv(J), F)
        step = np.linalg.pinv(J) @ F
        step /= np.linalg.norm(step)

        qDot = 0.5 * quaternProd(q, [0, Gyroscope[0], Gyroscope[1], Gyroscope[2]]) - self.Beta * step

        q = q + qDot * self.SamplePeriod
        self.Quaternion = q / np.linalg.norm(q)

    def UpdateIMU(self, Gyroscope, Accelerometer):
        q = self.Quaternion.copy()
        if np.linalg.norm(Accelerometer) == 0:
            return

        Accelerometer = Accelerometer / np.linalg.norm(Accelerometer)

        F = np.arrays([
        2*(q[1]*q[3] - q[0]*q[2]) - Accelerometer[0],
        2*(q[0]*q[1] + q[2]*q[3]) - Accelerometer[1],
        2*(0.5 - q[1]**2 - q[2]**2) - Accelerometer[2]])

        J = np.array([
        [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
        [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
        [0, -4*q[1], -4*q[2], 0]
        ])

        # step = np.linalg.inv(J.T @ J) @ J.T @ F
        step = np.linalg.pinv(J) @ F
        step = step / np.linalg.norm(step)

        qDot = 0.5 * quaternProd(q, [0, Gyroscope[0], Gyroscope[1], Gyroscope[2]]) - self.Beta * step

        q = q + qDot * self.SamplePeriod
        self.Quaternion = q / np.linalg.norm(q)
