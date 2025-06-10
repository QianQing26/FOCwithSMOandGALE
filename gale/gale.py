# 仿真用 Python 实现：GALE 融合滤波系统（Kalman, LPF, MAF, AdaLMS）
import numpy as np
import math


# ====================== Kalman Filter ======================
class KalmanFilter:
    def __init__(self, dt, init_speed):
        self.dt = dt
        self.initialized = True

        self.x = np.array([init_speed, 0.0])  # [speed, acceleration]
        self.x_pred = np.zeros(2)

        self.P = np.array([[1.0, 0.0], [0.0, 1.0]])
        self.P_pred = np.eye(2)

        self.F = np.array([[1, dt], [0, 1]])
        self.H = np.array([[1, 0]])
        self.Q = np.array([[0.001, 0], [0, 0.2]])
        self.R = np.array([[2.0]])

        self.K = np.zeros((2, 1))

    def update(self, z):
        # Predict
        self.x_pred = self.F @ self.x
        self.P_pred = self.F @ self.P @ self.F.T + self.Q

        # Update
        y = z - (self.H @ self.x_pred)[0]
        S = self.H @ self.P_pred @ self.H.T + self.R
        self.K = self.P_pred @ self.H.T @ np.linalg.inv(S)

        self.x = self.x_pred + (self.K * y).flatten()
        self.P = (np.eye(2) - self.K @ self.H) @ self.P_pred

        return self.x[0]

    def reset(self, speed):
        self.__init__(self.dt, speed)


# ====================== Low Pass Filter ======================
class LowPassFilter:
    def __init__(self, alpha, init_value=0.0):
        self.alpha = alpha
        self.value = init_value

    def update(self, input_value):
        self.value = self.alpha * input_value + (1 - self.alpha) * self.value
        return self.value

    def reset(self, value=0.0):
        self.value = value


# ====================== Moving Average Filter ======================
class MovingAverageFilter:
    def __init__(self, window_size, init_value=0.0):
        self.size = window_size
        self.buffer = [0.0] * window_size  # 初始化为空值
        self.index = 0
        self.sum = 0.0
        self.count = 0

        if init_value != 0.0:
            for i in range(window_size):
                self.buffer[i] = init_value
                self.sum += init_value
            self.count = window_size  # 如果初始值非零，直接填满窗口

    def update(self, input_value):
        if self.count < self.size:
            self.sum += input_value
            self.buffer[self.index] = input_value
            self.count += 1
        else:
            self.sum -= self.buffer[self.index]
            self.sum += input_value
            self.buffer[self.index] = input_value
        self.index = (self.index + 1) % self.size
        return self.sum / self.count if self.count > 0 else 0.0

    def reset(self, value=0.0):
        self.buffer = [0.0] * self.size
        self.index = 0
        self.sum = 0.0
        self.count = 0
        if value != 0.0:
            for i in range(self.size):
                self.buffer[i] = value
                self.sum += value
            self.count = self.size


# ====================== Adaptive LMS Filter ======================
class AdaLMSFilter:
    def __init__(self, mu=0.01, weight_range=(0.8, 1.2), bias_range=(-5, 5)):
        self.mu = mu
        self.weight = 1.0
        self.bias = 0.0
        self.prev_input = 0.0
        self.error = 0.0
        self.output = 0.0
        self.weight_min, self.weight_max = weight_range
        self.bias_min, self.bias_max = bias_range
        self.initialized = True

    def update(self, input_value, desired_output=0.0):
        y = self.weight * input_value + self.bias
        if desired_output != 0.0:
            e = desired_output - y
        else:
            e = -y + self.prev_input
        self.weight += self.mu * e * input_value
        self.bias += self.mu * e
        self.weight = np.clip(self.weight, self.weight_min, self.weight_max)
        self.bias = np.clip(self.bias, self.bias_min, self.bias_max)
        self.output = y
        self.error = e
        self.prev_input = 0.2 * input_value + 0.8 * self.prev_input
        return y

    def reset(self):
        self.__init__(
            self.mu, (self.weight_min, self.weight_max), (self.bias_min, self.bias_max)
        )


# ====================== Welford Statistics ======================
# class WelfordStats:
#     def __init__(self):
#         self.n = 0
#         self.mean = 0.0
#         self.M2 = 0.0

#     def update(self, value):
#         self.n += 1
#         delta = value - self.mean
#         self.mean += delta / self.n
#         delta2 = value - self.mean
#         self.M2 += delta * delta2

#     def get_mean(self):
#         return self.mean

#     def get_variance(self):
#         return self.M2 / (self.n - 1) if self.n > 1 else 1e-6

#     def get_stddev(self):
#         return math.sqrt(self.get_variance())
from gale.welford import WelfordStats


# ====================== GALE Fusion ======================
class GALE:
    def __init__(self, dt=0.01, init_speed=0.0, window_size=24):
        self.kalman = KalmanFilter(dt, init_speed)
        self.lpf = LowPassFilter(alpha=0.1, init_value=init_speed)
        self.maf = MovingAverageFilter(window_size=window_size, init_value=init_speed)
        self.lms = AdaLMSFilter()

        self.dt = dt

        self.filters = [self.kalman, self.lpf, self.maf, self.lms]
        self.stats = [WelfordStats() for _ in range(4)]

        self.u = np.ones(4) / 4  # Default uniform weight
        # self.u = np.array([1, 1, 0.5, 1])
        self.u = np.array([8, 2, 2, 1])
        self.z = np.zeros(4)
        self.initialized = True
        self.speed = init_speed
        self.fusion = 0.8

    def update(self, input_value):
        for i, f in enumerate(self.filters):
            if i == 3:
                desired = (
                    self.fusion * (self.kalman.x[0] + self.dt * (self.kalman.x[1]))
                    + (1 - self.fusion) * self.z[2]
                )
                self.z[i] = f.update(
                    self.speed, desired_output=self.kalman.x[0]
                ) + self.dt * (self.kalman.x[1])
            else:
                self.z[i] = f.update(input_value)
            self.stats[i].update(self.z[i])

        # 高斯似然融合
        weights = []
        for i in range(4):
            std = self.stats[i].get_std()
            mean = self.stats[i].get_mean()
            # likelihood = math.exp(-0.5 * ((input_value - self.z[i]) / std) ** 2) / (
            #     std * math.sqrt(2 * math.pi)
            # )
            likelihood = math.exp(-0.5 * ((input_value - self.z[i]) / std) ** 2)
            weights.append(self.u[i] * likelihood)

        weights = np.array(weights)
        if weights.sum() == 0:
            weights = np.ones(4) / 4
        else:
            weights /= weights.sum()

        fused_output = np.dot(weights, self.z)
        self.speed = fused_output
        return fused_output

    def reset(self):
        self.__init__()

    def get_speed(self):
        return self.speed
