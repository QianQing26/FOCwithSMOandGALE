import math


class WelfordCore:
    def __init__(self):
        self.mean = 0.0
        self.M2 = 0.0
        self.n = 0

    def update(self, x: float):
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        self.M2 += delta * (x - self.mean)

    def reset(self):
        self.mean = 0.0
        self.M2 = 0.0
        self.n = 0

    def variance(self):
        return self.M2 / (self.n - 1) if self.n > 1 else 0.0

    def std(self):
        return math.sqrt(self.variance())


class WelfordStats:
    def __init__(self, max_count=100000):
        self.cores = [WelfordCore(), WelfordCore()]
        self.active = 0
        self.max_count = max_count

    def update(self, x: float):
        a = self.active
        b = 1 - a
        active_core = self.cores[a]
        standby_core = self.cores[b]

        # 更新 active core
        active_core.update(x)

        # 如果 active 的 n 超过 max_count/2，则备用 core 开始更新
        if active_core.n > self.max_count // 2:
            standby_core.update(x)

        # active 达到 max_count，进行切换
        if active_core.n >= self.max_count:
            self.active = b
            self.cores[a].reset()

    def get_mean(self):
        return self.cores[self.active].mean

    def get_std(self):
        return max(self.cores[self.active].std(), 1e-10)

    def get_n(self):
        return self.cores[self.active].n
