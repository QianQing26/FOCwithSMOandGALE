import numpy as np


class SignalGenerator:
    def __init__(self, duration=10.0, sample_rate=100.0):
        self.duration = duration
        self.sample_rate = sample_rate
        self.t = np.linspace(0, duration, int(duration * sample_rate))

    def generate(self, signal_type="sin", **kwargs):
        if signal_type == "constant":
            value = kwargs.get("value", 1.0)
            signal = np.full_like(self.t, value)

        elif signal_type == "sin":
            amp = kwargs.get("amplitude", 1.0)
            freq = kwargs.get("frequency", 1.0)
            offset = kwargs.get("offset", 0.0)
            signal = amp * np.sin(2 * np.pi * freq * self.t) + offset

        elif signal_type == "square":
            freq = kwargs.get("frequency", 1.0)
            amp = kwargs.get("amplitude", 1.0)
            offset = kwargs.get("offset", 0.0)
            signal = amp * np.sign(np.sin(2 * np.pi * freq * self.t)) + offset

        elif signal_type == "sawtooth":
            freq = kwargs.get("frequency", 1.0)
            amp = kwargs.get("amplitude", 1.0)
            offset = kwargs.get("offset", 0.0)
            signal = amp * 2 * (self.t * freq - np.floor(0.5 + self.t * freq)) + offset

        elif signal_type == "step":
            value_before = kwargs.get("value_before", 0.0)
            value_after = kwargs.get("value_after", 1.0)
            step_time = kwargs.get("step_time", self.duration / 2)
            signal = np.where(self.t < step_time, value_before, value_after)

        elif signal_type == "linear":
            slope = kwargs.get("slope", 1.0)
            intercept = kwargs.get("intercept", 0.0)
            signal = slope * self.t + intercept

        elif signal_type == "chirp":
            f0 = kwargs.get("f0", 1.0)
            f1 = kwargs.get("f1", 10.0)
            amp = kwargs.get("amplitude", 1.0)
            signal = amp * np.sin(
                2 * np.pi * self.t * (f0 + (f1 - f0) * self.t / self.duration)
            )

        elif signal_type == "triangle":
            freq = kwargs.get("frequency", 1.0)
            amp = kwargs.get("amplitude", 1.0)
            offset = kwargs.get("offset", 0.0)
            signal = (
                amp
                * (2 * np.abs(2 * (self.t * freq - np.floor(self.t * freq + 0.5))) - 1)
                + offset
            )

        elif signal_type == "exp_decay":
            tau = kwargs.get("tau", 1.0)
            amp = kwargs.get("amplitude", 1.0)
            signal = amp * np.exp(-self.t / tau)

        elif signal_type == "exp_growth":
            tau = kwargs.get("tau", 1.0)
            amp = kwargs.get("amplitude", 1.0)
            signal = amp * (1 - np.exp(-self.t / tau))

        elif signal_type == "impulse":
            amp = kwargs.get("amplitude", 1.0)
            impulse_time = kwargs.get("impulse_time", self.duration / 2)
            signal = np.zeros_like(self.t)
            idx = np.argmin(np.abs(self.t - impulse_time))
            signal[idx] = amp

        elif signal_type == "white_noise":
            amp = kwargs.get("amplitude", 1.0)
            signal = amp * np.random.normal(0, 1, size=self.t.shape)

        elif signal_type == "am":
            carrier_freq = kwargs.get("carrier_freq", 10.0)
            mod_freq = kwargs.get("mod_freq", 1.0)
            amp = kwargs.get("amplitude", 1.0)
            signal = (
                amp
                * (1 + np.sin(2 * np.pi * mod_freq * self.t))
                * np.sin(2 * np.pi * carrier_freq * self.t)
            )

        elif signal_type == "custom":
            func = kwargs.get("func", lambda t: np.zeros_like(t))
            signal = func(self.t)

        else:
            raise ValueError(f"Unsupported signal type: {signal_type}")

        # 添加高斯噪声
        noise_std = kwargs.get("noise_std", 0.0)
        if noise_std > 0:
            noise = np.random.normal(0, noise_std, size=signal.shape)
            signal += noise

        return self.t, signal
