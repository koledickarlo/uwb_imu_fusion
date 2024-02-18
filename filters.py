from abc import ABC, abstractmethod

import numpy as np


class Filter(ABC):
    def __init__(self):
        self.results = []

    @abstractmethod
    def estimate(self):
        pass


class RobustKF(Filter, ABC):
    def __init__(self, threshold, model_cov, measurement_cov):
        super().__init__()
        self.threshold = threshold
        self.model_cov = model_cov
        self.measurement_cov = measurement_cov

    def predict(self):
        F = self.model_jacobian()
        Q = self.model_noise()
        self.x = self.model_function()
        self.P = F @ self.P @ F.transpose() + Q

    def update(self):
        H = self.measurement_jacobian()
        R = self.measurement_noise()
        H_transpose = H.transpose()
        self.S = H @ self.P @ H_transpose + R

        inverse_S = np.linalg.inv(self.S)

        residual = self.measurement - self.measurement_function()

        mah_distance = np.sqrt(residual.transpose() @ inverse_S @ residual)

        if mah_distance < self.threshold:
            K = self.P @ H_transpose @ inverse_S
            self.x = self.x + K @ residual
            self.P = (np.identity(len(self.x)) - K @ H) @ self.P
        else:
            print("WARNING")

    def estimate(self, x_initial, P_initial, measurements, measurement_times):
        previous_time = measurement_times[0] - 1e-6

        self.x = x_initial
        self.P = P_initial

        for measurement, time in zip(measurements, measurement_times):
            self.delta_t = time - previous_time
            self.measurement = measurement.reshape(-1, 1)

            self.predict()
            self.update()

            self.results.append(self.x)
            previous_time = time

    @abstractmethod
    def model_jacobian(self):
        pass

    @abstractmethod
    def model_function(self):
        pass

    @abstractmethod
    def model_noise(self):
        pass

    @abstractmethod
    def measurement_jacobian(self):
        pass

    @abstractmethod
    def measurement_function(self):
        pass

    @abstractmethod
    def measurement_noise(self):
        pass


class ConstantVelocityPositionKF(RobustKF):
    def __init__(self, threshold, model_cov, measurement_cov):
        super().__init__(threshold, model_cov, measurement_cov)

    def model_jacobian(self):
        return np.array(
            [
                [1, 0, 0, self.delta_t, 0, 0],
                [0, 1, 0, 0, self.delta_t, 0],
                [0, 0, 1, 0, 0, self.delta_t],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )

    def model_function(self):
        return self.model_jacobian() @ self.x

    def model_noise(self):
        cov_p = self.delta_t * self.delta_t / 2 * self.model_cov
        cov_v = self.delta_t * self.model_cov
        return np.diag([cov_p, cov_p, cov_p, cov_v, cov_v, cov_v])

    def measurement_jacobian(self):
        return np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]])

    def measurement_function(self):
        return self.measurement_jacobian() @ self.x

    def measurement_noise(self):
        return np.diag(
            [self.measurement_cov, self.measurement_cov, self.measurement_cov]
        )


class ConstantVelocityRangeKF(RobustKF):
    def __init__(self, threshold, model_cov, measurement_cov, anchor_positions):
        super().__init__(threshold, model_cov, measurement_cov)
        self.anchor_positions = anchor_positions

    def estimate(self, x_initial, P_initial, measurements, measurement_times):
        previous_time = measurement_times[0] - 1e-6

        self.x = x_initial
        self.P = P_initial

        for measurement, time in zip(measurements, measurement_times):
            self.delta_t = time - previous_time
            self.measurement = np.array([x.distance for x in measurement]).reshape(
                -1, 1
            )
            self.anchor_indices = [x.index for x in measurement]

            self.predict()
            self.update()

            self.results.append(self.x)
            previous_time = time

    def model_jacobian(self):
        return np.array(
            [
                [1, 0, 0, self.delta_t, 0, 0],
                [0, 1, 0, 0, self.delta_t, 0],
                [0, 0, 1, 0, 0, self.delta_t],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )

    def model_function(self):
        return self.model_jacobian() @ self.x

    def model_noise(self):
        cov_p = self.delta_t * self.delta_t / 2 * self.model_cov
        cov_v = self.delta_t * self.model_cov
        return np.diag([cov_p, cov_p, cov_p, cov_v, cov_v, cov_v])

    def measurement_jacobian(self):
        H = np.zeros((len(self.anchor_indices), 6))
        for i in range(len(self.anchor_indices)):
            diff = self.x[:3] - self.anchor_positions[
                self.anchor_indices[i] - 1
            ].reshape(3, 1)

            H[i, :3] = diff.transpose() / np.linalg.norm(diff)

        return H

    def measurement_function(self):
        y = np.zeros((len(self.anchor_indices), 1))
        for i in range(len(self.anchor_indices)):
            diff = (
                self.x[:3]
                - self.anchor_positions[self.anchor_indices[i] - 1].transpose()
            )
            y[i, 0] = np.linalg.norm(diff)

        return y

    def measurement_noise(self):
        return np.diag([self.measurement_cov] * len(self.anchor_indices))
