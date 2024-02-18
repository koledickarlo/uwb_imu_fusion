import math
from typing import List
import gtsam
import numpy as np

from gtsam.symbol_shorthand import B, L, V, X

from data import ImuData, ImuParams, UwbMeasurement
from utils import Position3DTimed

GRAVITY = 9.81


class Smoother:
    """Base class for smoothers"""

    def __init__(self) -> None:
        """Initialize"""
        self.results = []

    def get_estimated_translations(self, index=-1) -> list[np.ndarray]:
        """Returns a list of estimated translations

        Args:
            index (int, optional): Iteration index. Defaults to -1, i.e., last solution.

        Returns:
            list[np.ndarray]: List of estimated translations.
        """
        values = self.results[index]
        poses = gtsam.utilities.allPose3s(values)
        keys = gtsam.KeyVector(poses.keys())

        translations = []

        for key in keys:
            if self.results[index].exists(key):
                translations.append(values.atPose3(key).translation())

        return translations

    def get_estimated_rotations(self, index=-1) -> list[np.ndarray]:
        """Returns a list of estimated rotations.

        Args:
            index (int, optional): Iteration index. Defaults to -1, i.e., last solution.

        Returns:
            list[np.ndarray]: List of estimated rotations.
        """
        values = self.results[index]
        poses = gtsam.utilities.allPose3s(values)
        keys = gtsam.KeyVector(poses.keys())

        rotations = []

        for key in keys:
            if self.results[index].exists(key):
                rotations.append(values.atPose3(key).rotation().xyz())

        return rotations

    def get_estimated_velocities(self, n: int, index=-1) -> list[int]:
        """Returns a list of estimated velocities

        Args:
            n (int): Last timestamp
            index (int, optional): Iteration index. Defaults to -1, i.e., last solution.

        Returns:
            list[int]: List of estimated velocities
        """
        values = self.results[index]
        velocities = []

        for i in range(n):
            velocities.append(values.atVector(V(i)))

        return velocities

    def get_imu_noises(self, imu_params: ImuParams) -> gtsam.PreintegrationParams:
        """Create gtsam IMU noise object

        Args:
            imu_params (ImuParams): IMU noise parameters

        Returns:
            gtsam.PreintegrationParams: gtsam IMU noise object
        """
        w_coriolis = np.zeros(3)

        measured_acc_cov = np.eye(3) * np.power(imu_params.accelerometer_sigma, 2)
        measured_omega_cov = np.eye(3) * np.power(imu_params.gyroscope_sigma, 2)
        integration_error_cov = np.eye(3) * np.power(imu_params.integration_sigma, 2)

        imu_params = gtsam.PreintegrationParams(np.array([0, -GRAVITY, 0]))

        imu_params.setAccelerometerCovariance(measured_acc_cov)
        imu_params.setIntegrationCovariance(integration_error_cov)
        imu_params.setGyroscopeCovariance(measured_omega_cov)
        imu_params.setOmegaCoriolis(w_coriolis)

        return imu_params


class ImuUwbLoosely(Smoother):
    """Loosely coupled smoother via factor-graph formulation.
    Smoother iteratively calculates a joint MAP estimate for all poses in the trajectory.
    Expect preprocessed measurements, i.e., the positions are precalculated via robust multilateration
    of UWB measurements
    Uses on-manifold IMU preintegration as proposed in 10.1109/TRO.2016.2597321.
    """

    def __init__(self):
        super().__init__()

    def estimate(
        self,
        positions: List[Position3DTimed],
        imu_data: ImuData,
        initial_position: np.array,
        variance_init_x: float,
        variance_init_v: float,
        variance_init_b: float,
        variance_positions: float,
    ):
        """Estimates positions, velocities and IMU biases for whole trajectory

        Args:
            positions (List[Position3DTimed]): timestamped positions calculated by multilateration
            imu_data (ImuData): IMU measurements
            initial_position (np.array): Initial position at timestamp 0
            variance_init_x (float): Initial position variance
            variance_init_v (float): Initial velocity variance
            variance_init_b (float): Initial IMU bias variance
            variance_positions (float): Variance of positions obtained by multilateration
        """

        imu_params = imu_data.imu_params
        imu_measurements = imu_data.imu_measurements

        noise_model_init_x = gtsam.noiseModel.Diagonal.Precisions(np.asarray([0, 0, 0] + [1.0 / variance_init_x] * 3))
        noise_model_init_v = gtsam.noiseModel.Diagonal.Variances(np.ones(3) * variance_init_v)
        noise_model_init_b = gtsam.noiseModel.Diagonal.Variances(np.ones(6) * variance_init_b)
        noise_model_positions = gtsam.noiseModel.Diagonal.Precisions(
            np.asarray([0, 0, 0] + [1.0 / variance_positions] * 3)
        )

        current_pose_global = gtsam.Pose3(gtsam.Rot3.Rx(math.pi / 2), initial_position)
        current_velocity_global = np.zeros(3)
        current_bias = gtsam.imuBias.ConstantBias()  # init with zero bias

        isam = gtsam.ISAM2()
        new_factors = gtsam.NonlinearFactorGraph()
        new_values = gtsam.Values()

        j = 0
        included_imu_measurement_count = 0

        for i in range(len(positions)):
            current_pose_key = X(i)
            current_vel_key = V(i)
            current_bias_key = B(i)
            t = positions[i].time

            if i == 0:
                new_values.insert(current_pose_key, current_pose_global)
                new_values.insert(current_vel_key, current_velocity_global)
                new_values.insert(current_bias_key, current_bias)

                new_factors.addPriorPose3(current_pose_key, current_pose_global, noise_model_init_x)
                new_factors.addPriorVector(current_vel_key, current_velocity_global, noise_model_init_v)
                new_factors.addPriorConstantBias(current_bias_key, current_bias, noise_model_init_b)

            else:

                t_previous = positions[i - 1].time

                current_summarized_measurement = gtsam.PreintegratedImuMeasurements(
                    self.get_imu_noises(imu_params), current_bias
                )

                while j < len(imu_measurements) and imu_measurements[j].time <= t:

                    if imu_measurements[j].time >= t_previous:
                        current_summarized_measurement.integrateMeasurement(
                            imu_measurements[j].accelerometer,
                            imu_measurements[j].gyroscope,
                            imu_measurements[j].dt,
                        )
                        included_imu_measurement_count += 1
                    j += 1

                previous_pose_key = X(i - 1)
                previous_vel_key = V(i - 1)
                previous_bias_key = B(i - 1)

                new_factors.push_back(
                    gtsam.ImuFactor(
                        previous_pose_key,
                        previous_vel_key,
                        current_pose_key,
                        current_vel_key,
                        previous_bias_key,
                        current_summarized_measurement,
                    )
                )

                sigma_between_b = gtsam.noiseModel.Diagonal.Sigmas(
                    np.asarray(
                        [np.sqrt(included_imu_measurement_count) * imu_params.accelerometer_bias_sigma] * 3
                        + [np.sqrt(included_imu_measurement_count) * imu_params.gyroscope_bias_sigma] * 3
                    )
                )

                new_factors.push_back(
                    gtsam.BetweenFactorConstantBias(
                        previous_bias_key,
                        current_bias_key,
                        gtsam.imuBias.ConstantBias(),
                        sigma_between_b,
                    )
                )

                pose = gtsam.Pose3(current_pose_global.rotation(), positions[i].position)

                new_factors.addPriorPose3(current_pose_key, pose, noise_model_positions)
                new_values.insert(current_pose_key, pose)

                new_values.insert(current_vel_key, current_velocity_global)
                new_values.insert(current_bias_key, current_bias)

                if i > 2:
                    isam.update(new_factors, new_values)

                    new_factors.resize(0)
                    new_values.clear()

                    result = isam.calculateEstimate()
                    self.results.append(result)

                    current_pose_global = result.atPose3(current_pose_key)
                    current_velocity_global = result.atVector(current_vel_key)
                    current_bias = result.atConstantBias(current_bias_key)


class ImuUwbTightly(Smoother):
    """Tightly coupled smoother via factor-graph formulation.
    Smoother iteratively calculates a joint MAP estimate for all poses in the trajectory.
    Unlike ImuUwbLoosely, works directly on UWB range measurements
    Uses on-manifold IMU preintegration as proposed in 10.1109/TRO.2016.2597321.
    """

    def __init__(self):
        super().__init__()

    def estimate(
        self,
        uwb_measurements_grouped: List[List[UwbMeasurement]],
        anchor_positions: List[np.ndarray],
        imu_data: ImuData,
        initial_position: np.ndarray,
        variance_init_x: float,
        variance_init_v: float,
        variance_init_b: float,
        variance_range: float,
    ):
        """Estimates positions, velocities and IMU biases for whole trajectory

        Args:
            uwb_measurements_grouped (List[List[UwbMeasurement]]): Grouped UWB measurements.
                                                                   Each timestamp contains N range measurements
            imu_data (ImuData): IMU measurements
            initial_position (np.array): Initial position at timestamp 0
            variance_init_x (float): Initial position variance
            variance_init_v (float): Initial velocity variance
            variance_init_b (float): Initial IMU bias variance
            variance_range (float): Variance of UWB range measurements
        """
        imu_params = imu_data.imu_params
        imu_measurements = imu_data.imu_measurements

        noise_model_init_x = gtsam.noiseModel.Diagonal.Precisions(np.asarray([0, 0, 0] + [1.0 / variance_init_x] * 3))

        new_factors = gtsam.NonlinearFactorGraph()
        new_values = gtsam.Values()

        noise_model_init_v = gtsam.noiseModel.Diagonal.Variances(np.ones(3) * variance_init_v)
        noise_model_init_b = gtsam.noiseModel.Diagonal.Variances(np.ones(6) * variance_init_b)
        noise_model_ranges = gtsam.noiseModel.Diagonal.Variances(np.asarray([variance_range]))

        current_pose_global = gtsam.Pose3(gtsam.Rot3.Rx(math.pi / 2), initial_position)
        current_velocity_global = np.zeros(3)
        current_bias = gtsam.imuBias.ConstantBias()  # init with zero bias

        isam = gtsam.ISAM2()

        j = 0
        included_imu_measurement_count = 0

        for i in range(len(anchor_positions)):
            current_anchor_key = L(i)

            new_values.insert(current_anchor_key, anchor_positions[i])
            new_factors.addPriorPoint3(
                current_anchor_key,
                anchor_positions[i],
                gtsam.noiseModel.Diagonal.Variances(np.zeros(3)),
            )

        for i in range(len(uwb_measurements_grouped)):
            current_pose_key = X(i)
            current_vel_key = V(i)
            current_bias_key = B(i)
            t = uwb_measurements_grouped[i][0].time

            if i == 0:
                new_values.insert(current_pose_key, current_pose_global)
                new_values.insert(current_vel_key, current_velocity_global)
                new_values.insert(current_bias_key, current_bias)

                new_factors.addPriorPose3(current_pose_key, current_pose_global, noise_model_init_x)
                new_factors.addPriorVector(current_vel_key, current_velocity_global, noise_model_init_v)
                new_factors.addPriorConstantBias(current_bias_key, current_bias, noise_model_init_b)

            else:
                t_previous = uwb_measurements_grouped[i - 1][0].time

                current_summarized_measurement = gtsam.PreintegratedImuMeasurements(
                    self.get_imu_noises(imu_params), current_bias
                )

                while j < len(imu_measurements) and imu_measurements[j].time <= t:

                    if imu_measurements[j].time >= t_previous:
                        current_summarized_measurement.integrateMeasurement(
                            imu_measurements[j].accelerometer,
                            imu_measurements[j].gyroscope,
                            imu_measurements[j].dt,
                        )
                        included_imu_measurement_count += 1
                    j += 1

                previous_pose_key = X(i - 1)
                previous_vel_key = V(i - 1)
                previous_bias_key = B(i - 1)

                new_factors.push_back(
                    gtsam.ImuFactor(
                        previous_pose_key,
                        previous_vel_key,
                        current_pose_key,
                        current_vel_key,
                        previous_bias_key,
                        current_summarized_measurement,
                    )
                )

                sigma_between_b = gtsam.noiseModel.Diagonal.Sigmas(
                    np.asarray(
                        [np.sqrt(included_imu_measurement_count) * imu_params.accelerometer_bias_sigma] * 3
                        + [np.sqrt(included_imu_measurement_count) * imu_params.gyroscope_bias_sigma] * 3
                    )
                )

                new_factors.push_back(
                    gtsam.BetweenFactorConstantBias(
                        previous_bias_key,
                        current_bias_key,
                        gtsam.imuBias.ConstantBias(),
                        sigma_between_b,
                    )
                )

                for measurement in uwb_measurements_grouped[i]:
                    new_factors.push_back(
                        gtsam.RangeFactor3D(
                            current_pose_key,
                            L(measurement.index - 1),
                            measurement.distance,
                            noise_model_ranges,
                        )
                    )

                new_values.insert(current_pose_key, current_pose_global)
                new_values.insert(current_vel_key, current_velocity_global)
                new_values.insert(current_bias_key, current_bias)

                if i > 0:
                    isam.update(new_factors, new_values)

                    new_factors.resize(0)
                    new_values.clear()

                    result = isam.calculateEstimate()
                    self.results.append(result)

                    current_pose_global = result.atPose3(current_pose_key)
                    current_velocity_global = result.atVector(current_vel_key)
                    current_bias = result.atConstantBias(current_bias_key)

                new_factors.addPriorPose3(current_pose_key, current_pose_global, noise_model_init_x)
                new_factors.addPriorVector(current_vel_key, current_velocity_global, noise_model_init_v)
                new_factors.addPriorConstantBias(current_bias_key, current_bias, noise_model_init_b)
