import math

import numpy as np

import csv_loader
import utils

GRAVITY = 9.80665


class ImuParams:
    def __init__(
        self,
        accelerometer_sigma: float,
        gyroscope_sigma: float,
        integration_sigma: float,
        accelerometer_bias_sigma: float,
        gyroscope_bias_sigma: float,
        average_delta_t: float,
    ) -> None:
        self.accelerometer_sigma = accelerometer_sigma
        self.gyroscope_sigma = gyroscope_sigma
        self.integration_sigma = integration_sigma
        self.accelerometer_bias_sigma = accelerometer_bias_sigma
        self.gyroscope_bias_sigma = gyroscope_bias_sigma
        self.average_delta_t = average_delta_t


class ImuData:
    def __init__(self, imu_params: ImuParams, data_file: str) -> None:

        self.imu_params = imu_params
        self.load_data_from_file(data_file=data_file)

    def load_data_from_file(self, data_file: str):
        data_accel = csv_loader.loadIMUCSVReturnFormatedSensorDataFrame(
            "ACCELEROMETER", data_file
        )
        data_gyro = csv_loader.loadIMUCSVReturnFormatedSensorDataFrame(
            "GYROSCOPE", data_file
        )

        self.imu_measurements = []

        for i in range(2, data_gyro.shape[0]):
            time = data_gyro.iloc[i]["time (us)"] * 1e-6
            time_previous = data_gyro.iloc[i - 1]["time (us)"] * 1e-6
            dt = time - time_previous

            accelerometer = np.array(
                [
                    data_accel.iloc[i + 1]["x (g)"] * GRAVITY,
                    data_accel.iloc[i + 1]["y (g)"] * GRAVITY,
                    data_accel.iloc[i + 1]["z (g)"] * GRAVITY,
                ]
            )

            gyroscope = np.array(
                [
                    data_gyro.iloc[i]["x (dps)"] * math.pi / 180,
                    data_gyro.iloc[i]["y (dps)"] * math.pi / 180,
                    data_gyro.iloc[i]["z (dps)"] * math.pi / 180,
                ]
            )

            self.imu_measurements.append(
                ImuMeasurement(
                    time=time, dt=dt, accelerometer=accelerometer, gyroscope=gyroscope
                )
            )


class ImuMeasurement:
    def __init__(
        self, time: float, dt: float, accelerometer: np.array, gyroscope: np.array
    ) -> None:
        self.time = time
        self.dt = dt
        self.accelerometer = accelerometer
        self.gyroscope = gyroscope


class UwbData:
    def __init__(self, anchor_positions: list, data_file: str) -> None:
        self.anchor_positions = anchor_positions
        self.load_data_from_file(data_file=data_file)

    def load_data_from_file(self, data_file: str) -> None:
        data = csv_loader.loadUWBCSVReturnFormatedDataFrame(data_file)
        n_rows = data.shape[0]
        self.uwb_measurements = []

        for i in range(n_rows):
            distance = data["distance (m)"][i]
            time = data["time(us)"][i] * 1e-6
            index = int(data["a"][i])

            if not math.isnan(distance):
                self.uwb_measurements.append(
                    UwbMeasurement(time=time, distance=distance, index=index)
                )

    def separate_uwb_measurements(self):
        separate_uwb_measurements = []

        for i in range(1, len(self.anchor_positions) + 1):
            separate_uwb_measurements.append(
                [x for x in self.uwb_measurements if x.index == i]
            )

        return separate_uwb_measurements

    def connect_uwb_measurements(self, separated_uwb_measurements: list):
        connected_uwb_measurements = []
        for per_anchor in separated_uwb_measurements:
            for data in per_anchor:
                connected_uwb_measurements.append(data)

        connected_uwb_measurements.sort(key=lambda data: data.time)

        return connected_uwb_measurements

    def filter_outliers(self, name: str, plot: bool, outlierfun: callable, *args):
        separate_uwb_measurements = self.separate_uwb_measurements()
        filtered_separated_uwb_measurements = []
        for i in range(len(separate_uwb_measurements)):
            new_series, indices = outlierfun(separate_uwb_measurements[i], *args)

            if plot:
                utils.plot_outliers(
                    uwb_data=separate_uwb_measurements[i],
                    indices=indices,
                    name=name + "_Anchor_" + str(i + 1),
                )
            filtered_separated_uwb_measurements.append(new_series)

        self.uwb_measurements = self.connect_uwb_measurements(
            separated_uwb_measurements=filtered_separated_uwb_measurements
        )

    def group_by_time(self, max_time_diff: float, min_length: int):
        uwb_measurements_grouped = []

        i = 0
        while i < len(self.uwb_measurements) - 6:
            group = []
            j = 0
            while j < 6:
                measurement = self.uwb_measurements[i + j]
                if len(group) == 0:
                    group.append(measurement)
                else:
                    if (
                        measurement.index not in [x.index for x in group]
                        and (measurement.time - group[0].time) < max_time_diff
                    ):
                        group.append(measurement)
                    else:
                        break
                j = j + 1
            i = i + j
            uwb_measurements_grouped.append(group)
        return [x for x in uwb_measurements_grouped if len(x) >= min_length]


class UwbMeasurement:
    def __init__(self, time: float, distance: float, index: int) -> None:
        self.time = time
        self.distance = distance
        self.index = index

    def func(a, b, c):
        pass
