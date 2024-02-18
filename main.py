import math

import gtsam
import numpy as np

import utils
from data import ImuData, ImuParams, UwbData
from smoothers import ImuUwbLoosely, ImuUwbTightly
from trilateration import estimate_positions


def main():
    anchor_positions = [
        gtsam.Point3(1.218987, 0.083615, -0.293541),
        gtsam.Point3(1.193123, 1.558471, -0.305832),
        gtsam.Point3(-0.694728, 0.095880, -1.049514),
        gtsam.Point3(-0.638702, 1.565646, -1.070000),
        gtsam.Point3(-0.605857, 0.107699, 0.858133),
        gtsam.Point3(-0.535126, 1.578035, 0.847774),
    ]

    uwb_data = UwbData(
        anchor_positions=anchor_positions,
        data_file="ultrax_mjerenja_2/mjerenje-hodanje-L/UWB.CSV",
    )
    gt_data = utils.read_gt_data(
        "ultrax_mjerenja_2/optitrack/mjerenje-hodanje-L-20102022.csv"
    )

    # Plots ranges before filtering
    utils.plot_distances(
        seperated_uwb_data=uwb_data.separate_uwb_measurements(), name="Noisy"
    )

    uwb_data.filter_outliers("Velocity_10_2", True, utils.velocity_filter, 1)
    uwb_data.filter_outliers(
        "Hampel_10_2.5", True, utils.hampel_filter_forloop, 10, 2.5
    )

    # Plots ranges after filtering
    utils.plot_distances(
        seperated_uwb_data=uwb_data.separate_uwb_measurements(), name="Filtered"
    )

    utils.plot_timediff(uwb_data=uwb_data.uwb_measurements)

    uwb_measurements_grouped = uwb_data.group_by_time(max_time_diff=0.05, min_length=6)

    utils.plot_timediff([x[0] for x in uwb_measurements_grouped])
    utils.plot_group_hist(uwb_data_grouped=uwb_measurements_grouped)

    initial_position = np.array([-0.08, 0.43, -2.50])
    timed_positions = estimate_positions(
        uwb_data_grouped=uwb_measurements_grouped,
        anchor_positions=anchor_positions,
        inlier_thresh=0.5,
        initial_position=initial_position
    )

    # Configure IMU params
    gyro_noise = 0.015 / 180 * math.pi
    accel_noise = 230e-6 * 9.81
    gyro_bias_noise = 4.33e-4
    accel_bias_noise = 2.66e-5
    integration_cov = 1e-3
    imu_data = ImuData(
        imu_params=ImuParams(
            accelerometer_sigma=accel_noise,
            gyroscope_sigma=gyro_noise,
            accelerometer_bias_sigma=accel_bias_noise,
            gyroscope_bias_sigma=gyro_bias_noise,
            integration_sigma=integration_cov,
            average_delta_t=0,
        ),
        data_file="ultrax_mjerenja_2/mjerenje-hodanje-L/IMU.CSV",
    )

    # Loosely coupled smoother
    smoother = ImuUwbLoosely()
    smoother.estimate(
        positions=timed_positions,
        imu_data=imu_data,
        initial_position=initial_position,
        variance_init_x=1,
        variance_init_b=1,
        variance_init_v=1,
        variance_positions=4,
    )

    translations = smoother.get_estimated_translations()
    rotations = smoother.get_estimated_rotations()
    velocities = smoother.get_estimated_velocities(n=len(timed_positions))

    gt_velocities = utils.calculate_velocities(gt_data)
    utils.plot3D_separate(data=velocities, name="Estimated_velocities")
    utils.plot3D_separate(data=gt_velocities, name="Gt_velocities")
    utils.plot3D_separate(data=rotations, name="Estimated_rot")

    utils.plot2D_from3D(data=translations, axis=["x", "z"], name="ISAM_loosely")
    utils.plot([np.linalg.norm(x) for x in gt_velocities], "Gt_velocity_magnitude")
    utils.plot([np.linalg.norm(x) for x in velocities], "Estimated_velocity_magnitude")

    # Tightly coupled smoother
    smoother = ImuUwbTightly()
    smoother.estimate(
        uwb_measurements_grouped=uwb_measurements_grouped,
        anchor_positions=anchor_positions,
        imu_data=imu_data,
        initial_position=initial_position,
        variance_init_x=1,
        variance_init_b=1,
        variance_init_v=1,
        variance_range=0.5,
    )

    translations = smoother.get_estimated_translations()
    rotations = smoother.get_estimated_rotations()

    utils.plot3D_separate(data=rotations, name="Estimated_rot")
    utils.plot2D_from3D(data=translations, axis=["x", "z"], name="ISAM_tightly")


if __name__ == "__main__":
    main()
