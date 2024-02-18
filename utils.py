import csv
import math
from typing import List

import matplotlib.pyplot as plt
import numpy as np

from csv_loader import loadUWBCSVReturnFormatedDataFrame

SPEED_OF_LIGHT = 299702.547

#Lots of old utils used during development, needs refactoring

class UWB:
    def __init__(self, time: float, distance: float, index: int) -> None:
        self.time = time
        self.distance = distance
        self.index = index


class GroundTruth:
    def __init__(self, time: float, data: dict) -> None:
        self.time = time
        self.data = data


class Position3DTimed:
    def __init__(self, position: np.ndarray, time: float) -> None:
        self.position = position
        self.time = time


def calculate_overall_distance(translations):
    distance = 0
    for i in range(1, len(translations)):
        distance += np.linalg.norm(translations[i] - translations[i - 1])

    return distance


def plot(data, name):
    plt.figure("results/" + name)
    plt.plot(range(len(data)), data)
    plt.savefig("results/" + name + ".png")


def plot2D_from3D(data, axis, name):
    d = {"x": 0, "y": 1, "z": 2}

    axis_1 = d[axis[0]]
    axis_2 = d[axis[1]]

    plt.figure("results/" + name)
    plt.plot([x[axis_1] for x in data], [x[axis_2] for x in data])
    plt.xlabel(axis[0])
    plt.ylabel(axis[1])

    plt.savefig("results/" + name + ".png")


def plot3D_separate(data: list, name: str):
    plt.figure(name)
    d = ["x", "y", "z"]

    for i in range(3):
        plt.subplot(1, 3, i + 1)
        plt.title(d[i] + " axis")

        plt.plot(range(len(data)), [x[i] for x in data])

    plt.savefig("results/" + name + ".png")


def plot_distances(seperated_uwb_data: list, name: str) -> None:
    for i in range(len(seperated_uwb_data)):
        distances = [data.distance for data in seperated_uwb_data[i]]
        plt.figure("Figure_" + name + str(i + 1))
        plt.plot(range(len(distances)), distances)

        plt.xlabel("Measurement index")
        plt.ylabel("Distance (m)")
        plt.title("Distances from anchor " + str(i + 1))

        plt.savefig("results/" + name + "_anchor_" + str(i + 1) + ".png")
        plt.close()


def read_UWB_data(filename: str) -> UWB:
    data = loadUWBCSVReturnFormatedDataFrame(filename)
    n_rows = data.shape[0]
    uwb_data = []

    for i in range(n_rows):
        distance = data["distance (m)"][i]
        time = data["time(us)"][i] * 1e-6
        index = int(data["a"][i])

        if not math.isnan(distance):
            uwb_data.append(UWB(time=time, distance=distance, index=index))

    return uwb_data


def read_gt_data(filename: str) -> list:
    with open(filename, "r") as file:
        csvreader = list(csv.reader(file))

        gt_data = []
        for row in csvreader[7:]:
            try:
                gt_data.append(
                    Position3DTimed(
                        position=np.array(
                            [float(row[6]), float(row[7]), float(row[8])]
                        ),
                        time=float(row[1]),
                    )
                )
            except ValueError:
                continue

    return gt_data


def calculate_velocities(timed_positions: List[Position3DTimed]):
    velocities = []

    for i in range(1, len(timed_positions)):
        dt = timed_positions[i].time - timed_positions[i - 1].time
        velocities.append(
            (timed_positions[i].position - timed_positions[i - 1].position) / dt
        )

    return velocities


def plot_gt(gt_data: list) -> None:

    plt.figure("Ground truth")
    plt.plot(
        [m.data["m1"].x for m in gt_data if m.data["m1"] is not None],
        [m.data["m1"].z for m in gt_data if m.data["m1"] is not None],
    )

    plt.axis([-5, 5, -5, 5])
    plt.savefig("Gt.png")
    plt.close()


def plot_gt3D(gt_data: list) -> None:
    plt.figure("Ground truth")
    ax = plt.axes(projection="3d")
    ax.plot3D(
        [m.data["m1"].x for m in gt_data if m.data["m1"] is not None],
        [m.data["m1"].y for m in gt_data if m.data["m1"] is not None],
        [m.data["m1"].z for m in gt_data if m.data["m1"] is not None],
    )

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    plt.show()


def plot_3D(data: list, name: str) -> None:
    plt.figure("3D trajectory")
    ax = plt.axes(projection="3d")
    ax.plot3D([x[0] for x in data], [x[1] for x in data], [x[2] for x in data])

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    plt.savefig("results/Trajectory_" + name + ".png")
    plt.close()


def plot_timediff(uwb_data: UWBData):
    times = []
    for i in range(1, len(uwb_data)):
        times.append(uwb_data[i].time - uwb_data[i - 1].time)

    plt.figure("Times histogram")
    fig, axs = plt.subplots(1, 1, figsize=(10, 7), tight_layout=True)

    axs.hist(times, bins=20)
    plt.savefig("results/Times_histogram.png")
    plt.close()


def plot_velocities(gt_data: list) -> None:
    vx = []
    vy = []
    vz = []

    for i in range(1, len(gt_data)):
        try:
            vx.append(
                (gt_data[i].data["m1"].x - gt_data[i - 1].data["m1"].x)
                / (gt_data[i].time - gt_data[i - 1].time)
            )
            vy.append(
                (gt_data[i].data["m1"].y - gt_data[i - 1].data["m1"].y)
                / (gt_data[i].time - gt_data[i - 1].time)
            )
            vz.append(
                (gt_data[i].data["m1"].z - gt_data[i - 1].data["m1"].z)
                / (gt_data[i].time - gt_data[i - 1].time)
            )
        except:
            continue
    fig, axs = plt.subplots(1, 1, figsize=(10, 7), tight_layout=True)

    axs.hist(vx, bins=20)
    plt.show()
    plt.close()


def plot_dynamic_gt3D(gt_data: list) -> None:
    plt.figure("Ground truth")
    ax = plt.axes(projection="3d")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_zlim(-3, 3)
    x = [m.data["m1"].x for m in gt_data if m.data["m1"] is not None]
    y = [m.data["m1"].y for m in gt_data if m.data["m1"] is not None]
    z = [m.data["m1"].z for m in gt_data if m.data["m1"] is not None]

    for i in range(len(x)):
        ax.scatter3D(x[i], y[i], z[i])
        print(x[i])
        plt.pause(0.001)

    plt.show()
    plt.close()


def plot_outliers(uwb_data, indices, name):
    plt.figure("Plot with outliers_" + name)
    plt.plot(range(len(uwb_data)), [x.distance for x in uwb_data])
    for j in indices:
        plt.scatter(j, uwb_data[j].distance)

    plt.xlabel("Measurement index")
    plt.ylabel("Distance (m)")
    plt.title("Filtered distances from anchor")

    plt.savefig("results/" + name + ".png")
    plt.close()


def separate_uwb_data(uwb_data):
    separate_uwb = []

    for i in range(1, 7):
        separate_uwb.append([x for x in uwb_data if x.index == i])

    return separate_uwb


def connect_uwb_data(separated_uwb):
    connected_uwb = []
    for per_anchor in separated_uwb:
        for data in per_anchor:
            connected_uwb.append(data)

    connected_uwb.sort(key=lambda data: data.time)

    return connected_uwb


def find_plot_outliers(separate_uwb, name, outlierfun, *args):
    filtered_separated_uwb = []
    for i in range(len(separate_uwb)):
        new_series, indices = outlierfun(separate_uwb[i], *args)

        plt.figure("Plot with outliers_" + str(i + 1))
        plt.plot(range(len(separate_uwb[i])), [x.distance for x in separate_uwb[i]])
        for j in indices:
            plt.scatter(j, separate_uwb[i][j].distance)

        plt.xlabel("Measurement index")
        plt.ylabel("Distance (m)")
        plt.title("Filtered distances from anchor " + str(i + 1))

        plt.savefig("results/" + name + "_" + str(i + 1))
        plt.close()

        filtered_separated_uwb.append(new_series)
    return filtered_separated_uwb


def velocity_filter(input_series, max_velocity=0.5):

    indices = []
    new_series = input_series.copy()
    j = 0
    for i in range(1, len(input_series)):
        velocity = (input_series[i].distance - input_series[j].distance) / (
            input_series[i].time - input_series[j].time
        )
        if velocity > max_velocity:
            indices.append(i)
        else:
            j += 1

    for index in sorted(indices, reverse=True):
        del new_series[index]

    return new_series, indices


def hampel_filter_forloop(input_series, window_size=10, n_sigmas=3):

    n = len(input_series)
    new_series = input_series.copy()
    k = 1.4826

    indices = []

    for i in range((window_size), (n - window_size)):
        window = [
            x.distance for x in input_series[(i - window_size) : (i + window_size)]
        ]
        x0 = np.median(window)
        S0 = k * np.median(np.abs(window - x0))

        if np.abs(input_series[i].distance - x0) > n_sigmas * S0:
            indices.append(i)

    for index in sorted(indices, reverse=True):
        del new_series[index]
    return new_series, indices


def group_by_time(uwb_data, max_time_diff):
    uwb_data_grouped = []

    i = 0
    while i < len(uwb_data) - 6:
        group = []
        j = 0
        while j < 6:
            measurement = uwb_data[i + j]
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
        uwb_data_grouped.append(group)
    return uwb_data_grouped


def plot_group_hist(uwb_data_grouped):
    sizes = [len(x) for x in uwb_data_grouped]

    plt.figure("Group sizes histogram")
    fig, axs = plt.subplots(1, 1, figsize=(10, 7), tight_layout=True)

    axs.hist(sizes, bins=6)
    plt.savefig("results/Sizes_histogram.png")
    plt.close()


def plot_spheres(centers, radiuses):
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    for i in range(len(centers)):
        u, v = np.mgrid[0 : 2 * np.pi : 50j, 0 : np.pi : 50j]
        x = radiuses[i] * np.cos(u) * np.sin(v)
        y = radiuses[i] * np.sin(u) * np.sin(v)
        z = radiuses[i] * np.cos(v)

        ax.plot_surface(
            x - centers[i][0],
            y - centers[i][1],
            z - centers[i][2],
            color=np.random.choice(["g", "b", "r", "c", "m", "y"]),
            alpha=0.5 * np.random.random() + 0.5,
        )

    plt.show()
    plt.close()



