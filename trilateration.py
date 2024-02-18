import math
import gtsam
import numpy as np
import itertools
from utils import Position3DTimed


def multilateration(distances, anchor_positions):
    """Linear multilateration https://www.merl.com/publications/docs/TR2008-025.pdf"""
    N = len(distances)
    A = np.empty([N - 1, 3])
    b = np.empty([N - 1, 1])
    for i in range(N - 1):
        x_i = anchor_positions[i + 1][0]
        y_i = anchor_positions[i + 1][1]
        z_i = anchor_positions[i + 1][2]

        x_1 = anchor_positions[0][0]
        y_1 = anchor_positions[0][1]
        z_1 = anchor_positions[0][2]
        A[i, 0] = x_i - x_1
        A[i, 1] = y_i - y_1
        A[i, 2] = z_i - z_1

        b[i, 0] = (
            distances[0] * distances[0]
            - distances[i + 1] * distances[i + 1]
            - x_1 * x_1
            - y_1 * y_1
            - z_1 * z_1
            + x_i * x_i
            + y_i * y_i
            + z_i * z_i
        )
    A = A * 2

    return np.squeeze(np.linalg.lstsq(a=A, b=b)[0])


def multilateration_gtsam(distances, anchor_positions, initial):
    """Nonlinear multilateration"""
    graph = gtsam.NonlinearFactorGraph()

    LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.0, 0.0, 0.0]))
    MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1]))

    for i in range(len(distances)):
        graph.add(
            gtsam.PriorFactorPoint3(
                i + 1,
                gtsam.Point3(
                    anchor_positions[i][0],
                    anchor_positions[i][1],
                    anchor_positions[i][2],
                ),
                LANDMARK_NOISE,
            )
        )
        graph.add(
            gtsam.RangeFactor3(
                i + 1, len(distances) + 1, distances[i], MEASUREMENT_NOISE
            )
        )

    initial_estimate = gtsam.Values()

    for i in range(len(distances)):
        initial_estimate.insert(
            i + 1,
            gtsam.Point3(
                anchor_positions[i][0], anchor_positions[i][1], anchor_positions[i][2]
            ),
        )

    initial_estimate.insert(len(distances) + 1, initial)

    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
    result = optimizer.optimize()

    return result.atPoint3(len(distances) + 1)


def multilateration_robust(distances, anchor_positions, initial, inlier_thresh=0.5):
    "RANSAC multilateration"
    combinations = itertools.combinations(range(len(distances)), 4)

    best_inliers = 0
    best_error = 10e6
    best_position = None
    best_indices = None

    for comb in combinations:
        curr_distances = [distances[i] for i in comb]
        curr_anchor_positions = [anchor_positions[i] for i in comb]

        curr_position = multilateration_gtsam(
            distances=curr_distances,
            anchor_positions=curr_anchor_positions,
            initial=initial,
        )
        inliers = 0
        inlier_indices = []
        error = 0
        for i in range(len(distances)):
            diff = math.pow(
                distances[i] - distance3D(curr_position, anchor_positions[i]), 2
            )
            if diff < inlier_thresh:
                inlier_indices.append(i)
                inliers += 1
                error = error + diff

        if inliers > best_inliers:
            best_inliers = inliers
            best_error = error
            best_indices = inlier_indices
        if inliers > 0 and inliers == best_inliers:
            if error < best_error:
                best_error = error
                best_indices = inlier_indices

    if best_inliers < 4:
        return None, None

    best_distances = [distances[i] for i in best_indices]
    best_anchor_positions = [anchor_positions[i] for i in best_indices]

    best_position = multilateration_gtsam(
        distances=best_distances,
        anchor_positions=best_anchor_positions,
        initial=initial,
    )

    return best_position, best_indices


def estimate_positions(
    uwb_data_grouped, anchor_positions, inlier_thresh, initial_position
):
    timed_positions = []
    current_position = initial_position
    for group in uwb_data_grouped:
        curr_anchor_positions = [anchor_positions[x.index - 1] for x in group]
        curr_distances = [x.distance for x in group]

        position, _ = multilateration_robust(
            distances=curr_distances,
            anchor_positions=curr_anchor_positions,
            initial=current_position,
            inlier_thresh=inlier_thresh,
        )
        if position is not None:
            current_position = position

        timed_positions.append(Position3DTimed(position=position, time=group[0].time))

    return timed_positions


def distance3D(x, y):
    diff = x - y
    return np.linalg.norm(diff)
