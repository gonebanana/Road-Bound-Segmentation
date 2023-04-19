"""
Module implements functions for detection road bounds
by input cloud point file .las.

"""
import os
import numpy as np
from laspy.file import File
from sklearn.linear_model import RANSACRegressor, LinearRegression
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import DBSCAN


def _extend_upper_right_bound(upper_right_bound, right_part):
    """
    Extends upper right bound of the crossroad that found by
    clusterization. It works with the projection of road on
    plane Oxy.
    At each step consider some train points from the right side.
    Looking for approximation line for train points by linear regression
    extend array of points by adding nearest neighbours of points
    predicted by fitted linear regressor. Stops by tracking neighbours
    number.

    Parameters
    ----------
    upper_right_bound : array
        Part of the upper right bound of the road that should
        be extended.
    right_part : array
        Part of the road that surrounds this bound.

    Returns
    -------
    upper_right_bound : array
        Extended upper right bound.

    """
    dx = 70
    radius = 10
    stop = 0
    neigh = NearestNeighbors(radius=radius, algorithm='kd_tree').fit(right_part[:, :2])
    linreg = LinearRegression()
    while stop != 1:
        # detection the most right point of upper_right_bound
        p0 = max(upper_right_bound, key=lambda el: el[0]).reshape(1, -1)
        train_points = right_part[:, :2][
            (abs(right_part[:, 0] - p0[0, 0]) < abs(dx)) &
            (abs(right_part[:, 1] - p0[0, 1]) < abs(dx))]
        # build regressor by points from around the most right point of
        # upper right bound in the square 2dx x 2dx by
        # sum(Ax_i + B - y_i)^2 -> min
        linreg.fit(train_points[:, 0].reshape(1, -1).T, train_points[:, 1])
        # points for prediction
        x_centers = np.arange(p0[0, 0], p0[0, 0] + dx/2, dx/10)
        neigh_ind = np.array([], dtype=int)
        centers = zip(x_centers,
                      linreg.predict(np.array(x_centers).reshape(1, -1).T))
        for center in centers:
            # add nearest neighbours of the center point
            neigh_ind = np.append(
                neigh_ind,
                neigh.radius_neighbors(np.array(center).reshape(1, -1),
                                       return_distance=False)[0])
        # Extended upper right bound with nearest neighbours
        upper_right_bound = np.unique(
            np.append(upper_right_bound, right_part[neigh_ind],
                      axis=0), axis=0)
        stop = 1 if len(neigh_ind) < 10 else 0
    return upper_right_bound


def find_road_bounds(file, save_path):
    """
    by given file generates file with road bounds points
    and returns a path to it.

    Parameters
    ----------
    file : str
        The name of cloud points .las file.
    save_path : str
        The name of `.las` file that contained detected road bound points.
    """

    in_file = File(file, mode="r")
    points = np.array(list(zip(in_file.X, in_file.Y, in_file.Z)))

    # select ground using RANSAC
    ransac = RANSACRegressor()
    ransac.fit(points[:, :2], points[:, 2])
    ground = points[ransac.inlier_mask_, :]

    linreg = LinearRegression()
    # select the part of RANSAC ground that probably suits road
    road = ground[ground[:, 2] < min(ground[:, 2]) + 35]
    for indent in range(25, 0, -5):
        # at each step improve detection of road using
        # linear regression of that part of the ground
        # that is most closed to the predicted plane
        linreg.fit(road[:, :2], road[:, 2])
        plane_z_linreg = linreg.predict(ground[:, :2])
        road = ground[(ground[:, 2] <= plane_z_linreg + indent)]

    # clustering road using DBSCAN
    clustering = DBSCAN(eps=50, min_samples=20).fit(road)
    left_bound = road[(clustering.labels_ == 1) | (clustering.labels_ == 7)]
    right_part = road[clustering.labels_ == 0]

    # clustering right part using DBSCAN
    clustering = DBSCAN(eps=5, min_samples=10).fit(right_part)
    low_right_bound = right_part[(clustering.labels_ == 27) |
                                 (clustering.labels_ == 1)]
    upper_right_bound = right_part[(clustering.labels_ == 16)]
    upper_right_bound = _extend_upper_right_bound(upper_right_bound,
                                                  right_part)
    bound = np.vstack((left_bound, low_right_bound, upper_right_bound))

    with File(save_path, mode='w', header=in_file.header) as out_file:
        (out_file.X, out_file.Y, out_file.Z) = (bound[:, 0], bound[:, 1], bound[:, 2])
