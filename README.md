# Road Bound Segmentation
<img src="example\road_bound.jpg" width="700" height="350"><em>Crossroad bounds (red color)</em>

## Description
Detection road bounds by the LiDar 3D Point Cloud `*.las` file. Clustering is performed by the combination of
the Linear Regression and DBSCAN (Density-Based Spatial Clustering) Algorithms. The implemented approach involves
a custom extension of the segmented points search.

## HowTo
Download the project from GitHub repository, then write in console: <br>
```bash
cd <path_to_download_folder>
```
Create and activate the environment:
 ```bash
 sudo pip install virtualenv
 virtualenv venv --python=<your_python_version>
 source venv/bin/activate
```
To test, run `example\example.py`. Road bound points are saved to `example\road_bound.las` file.
