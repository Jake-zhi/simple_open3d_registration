# simple_open3d_registration

Point Cloud registration and outlier rejection using open3d.

* [Captured by Capture 3D Scan Anything (Iphone XR)](https://apps.apple.com/app/capture-3d-scan-anything/id1444183458?ref=producthunt)

* [Edit by open3D](http://www.open3d.org/)

* [Published on Sketchfab](https://skfb.ly/6QUqL)

# Install

```bash
pip3 install open3d
```

# Usage

```bash
python3 simple_registration.py ply/sample_gundum_01.ply ply/sample_gundum_02.ply
```

# Result

|01_before_registration|02_after_registration|
|:---|:---|
|![01_before_registration](01_before_registration.png)|![02_after_registration](02_after_registration.png)|

|03_vis_outlier|04_after_outlier|
|:---|:---|
|![03_vis_outlier](03_vis_outlier.png)|![04_after_outlier](04_after_outlier.png)|

|03_vis_outlier_Y|04_after_outlier_normal|
|:---|:---|
|![03_vis_outlier_Y](04_after_outlier_Y.png)|![04_after_outlier_normal](04_after_outlier_normal.png)|

|05_surface_reconstruction|05_visualize_densities|
|:---|:---|
|![05_surface_reconstruction](05_surface_reconstruction.png)|![05_visualize_densities](05_visualize_densities.png)|

