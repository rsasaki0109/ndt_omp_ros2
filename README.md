# ndt_omp_ros2

[![CI](https://github.com/rsasaki0109/ndt_omp_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/ndt_omp_ros2/actions/workflows/ci.yml)

This package provides an OpenMP-boosted Normal Distributions Transform (and GICP) algorithm derived from pcl. The NDT algorithm is modified to be SSE-friendly and multi-threaded. It can run up to 10 times faster than its original version in pcl.

## Supported ROS distributions

- ROS 2 Humble
- ROS 2 Jazzy

## Build from source

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ndt_omp_ros2
source install/setup.bash
```

The package installs and exports the `ndt_omp_ros2::ndt_omp` CMake target.
A downstream package can use it with:

```cmake
find_package(ndt_omp_ros2 REQUIRED)
target_link_libraries(my_registration_node ndt_omp_ros2::ndt_omp)
```

## Benchmark

The repository includes two sample point clouds for the benchmark executable:

```bash
cd ~/ros2_ws/src/ndt_omp_ros2/data
ros2 run ndt_omp_ros2 align 251370668.pcd 251371071.pcd
```

```text
--- pcl::GICP ---
single : 267.385[msec]
10times: 1151.76[msec]
fitness: 0.220382

--- pclomp::GICP ---
single : 173.152[msec]
10times: 1299.14[msec]
fitness: 0.220388

--- pcl::NDT ---
single : 425.142[msec]
10times: 3638.77[msec]
fitness: 0.213937

--- pclomp::NDT (KDTREE, 1 threads) ---
single : 308.935[msec]
10times: 3095.53[msec]
fitness: 0.213937

--- pclomp::NDT (DIRECT7, 1 threads) ---
single : 188.942[msec]
10times: 1373.47[msec]
fitness: 0.214205

--- pclomp::NDT (DIRECT1, 1 threads) ---
single : 41.3584[msec]
10times: 347.261[msec]
fitness: 0.208511

--- pclomp::NDT (KDTREE, 8 threads) ---
single : 108.68[msec]
10times: 1046.16[msec]
fitness: 0.213937

--- pclomp::NDT (DIRECT7, 8 threads) ---
single : 56.9189[msec]
10times: 545.279[msec]
fitness: 0.214205

--- pclomp::NDT (DIRECT1, 8 threads) ---
single : 16.7266[msec]
10times: 169.097[msec]
fitness: 0.208511
```

Several methods for neighbor voxel search are implemented. If you select pclomp::KDTREE, results will be completely same as the original pcl::NDT. We recommend to use pclomp::DIRECT7 which is faster and stable. If you need extremely fast registration, choose pclomp::DIRECT1, but it might be a bit unstable.

<img src="data/screenshot.png" height="400pix" /><br>
Red: target, Green: source, Blue: aligned

## License

BSD-2-Clause. This project is a ROS 2 port of
[koide3/ndt_omp](https://github.com/koide3/ndt_omp).
