# ndt_omp_ros2
This package provides an OpenMP-boosted Normal Distributions Transform (and GICP) algorithm derived from pcl. The NDT algorithm is modified to be SSE-friendly and multi-threaded. It can run up to 10 times faster than its original version in pcl.

### Benchmark
```
$ cd ~/ros2_ws/src/ndt_omp_ros2/data
$ ~/ros2_ws/install/ndt_omp_ros2/lib/ndt_omp/align 251370668.pcd 251371071.pcd
--- pcl::GICP ---
single : 253.065[msec]
10times: 1247.27[msec]
fitness: 0.220291

--- pcl::NDT ---
single : 111.041[msec]
10times: 1082.99[msec]
fitness: 0.246716
