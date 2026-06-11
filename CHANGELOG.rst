^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ndt_omp_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-06-11)
------------------
* First release prepared for the ROS 2 buildfarm (Humble / Jazzy).
* ROS 2 port of koide3/ndt_omp (OpenMP boosted NDT / GICP, BSD-2-Clause)
  with Humble support, an NDT Hessian fix, and a rotation/translation
  prior + mean-correspondence-distance API used by lidarslam_ros2.
* Runtime dependencies declared via <depend> so the binary package pulls
  rclcpp / PCL at install time; SPDX license tag; maintainer updated for
  the fork (original author credited).
* Contributors: Ryohei Sasaki, koide
