^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ndt_omp_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-11-03)
------------------
* ``VoxelGridCovariance`` has a new member (the leaf hash table below), so its
  layout changed: code built against the 0.1.0 headers must be rebuilt. The
  API is unchanged.
* GICP aligns faster, with the same result.

  * **False sharing:** the cost and gradient summed each thread's share into
    its own entry of shared vectors (``f_array``, ``g_array``, ``R_array``)
    for every correspondence. The correspondence search took slots from a
    shared atomic counter and wrote them into shared arrays. The threads now
    sum into locals and collect their correspondences locally; sorted by
    source index, as before, the correspondences are the same.
  * **One barrier per parallel loop:** BFGS evaluates the cost about 77 times
    per alignment, so the loops use ``nowait`` and wait only at the end of
    their parallel region.
  * **Measured:** on 154 Boreas scans (4 650 points against a 100 m crop of the
    map thinned to 0.5 m, 4 vCPUs), an alignment takes 10.9-11.6 ms instead of
    17.9-19.5 ms on 4 threads, and 23.9-24.6 ms instead of 25.7-26.0 ms on 8.
    The final transforms are bit-identical on all 154 scans.
* NDT aligns about twice as fast, with the same result.

  * **False sharing:** each thread added its score, gradient, Hessian and
    correspondence counts into its own entry of shared vectors, for every point
    and correspondence. The entries share cache lines, so the threads kept
    invalidating each other's caches. They now sum into locals and store them
    once.
  * **Voxel lookups:** the DIRECT1/7/26 neighbourhoods and the KDTREE results
    looked each voxel up in a ``std::map``, about 17 tree nodes for a
    100 000-voxel target. A hash table over the leaves, filled in the pass
    ``filter()`` already makes over them, replaces those lookups.
  * The transform's derivatives for a point are computed once, not once per
    voxel it falls near, and the neighbour offsets are built once.
  * **Measured:** on 154 Boreas scans (4 650 points, 100 m crop of the map,
    resolution 1.0, 4 vCPUs), DIRECT7 takes 14 ms instead of 27.5 ms on
    8 threads, and 12 ms instead of 33.6 ms on 4. From a seed 0.5 m off it
    takes 33 ms instead of 65 ms. KDTREE, DIRECT26 and DIRECT1 are 1.7-2.4
    times faster. The final transforms are bit-identical on all 154 scans for
    all four methods.
* Build as Release when no ``CMAKE_BUILD_TYPE`` is given. A plain
  ``colcon build`` compiled NDT and GICP without optimization: on a 1500-point
  Autoware Istanbul scan NDT aligned in 250 ms instead of 19 ms (4 threads).
  A build type set by the caller or the buildfarm is kept.
* Contributors: Ryohei Sasaki

0.1.0 (2026-06-11)
------------------
* First release prepared for the ROS 2 buildfarm (Humble / Jazzy).
* ROS 2 port of koide3/ndt_omp (OpenMP boosted NDT / GICP, BSD-2-Clause)
  with Humble support, an NDT Hessian fix, and a rotation/translation
  prior + mean-correspondence-distance API used by lidarslam_ros2.
* Runtime dependencies declared via <depend> so the binary package pulls
  rclcpp / PCL at install time; SPDX license tag; maintainer updated for
  the fork (original author credited).
* Install and export the ndt_omp library for downstream CMake consumers;
  keep the align executable available through ``ros2 run``.
* Respect buildfarm and caller-selected build types, and use the imported
  OpenMP target instead of global compiler flags.
* Add public API and installed-consumer smoke tests.
* Contributors: Ryohei Sasaki, koide
