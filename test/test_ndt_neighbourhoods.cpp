// The voxel lookups NDT runs on every point, and the derivative sums it spreads over
// threads.
//
// VoxelGridCovariance finds a point's voxels through a hash table over its leaves. Its
// neighbourhoods must be the ones a lookup in getLeaves() gives, in the same order, and
// a copy must hand out its own leaves. NDT sums the derivatives in per-thread locals;
// the score, gradient and Hessian must not depend on the thread count.
#include <gtest/gtest.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/voxel_grid_covariance_omp.h>

#include <Eigen/Core>
#include <cmath>
#include <random>
#include <vector>

namespace
{

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Grid = pclomp::VoxelGridCovariance<Point>;

// A 20 x 20 m ground with noise, two walls, and a sparse cloud above them, so that the
// grid has full voxels, voxels with too few points, and empty ones.
Cloud::Ptr makeCloud(float shift, unsigned seed)
{
  std::mt19937 random(seed);
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  std::normal_distribution<float> noise(0.0f, 0.03f);
  Cloud::Ptr cloud(new Cloud);
  const auto add = [&cloud, shift](float x, float y, float z) {
      cloud->push_back(Point(x + shift, y, z));
    };
  for (int i = 0; i < 40000; ++i) {
    add(20.0f * unit(random) - 10.0f, 20.0f * unit(random) - 10.0f, noise(random));
  }
  for (int i = 0; i < 8000; ++i) {
    add(-6.0f + noise(random), 16.0f * unit(random) - 8.0f, 4.0f * unit(random));
    add(12.0f * unit(random) - 6.0f, 7.0f + noise(random), 4.0f * unit(random));
  }
  for (int i = 0; i < 300; ++i) {
    add(20.0f * unit(random) - 10.0f, 20.0f * unit(random) - 10.0f, 1.0f + 5.0f * unit(random));
  }
  return cloud;
}

Grid makeGrid(const Cloud::Ptr & cloud)
{
  Grid grid;
  grid.setLeafSize(1.0f, 1.0f, 1.0f);
  grid.setInputCloud(cloud);
  grid.filter(true);
  return grid;
}

Eigen::MatrixXi offsets7()
{
  Eigen::MatrixXi offsets = Eigen::MatrixXi::Zero(3, 7);
  offsets(0, 1) = 1;
  offsets(0, 2) = -1;
  offsets(1, 3) = 1;
  offsets(1, 4) = -1;
  offsets(2, 5) = 1;
  offsets(2, 6) = -1;
  return offsets;
}

// The neighbourhood found by looking each voxel up in getLeaves(), the std::map.
std::vector<Grid::LeafConstPtr> leafMapNeighbourhood(
  Grid & grid, const Eigen::MatrixXi & offsets, const Point & point)
{
  const Eigen::Vector3f leaf_size = grid.getLeafSize();
  const Eigen::Vector3i min_b = grid.getMinBoxCoordinates();
  const Eigen::Vector3i max_b = grid.getMaxBoxCoordinates();
  const Eigen::Vector3i multiplier = grid.getDivisionMultiplier();
  const Eigen::Vector3i ijk(
    static_cast<int>(std::floor(point.x / leaf_size[0])),
    static_cast<int>(std::floor(point.y / leaf_size[1])),
    static_cast<int>(std::floor(point.z / leaf_size[2])));
  std::vector<Grid::LeafConstPtr> neighbours;
  for (int i = 0; i < offsets.cols(); ++i) {
    const Eigen::Vector3i cell = ijk + offsets.col(i);
    if ((cell.array() < min_b.array()).any() || (cell.array() > max_b.array()).any()) {
      continue;
    }
    const auto leaf = grid.getLeaves().find((cell - min_b).dot(multiplier));
    if (leaf != grid.getLeaves().end() && leaf->second.nr_points >= grid.getMinPointPerVoxel()) {
      neighbours.push_back(&leaf->second);
    }
  }
  return neighbours;
}

// Query points over and around the grid, some outside its bounds.
std::vector<Point> queryPoints()
{
  std::mt19937 random(7);
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  std::vector<Point> points;
  for (int i = 0; i < 20000; ++i) {
    points.emplace_back(
      26.0f * unit(random) - 13.0f, 26.0f * unit(random) - 13.0f, 8.0f * unit(random) - 1.5f);
  }
  return points;
}

// Every query's DIRECT1, DIRECT7 and DIRECT26 neighbourhood matches the leaf map's.
// Returns the number of leaves found, so that a caller can check the queries hit some.
std::size_t expectNeighbourhoodsMatchLeafMap(Grid & grid)
{
  const Eigen::MatrixXi one = Eigen::MatrixXi::Zero(3, 1);
  const Eigen::MatrixXi seven = offsets7();
  const Eigen::MatrixXi twenty_six = pcl::getAllNeighborCellIndices();
  std::vector<Grid::LeafConstPtr> neighbours;
  std::size_t found = 0;
  for (const Point & point : queryPoints()) {
    grid.getNeighborhoodAtPoint1(point, neighbours);
    EXPECT_EQ(neighbours, leafMapNeighbourhood(grid, one, point));
    grid.getNeighborhoodAtPoint7(point, neighbours);
    EXPECT_EQ(neighbours, leafMapNeighbourhood(grid, seven, point));
    found += neighbours.size();
    grid.getNeighborhoodAtPoint(point, neighbours);
    EXPECT_EQ(neighbours, leafMapNeighbourhood(grid, twenty_six, point));
  }
  return found;
}

TEST(VoxelGridNeighbourhoods, MatchTheLeafMap)
{
  Grid grid = makeGrid(makeCloud(0.0f, 1));
  std::size_t valid = 0;
  for (const auto & leaf : grid.getLeaves()) {
    valid += leaf.second.nr_points >= grid.getMinPointPerVoxel();
  }
  // Some voxels hold too few points; the lookups must skip them.
  ASSERT_GT(valid, 100u);
  ASSERT_LT(valid, grid.getLeaves().size());
  EXPECT_GT(expectNeighbourhoodsMatchLeafMap(grid), 10000u);
}

TEST(VoxelGridNeighbourhoods, CopiesHandOutTheirOwnLeaves)
{
  Grid original = makeGrid(makeCloud(0.0f, 2));
  Grid copy(original);
  Grid assigned;
  assigned = original;
  // The original goes away: leaves pointing into it would dangle.
  original = makeGrid(makeCloud(3.0f, 3));
  EXPECT_GT(expectNeighbourhoodsMatchLeafMap(copy), 10000u);
  EXPECT_GT(expectNeighbourhoodsMatchLeafMap(assigned), 10000u);
  EXPECT_GT(expectNeighbourhoodsMatchLeafMap(original), 10000u);
}

TEST(VoxelGridNeighbourhoods, FilteringAgainReplacesTheLeaves)
{
  Grid grid = makeGrid(makeCloud(0.0f, 4));
  grid.setInputCloud(makeCloud(0.5f, 5));
  grid.filter(true);
  EXPECT_GT(expectNeighbourhoodsMatchLeafMap(grid), 10000u);
}

// Exposes one derivative pass of NDT at the identity.
class NdtProbe : public pclomp::NormalDistributionsTransform<Point, Point>
{
public:
  double derivatives(Eigen::Matrix<double, 6, 1> & gradient, Eigen::Matrix<double, 6, 6> & hessian)
  {
    Cloud source = *input_;
    Eigen::Matrix<double, 6, 1> p = Eigen::Matrix<double, 6, 1>::Zero();
    return computeDerivatives(gradient, hessian, source, p, true);
  }
};

TEST(NdtDerivatives, DoNotDependOnTheThreadCount)
{
  const Cloud::Ptr target = makeCloud(0.0f, 6);
  // A sparser scan of the same scene, moved by 0.3 m.
  pcl::VoxelGrid<Point> downsample;
  downsample.setLeafSize(0.7f, 0.7f, 0.7f);
  downsample.setInputCloud(makeCloud(0.3f, 7));
  Cloud::Ptr source(new Cloud);
  downsample.filter(*source);
  ASSERT_GT(source->size(), 500u);

  for (const auto method : {pclomp::KDTREE, pclomp::DIRECT26, pclomp::DIRECT7, pclomp::DIRECT1}) {
    double reference_score = 0.0;
    double reference_distance = 0.0;
    Eigen::Matrix<double, 6, 1> reference_gradient;
    Eigen::Matrix<double, 6, 6> reference_hessian;
    for (const int threads : {1, 2, 3, 4, 8}) {
      NdtProbe ndt;
      ndt.setResolution(1.0);
      ndt.setNumThreads(threads);
      ndt.setNeighborhoodSearchMethod(method);
      ndt.setInputTarget(target);
      ndt.setInputSource(source);
      // One alignment sets up the target grid and the Gaussian constants.
      ndt.setMaximumIterations(1);
      Cloud aligned;
      ndt.align(aligned);

      Eigen::Matrix<double, 6, 1> gradient;
      Eigen::Matrix<double, 6, 6> hessian;
      const double score = ndt.derivatives(gradient, hessian);
      const double distance = ndt.getLastMeanCorrespondenceDistance();
      if (threads == 1) {
        reference_score = score;
        reference_distance = distance;
        reference_gradient = gradient;
        reference_hessian = hessian;
        ASSERT_GT(std::abs(score), 1.0) << "method " << method;
        ASSERT_GT(gradient.norm(), 1.0) << "method " << method;
        continue;
      }
      const double tolerance = 1e-9;
      EXPECT_NEAR(score, reference_score, tolerance * std::abs(reference_score))
        << "method " << method << ", " << threads << " threads";
      EXPECT_NEAR(distance, reference_distance, tolerance * reference_distance)
        << "method " << method << ", " << threads << " threads";
      EXPECT_LE((gradient - reference_gradient).norm(), tolerance * reference_gradient.norm())
        << "method " << method << ", " << threads << " threads";
      EXPECT_LE((hessian - reference_hessian).norm(), tolerance * reference_hessian.norm())
        << "method " << method << ", " << threads << " threads";
    }
  }
}

}  // namespace
