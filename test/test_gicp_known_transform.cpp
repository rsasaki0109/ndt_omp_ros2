// Regression fixture for the PCL BFGS gradient callback contract.
#include <pclomp/gicp_omp.h>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <omp.h>
#include <cmath>
#include <random>
namespace {
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
// A floor and two walls of 1200 random points each.
Cloud::Ptr makeTarget() {
  Cloud::Ptr target(new Cloud);
  std::mt19937 random(20260924);
  std::uniform_real_distribution<float> uniform(-1.5f, 1.5f);
  for (int i=0;i<1200;++i) {
    const float a=uniform(random), b=uniform(random);
    Point p; p.intensity=0;
    p.x=a;p.y=b;p.z=0;target->push_back(p);
    p.x=-2;p.y=a;p.z=b+1.7f;target->push_back(p);
    p.x=a+.4f;p.y=2.2f;p.z=b+1.9f;target->push_back(p);
  }
  return target;
}
}  // namespace
TEST(GicpOmp, RecoversKnownTransforms) {
  const Cloud::Ptr target = makeTarget();
  for (int c=0;c<4;++c) {
    Eigen::Matrix4f truth=Eigen::Matrix4f::Identity();
    if(c==1)truth.block<3,1>(0,3)=Eigen::Vector3f(.08f,-.04f,.03f);
    if(c==2)truth.block<3,3>(0,0)=Eigen::AngleAxisf(.04f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    if(c==3) {
      truth.block<3,1>(0,3)=Eigen::Vector3f(-.06f,.08f,-.02f);
      truth.block<3,3>(0,0)=(Eigen::AngleAxisf(.03f,Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(-.04f,Eigen::Vector3f::UnitY())).toRotationMatrix();
    }
    Cloud::Ptr source(new Cloud);
    pcl::transformPointCloud(*target,*source,truth.inverse().eval());
    pclomp::GeneralizedIterativeClosestPoint<Point,Point> reg;
    reg.setInputSource(source);reg.setInputTarget(target);
    reg.setCorrespondenceRandomness(20);reg.setMaxCorrespondenceDistance(.5);
    reg.setTransformationEpsilon(.0001);reg.setMaximumIterations(40);
    Cloud aligned;reg.align(aligned,Eigen::Matrix4f::Identity());
    const auto result=reg.getFinalTransformation();
    const float position=(result.block<3,1>(0,3)-truth.block<3,1>(0,3)).norm();
    const float angle=std::abs(Eigen::AngleAxisf(truth.block<3,3>(0,0).transpose()*result.block<3,3>(0,0)).angle());
    const bool passed=reg.hasConverged()&&result.allFinite()&&position<.01f&&angle<.01f;
    EXPECT_TRUE(passed) << "case=" << c << " converged=" << reg.hasConverged()
                        << " translation_error=" << position << " rotation_error=" << angle;
  }
}
// The threads sum the cost and gradient in their own locals and collect their own
// correspondences; the result must not depend on how many there are.
TEST(GicpOmp, SameTransformOnEveryThreadCount) {
  const Cloud::Ptr target = makeTarget();
  Eigen::Matrix4f truth=Eigen::Matrix4f::Identity();
  truth.block<3,1>(0,3)=Eigen::Vector3f(-.06f,.08f,-.02f);
  truth.block<3,3>(0,0)=(Eigen::AngleAxisf(.03f,Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(-.04f,Eigen::Vector3f::UnitY())).toRotationMatrix();
  Cloud::Ptr source(new Cloud);
  pcl::transformPointCloud(*target,*source,truth.inverse().eval());
  const int default_threads=omp_get_max_threads();
  Eigen::Matrix4f reference;
  for (const int threads : {1,2,3,4,8}) {
    omp_set_num_threads(threads);
    pclomp::GeneralizedIterativeClosestPoint<Point,Point> reg;
    reg.setInputSource(source);reg.setInputTarget(target);
    reg.setCorrespondenceRandomness(20);reg.setMaxCorrespondenceDistance(.5);
    reg.setTransformationEpsilon(.0001);reg.setMaximumIterations(40);
    Cloud aligned;reg.align(aligned,Eigen::Matrix4f::Identity());
    ASSERT_TRUE(reg.hasConverged()) << threads << " threads";
    const Eigen::Matrix4f result=reg.getFinalTransformation();
    if (threads==1) {
      reference=result;
      EXPECT_LT((result.block<3,1>(0,3)-truth.block<3,1>(0,3)).norm(),.01f);
      continue;
    }
    EXPECT_LT((result-reference).cwiseAbs().maxCoeff(),1e-5f) << threads << " threads";
  }
  omp_set_num_threads(default_threads);
}
