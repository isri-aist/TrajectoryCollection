/* Author: Masaki Murooka */

#include <fstream>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <TrajColl/BangBangInterpolator.h>

TEST(TestBangBangInterpolator, Test1)
{
  std::map<double, Eigen::Vector3d> points = {{10, Eigen::Vector3d(0, 10, 123)},
                                              {11, Eigen::Vector3d(1, 100, 123)},
                                              {12, Eigen::Vector3d(2, 0, 123)},
                                              {20, Eigen::Vector3d(3, -10, 123)},
                                              {100, Eigen::Vector3d(4, -100, 123)}};
  std::vector<double> accelDurationList = {0.0, 0.0, 0.1, 3.0, 40.0 - 1e-10};

  // setup interpolator
  TrajColl::BangBangInterpolator<Eigen::Vector3d> interp;
  auto it = points.begin();
  for(size_t i = 0; i < points.size(); i++, it++)
  {
    interp.appendPoint(*it, accelDurationList[i]);
  }
  interp.calcCoeff();

  // check for the sampled points of each section
  for(size_t i = 0; i < points.size() - 1; i++)
  {
    double t0 = std::next(points.begin(), i)->first;
    double t1 = std::next(points.begin(), i + 1)->first;
    const Eigen::Vector3d & x0 = std::next(points.begin(), i)->second;
    const Eigen::Vector3d & x1 = std::next(points.begin(), i + 1)->second;
    Eigen::Vector3d x_min = x0.array().min(x1.array());
    Eigen::Vector3d x_max = x0.array().max(x1.array());
    for(double t = t0; t <= t1; t += 0.01)
    {
      Eigen::Vector3d x = interp(t);
      EXPECT_TRUE(((x - x_min).array() > -1e-10).all() && ((x_max - x).array() > -1e-10).all());
    }
  }

  // check for the boundaries of each section
  for(const auto & point : points)
  {
    double t0 = point.first;
    const Eigen::Vector3d & x0 = point.second;
    Eigen::Vector3d x = interp(t0);
    Eigen::Vector3d v = interp.derivative(t0, 1);
    EXPECT_TRUE((x - x0).norm() < 1e-10);
    EXPECT_TRUE(v.norm() < 1e-10);
  }

  // output file
  {
    double t_start = points.begin()->first;
    double t_end = points.rbegin()->first;
    double t = t_start;
    bool break_flag = false;
    std::ofstream ofs("/tmp/TestBangBangInterpolator.txt");
    do
    {
      if(t >= t_end)
      {
        t = std::min(t, t_end);
        break_flag = true;
      }

      ofs << t << " " << interp(t).transpose() << std::endl;

      t += 0.01;
    } while(!break_flag);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
