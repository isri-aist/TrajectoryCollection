#include <TrajectoryCollection/CubicSpline.h>

namespace TrajectoryCollection
{
template class CubicSpline<Eigen::Vector3d>;
template class CubicSpline<Eigen::VectorXd>;
} // namespace TrajectoryCollection
