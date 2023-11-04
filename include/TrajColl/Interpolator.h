#pragma once

#include <TrajColl/Func.h>

namespace TrajColl
{
/** \brief Interpolator of a sequence of waypoints.
    \tparam T value type
    \tparam U derivative type

    The velocity of each waypoint is assumed to be zero.
*/
template<class T, class U = T>
class Interpolator
{
public:
  /** \brief Constructor.
      \param points times and values to be interpolated
      \param accelDurationList list of acceleration/deceleration duration
  */
  Interpolator(const std::map<double, T> & points = {}) : points_(points) {}

  /** \brief Copy constructor. */
  Interpolator(const Interpolator & inst)
  {
    points_ = inst.points_;
  }

  /** \brief Clone this instance and get shared pointer. */
  virtual std::shared_ptr<Interpolator<T, U>> clone() const = 0;

  /** \brief Clear points. */
  virtual void clearPoints()
  {
    points_.clear();
  }

  /** \brief Add point.
      \param point time and value

      \note Interpolator::calcCoeff should be called before calling Interpolator::operator().
  */
  virtual void appendPoint(const std::pair<double, T> & point) = 0;

  /** \brief Calculate coefficients. */
  virtual void calcCoeff() = 0;

  /** \brief Calculate interpolated value.
      \param t time
  */
  virtual T operator()(double t) const = 0;

  /** \brief Calculate the derivative of interpolated value.
      \param t time
      \param order derivative order

      It is assumed that interpolateDerivative() returns zero if derivative order is greater than or equal to 2.
  */
  virtual U derivative(double t, int order = 1) const = 0;

  /** \brief Get start time. */
  double startTime() const
  {
    return points_.begin()->first;
  }

  /** \brief Get end time. */
  double endTime() const
  {
    return points_.rbegin()->first;
  }

  /** \brief Get points. */
  const std::map<double, T> & points() const
  {
    return points_;
  }

protected:
  //! Times and values to be interpolated
  std::map<double, T> points_;
};
} // namespace TrajColl
