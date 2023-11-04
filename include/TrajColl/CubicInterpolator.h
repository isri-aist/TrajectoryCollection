#pragma once

#include <TrajColl/CubicHermiteSpline.h>
#include <TrajColl/ElementInterpolation.h>
#include <TrajColl/Interpolator.h>

namespace TrajColl
{
/** \brief Cubic interpolator.
    \tparam T value type
    \tparam U derivative type

    The velocity of each waypoint is assumed to be zero.
*/
template<class T, class U = T>
class CubicInterpolator : public Interpolator<T, U>
{
protected:
  //! 1D vector
  using Vector1d = Eigen::Matrix<double, 1, 1>;

public:
  /** \brief Constructor.
      \param points times and values to be interpolated
  */
  CubicInterpolator(const std::map<double, T> & points = {}) : Interpolator<T, U>(points)
  {
    func_ = std::make_shared<CubicHermiteSpline<Vector1d>>(1, std::map<double, std::pair<Vector1d, Vector1d>>{});

    if(this->points_.size() >= 2)
    {
      calcCoeff();
    }
  }

  /** \brief Copy constructor. */
  CubicInterpolator(const CubicInterpolator & inst) : Interpolator<T, U>(inst)
  {
    func_ = std::make_shared<CubicHermiteSpline<Vector1d>>(*inst.func_);
  }

  /** \brief Clone this instance and get shared pointer. */
  virtual std::shared_ptr<Interpolator<T, U>> clone() const override
  {
    return std::shared_ptr<Interpolator<T, U>>(new CubicInterpolator(*this));
  }

  /** \brief Add point.
      \param point time and value

      \note CubicInterpolator::calcCoeff should be called before calling CubicInterpolator::operator().
  */
  virtual void appendPoint(const std::pair<double, T> & point) override
  {
    this->points_.insert(point);
  }

  /** \brief Calculate coefficients. */
  virtual void calcCoeff() override
  {
    if(this->points_.size() < 2)
    {
      throw std::out_of_range("[CubicInterpolator] Number of points should be 2 or more: "
                              + std::to_string(this->points_.size()));
    }

    func_->clearPoints();

    auto it = this->points_.begin();
    for(size_t i = 0; i < this->points_.size(); i++)
    {
      func_->appendPoint(std::make_pair(
          it->first, std::make_pair((Vector1d() << static_cast<double>(i)).finished(), Vector1d::Zero())));
      it++;
    }

    func_->setDomainLowerLimit(this->points_.begin()->first);
    func_->calcCoeff();
  }

  /** \brief Calculate interpolated value.
      \param t time
  */
  virtual T operator()(double t) const override
  {
    size_t idx = func_->index(t);
    double ratio = (*func_)(t)[0] - static_cast<double>(idx);
    return interpolate<T>(std::next(this->points_.begin(), idx)->second,
                          std::next(this->points_.begin(), idx + 1)->second, std::clamp(ratio, 0.0, 1.0));
  }

  /** \brief Calculate the derivative of interpolated value.
      \param t time
      \param order derivative order

      It is assumed that interpolateDerivative() returns zero if derivative order is greater than or equal to 2.
  */
  virtual U derivative(double t, int order = 1) const override
  {
    size_t idx = func_->index(t);
    double ratio = (*func_)(t)[0] - static_cast<double>(idx);
    return func_->derivative(t, order)[0]
           * interpolateDerivative<T, U>(std::next(this->points_.begin(), idx)->second,
                                         std::next(this->points_.begin(), idx + 1)->second, std::clamp(ratio, 0.0, 1.0),
                                         1);
  }

protected:
  //! Function to calculate the ratio of interpolation points
  std::shared_ptr<CubicHermiteSpline<Vector1d>> func_;
};
} // namespace TrajColl
