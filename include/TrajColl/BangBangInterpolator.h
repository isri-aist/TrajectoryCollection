#pragma once

#include <TrajColl/ElementInterpolation.h>
#include <TrajColl/Func.h>
#include <TrajColl/Interpolator.h>

namespace TrajColl
{
/** \brief Interpolator with bang-bang control.
    \tparam T value type
    \tparam U derivative type

    The velocity of each waypoint is assumed to be zero.
*/
template<class T, class U = T>
class BangBangInterpolator : Interpolator<T, U>
{
public:
  /** \brief Constructor.
      \param points times and values to be interpolated
      \param accelDurationList list of acceleration/deceleration duration
  */
  BangBangInterpolator(const std::map<double, T> & points = {}, const std::vector<double> & accelDurationList = {})
  : Interpolator<T, U>(points), accelDurationList_(accelDurationList)
  {
    func_ = std::make_shared<PiecewiseFunc<double>>();

    if(accelDurationList_.size() > 0 && (this->points_.size() != accelDurationList_.size() + 1))
    {
      throw std::invalid_argument(
          "[BangBangInterpolator] The size of the accelDurationList must be one less than the size of the points: "
          + std::to_string(accelDurationList_.size()) + " != " + std::to_string(this->points_.size()) + " + 1");
    }

    if(this->points_.size() >= 2)
    {
      calcCoeff();
    }
  }

  /** \brief Copy constructor. */
  BangBangInterpolator(const BangBangInterpolator & inst) : Interpolator<T, U>(inst)
  {
    accelDurationList_ = inst.accelDurationList_;
    func_ = std::make_shared<PiecewiseFunc<double>>(*inst.func_);
  }

  /** \brief Add point.
      \param point time and value

      \note BangBangInterpolator::calcCoeff should be called before calling BangBangInterpolator::operator().
  */
  virtual void appendPoint(const std::pair<double, T> & point) override
  {
    appendPoint(point, 0.0);
  }

  /** \brief Add point.
      \param point time and value
      \param accelDuration acceleration/deceleration duration

      \note BangBangInterpolator::calcCoeff should be called before calling BangBangInterpolator::operator().
  */
  void appendPoint(const std::pair<double, T> & point, double accelDuration)
  {
    if(!this->points_.empty() && point.first <= this->points_.rbegin()->first)
    {
      throw std::invalid_argument("[BangBangInterpolator] Only adding a trailing point is allowed: "
                                  + std::to_string(point.first)
                                  + " <= " + std::to_string(this->points_.rbegin()->first));
    }

    this->points_.insert(point);

    // Set accelDurationList_
    if(this->points_.size() >= 2)
    {
      double interpDuration = this->points_.rbegin()->first - std::next(this->points_.rbegin())->first;

      if(accelDuration <= 0.0)
      {
        constexpr double accelDurationRatio = 0.2;
        accelDuration = accelDurationRatio * interpDuration;
      }

      if(accelDuration >= 0.5 * interpDuration)
      {
        throw std::invalid_argument("[BangBangInterpolator] accelDuration must be less than half of interpDuration: "
                                    + std::to_string(accelDuration) + " >= " + std::to_string(0.5 * interpDuration));
      }

      accelDurationList_.push_back(accelDuration);
    }
  }

  /** \brief Calculate coefficients. */
  virtual void calcCoeff() override
  {
    if(this->points_.size() < 2)
    {
      throw std::out_of_range("[BangBangInterpolator] Number of points should be 2 or more: "
                              + std::to_string(this->points_.size()));
    }

    func_->clearFuncs();

    auto pointIt = this->points_.begin();
    for(int i = 0; i < static_cast<int>(this->points_.size()) - 1; i++, pointIt++)
    {
      double segStartTime = pointIt->first;
      double segEndTime = std::next(pointIt)->first;
      double segAccelDuration = accelDurationList_[i];
      double segVel = 1.0 / (segEndTime - segStartTime - segAccelDuration);
      double segAccel = segVel / segAccelDuration;

      // Set segFunc
      std::shared_ptr<PiecewiseFunc<double>> segFunc = std::make_shared<PiecewiseFunc<double>>();
      segFunc->setDomainLowerLimit(segStartTime);
      segFunc->appendFunc(segStartTime + segAccelDuration,
                          std::make_shared<QuadraticPolynomial<double>>(
                              std::array<double, 3>{static_cast<double>(i), 0.0, 0.5 * segAccel}, segStartTime));
      segFunc->appendFunc(
          segEndTime - segAccelDuration,
          std::make_shared<LinearPolynomial<double>>(std::array<double, 2>{static_cast<double>(i) + 0.5, segVel},
                                                     0.5 * (segStartTime + segEndTime)));
      segFunc->appendFunc(segEndTime,
                          std::make_shared<QuadraticPolynomial<double>>(
                              std::array<double, 3>{static_cast<double>(i + 1), 0.0, -0.5 * segAccel}, segEndTime));

      func_->appendFunc(segEndTime, segFunc);
    }

    func_->setDomainLowerLimit(this->points_.begin()->first);
  }

  /** \brief Calculate interpolated value.
      \param t time
  */
  virtual T operator()(double t) const override
  {
    size_t idx = func_->index(t);
    double ratio = (*func_)(t) - static_cast<double>(idx);
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
    double ratio = (*func_)(t) - static_cast<double>(idx);
    return func_->derivative(t, order)
           * interpolateDerivative<T, U>(std::next(this->points_.begin(), idx)->second,
                                         std::next(this->points_.begin(), idx + 1)->second, std::clamp(ratio, 0.0, 1.0),
                                         1);
  }

protected:
  //! List of acceleration/deceleration duration
  std::vector<double> accelDurationList_;

  //! Function to calculate the ratio of interpolation points
  std::shared_ptr<PiecewiseFunc<double>> func_;
};
} // namespace TrajColl
