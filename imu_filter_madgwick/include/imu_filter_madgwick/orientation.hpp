#ifndef ORIENTATION__ORIENTATION_HPP_
#define ORIENTATION__ORIENTATION_HPP_

#include <Eigen/Geometry>
#include <iostream>

template<typename T, int DIM>
class Orientation;

template<typename T>
class Orientation<T, 3>
{
public:
  Orientation()
    : d_quaternion{Eigen::Quaternion<T>::Identity()}
  {
  }

  Eigen::Quaternion<T> const& getQuaternion() const
  {
    return d_quaternion;
  }

  void reset();
  void reset(Eigen::Quaternion<T> quat);

  /** Integrate measurement of angular rate
   *
   * @param angularRate Measured angular rate around axes, in rad/sec, e.g. from gyroscope
   * @param interval Time interval to integrate over, in secons
   */
  void integrate(Eigen::Matrix<T, 3, 1> const& angularRate, T interval);

  /** Integrate measurement of angular rate, corrected given a reference measurement
   *
   * @param angularRate Measured angular rate around axes, in rad/sec, e.g. from gyroscope
   * @param interval Time interval to integrate over, in seconds
   * @param referenceDirMeas Measurement of the reference directory in local reference frame
   * @param maxGyroError Maximum gyroscope measurement error, in rad/sec. Used to determine correction weight
   * @param referenceDir Reference direction that is measured in global reference frame. By default direction of gravity
   */
  void integrate(Eigen::Matrix<T, 3, 1> const& angularRate, T interval,
                 Eigen::Matrix<T, 3, 1> const& referenceDirMeas,
                 T maxGyroError,
                 Eigen::Matrix<T, 3, 1> const& referenceDir = Eigen::Matrix<T, 3, 1>{0, 0, 1});
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Eigen::Quaternion<T> d_quaternion;
};

using Orientation3d = Orientation<double, 3>;



template<typename T>
void Orientation<T, 3>::reset()
{
  d_quaternion = Eigen::Quaternion<T>::Identity();
}

template<typename T>
void Orientation<T, 3>::reset(Eigen::Quaternion<T> quat)
{
  d_quaternion = quat;
}

template<typename T>
void Orientation<T, 3>::integrate(const Eigen::Matrix<T, 3, 1> &angularRate, T interval)
{
  auto theta = angularRate * interval / 2;
  auto thetaMag = theta.norm();
  if (thetaMag == 0)
    return;

  auto deltaQuat = Eigen::Quaternion<T>{};
  deltaQuat.w() = cos(thetaMag);
  deltaQuat.vec() = sin(thetaMag) / thetaMag * theta;

  // T_A2^W = T_A1^W * T_A2^A1
  d_quaternion = d_quaternion * deltaQuat;
}

template<typename T>
void Orientation<T, 3>::integrate(Eigen::Matrix<T, 3, 1> const& angularRate, T interval,
                                  Eigen::Matrix<T, 3, 1> const& referenceDirLocalMeas,
                                  T maxGyroError,
                                  Eigen::Matrix<T, 3, 1> const& referenceDirGlobal)
{
  integrate(angularRate, interval);

  T constexpr sqrt34 = std::sqrt(0.75);
  auto beta = sqrt34 * maxGyroError;

  // The objective function is the difference between expected and measured reference dir in local frame
  // The quaternion describes the orientation of the local frame in the global frame,
  // so a vector multiplied with it is transformed from local to global

  Eigen::Matrix<T, 3, 1> objectiveFunction =
    d_quaternion.conjugate() * referenceDirGlobal - referenceDirLocalMeas;

  auto q1 = d_quaternion.w();
  auto q2 = d_quaternion.x();
  auto q3 = d_quaternion.y();
  auto q4 = d_quaternion.z();

  auto jacobian =
    (Eigen::Matrix<T, 3, 4>() <<
     2 * q4  , -2 * q1 , 2 * q2 , -2 * q3,
     2 * q1  , 2 * q4  , 2 * q3 , 2 * q2,
     -4 * q2 , -4 * q3 , 0.0    , 0.0   ).
    finished();

  Eigen::Vector4d normalizedObjectiveFunctionGradient =
    jacobian.transpose() * objectiveFunction;

  if (normalizedObjectiveFunctionGradient.squaredNorm() > 1e-16)
    normalizedObjectiveFunctionGradient.normalize();

  d_quaternion.coeffs() -= beta * normalizedObjectiveFunctionGradient * interval;
  d_quaternion.normalize();
}

#endif // ORIENTATION__ORIENTATION_HPP_
