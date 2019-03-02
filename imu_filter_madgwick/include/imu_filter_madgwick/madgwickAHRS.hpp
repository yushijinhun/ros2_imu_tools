#ifndef MADGWICKAHRS__MADGWICKAHRS_HPP_
#define MADGWICKAHRS__MADGWICKAHRS_HPP_

#include <cmath>

namespace imu_filter_madgwick
{

class MadgwickAHRS
{
public:
  MadgwickAHRS();

  virtual ~MadgwickAHRS();

private:

  const float sampleFreq = 512.0f;
  const float betaDef = 0.1f;

  float invSqrt(float x);

public:

  float beta;              // algorithm gain
  float q0, q1, q2, q3;    // quaternion of sensor frame relative to auxiliary frame

  void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

};

}  // namespace imu_filter_madgwick

#endif  // MADGWICKAHRS__MADGWICKAHRS_HPP_
