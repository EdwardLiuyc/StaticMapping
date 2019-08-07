// MIT License

// Copyright (c) 2019 Edward Liu

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

// third party
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/expressions.h>
// stl
#include <string>

namespace static_map {
namespace back_end {

class OdomToPoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  // measurement information for rtk odom
  double z_;

 public:
  /**
   * Constructor
   * @param poseKey    associated pose varible key
   * @param model      noise model for GPS snesor, in X-Y
   * @param m          Point2 measurement
   */
  OdomToPoseFactor(gtsam::Key poseKey, const double& z,
                   const gtsam::noiseModel::Base::shared_ptr& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), z_(z) {}

  // error function
  // @param p    the pose in Pose3
  // @param H    the optional Jacobian matrix, which use boost optional and has
  // default null pointer
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> H = boost::none) const {
    if (H) (*H) = (gtsam::Matrix16() << 0., 0., 1., 0., 0., 0.).finished();
    return (gtsam::Vector1() << z_ - pose.translation().z()).finished();
  }
};

class OdomCalibrationFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3 /* lidar pose */,
                                      gtsam::Pose3 /* calibration tf */> {
 protected:
  gtsam::Pose3 measured_;

 public:
  typedef OdomCalibrationFactor This;
  typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
      Base;  ///< typedef for the base class
  // shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  OdomCalibrationFactor(const gtsam::Pose3& measure_odom,
                        const gtsam::SharedNoiseModel& model,
                        gtsam::Key poseKey, gtsam::Key calibKey)
      : Base(model, poseKey, calibKey), measured_(measure_odom) {}
  OdomCalibrationFactor() {}  ///< default constructor

  virtual ~OdomCalibrationFactor() {}  ///< destructor

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "OdomCalibrateFactor",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    Base::print(s, keyFormatter);
    gtsam::traits<gtsam::Pose3>::Print(measured_, s + ".z");
  }

  /**
   * equals
   */
  bool equals(const gtsam::NonlinearFactor& p, double tol = 1e-9) const {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol);
  }

  /** h(x)-z */
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::Pose3& calib,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {
    auto transformed =
        gtsam::traits<gtsam::Pose3>::Between(calib, pose, H2, H1) * calib;
    return gtsam::traits<gtsam::Pose3>::Local(measured_, transformed);
  }
};

}  // namespace back_end
}  // namespace static_map
