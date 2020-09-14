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

#include "common/bounding_box.h"

#include "gtest/gtest.h"

namespace static_map {
namespace common {

TEST(BoundingBox, Constructor) {
  EXPECT_NO_THROW(BoundingBox bbox(Eigen::Vector3d(0, 0, 0), 2.));
  EXPECT_NO_THROW(
      BoundingBox bbox(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2)));
  // EXPECT_ANY_THROW(
  //     BoundingBox bbox(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 1)));
}

TEST(BoundingBox, GenerateSubBoxes) {
  BoundingBox bbox(Eigen::Vector3d(0, 0, 0), 2.);
  const auto sub_boxes = bbox.GenerateSubBoxes();

  EXPECT_EQ(8, sub_boxes.size());
  EXPECT_EQ(Eigen::Vector3d(0.5, 0.5, 0.5), sub_boxes[0].GetCenter());
  EXPECT_EQ(Eigen::Vector3d(-0.5, 0.5, 0.5), sub_boxes[1].GetCenter());
  EXPECT_EQ(Eigen::Vector3d(-0.5, -0.5, 0.5), sub_boxes[2].GetCenter());
  EXPECT_EQ(Eigen::Vector3d(0.5, -0.5, 0.5), sub_boxes[3].GetCenter());
  EXPECT_EQ(Eigen::Vector3d(0.5, 0.5, -0.5), sub_boxes[4].GetCenter());
  EXPECT_EQ(Eigen::Vector3d(-0.5, 0.5, -0.5), sub_boxes[5].GetCenter());
  EXPECT_EQ(Eigen::Vector3d(-0.5, -0.5, -0.5), sub_boxes[6].GetCenter());
  EXPECT_EQ(Eigen::Vector3d(0.5, -0.5, -0.5), sub_boxes[7].GetCenter());
}

TEST(BoundingBox, ContainsPoint) {
  BoundingBox bbox(Eigen::Vector3d(0, 0, 0), 2.);
  EXPECT_TRUE(bbox.ContainsPoint(Eigen::Vector3d(1, 1, 1)));
  EXPECT_TRUE(bbox.ContainsPoint(Eigen::Vector3d(1, 1, 0)));
  EXPECT_TRUE(bbox.ContainsPoint(Eigen::Vector3d(1, 1, -1)));
  EXPECT_TRUE(bbox.ContainsPoint(Eigen::Vector3d(-1, 1, 1)));
  EXPECT_TRUE(bbox.ContainsPoint(Eigen::Vector3d(0, 0, 0)));

  EXPECT_FALSE(bbox.ContainsPoint(Eigen::Vector3d(1, 1, 1.001)));
  EXPECT_FALSE(bbox.ContainsPoint(Eigen::Vector3d(0, 0, -1.001)));
  EXPECT_FALSE(bbox.ContainsPoint(Eigen::Vector3d(0, 1.0003, 0)));
}

TEST(BoundingBox, GetSubBoxIndexForPoint_1) {
  BoundingBox bbox(Eigen::Vector3d(0, 0, 0), 2.);
  EXPECT_EQ(0, bbox.GetSubBoxIndexForPoint(Eigen::Vector3d(0.5, 0.5, 0.5)));
  EXPECT_EQ(6, bbox.GetSubBoxIndexForPoint(Eigen::Vector3d(-0.5, -0.5, -0.5)));
  EXPECT_EQ(-1, bbox.GetSubBoxIndexForPoint(Eigen::Vector3d(1.003, 0.5, 0.5)));
}

TEST(BoundingBox, GetSubBoxIndexForPoint_2) {
  BoundingBox bbox(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));
  EXPECT_EQ(0, bbox.GetSubBoxIndexForPoint(Eigen::Vector3d(0.5, 0.5, 0.5)));
  EXPECT_EQ(0, bbox.GetSubBoxIndexForPoint(Eigen::Vector3d(0., 0., 0.)));
  EXPECT_EQ(6, bbox.GetSubBoxIndexForPoint(Eigen::Vector3d(-0.5, -0.5, -0.5)));
  EXPECT_EQ(-1, bbox.GetSubBoxIndexForPoint(Eigen::Vector3d(1.003, 0.5, 0.5)));

  EXPECT_EQ(Eigen::Vector3d(0, 0, 0), bbox.GetCenter());
}

}  // namespace common
}  // namespace static_map
