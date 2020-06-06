
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

#include "registrators/interface.h"

#include "registrators/icp_fast.h"
#include "registrators/icp_pointmatcher.h"
#include "registrators/ndt.h"
#include "registrators/ndt_gicp.h"

namespace static_map {
namespace registrator {

template <typename PointType>
std::shared_ptr<Interface<PointType>> CreateMatcher(
    const MatcherOptions& options, bool verbose) {
  std::shared_ptr<Interface<PointType>> matcher;
  switch (options.type) {
    case registrator::kIcpPM:
      matcher.reset(new IcpUsingPointMatcher<PointType>);
      break;
    case registrator::kNdtWithGicp:
      matcher.reset(new NdtWithGicp<PointType>);
      break;
    case registrator::kNdt:
      matcher.reset(new Ndt<PointType>);
      break;
    case registrator::kFastIcp:
      matcher.reset(new registrator::IcpFast<PointType>());
      break;
    case registrator::kLibicp:
    case registrator::kLegoLoam:
      LOG(FATAL) << "The registrator using libicp & lego-loam is deprecated. "
                    "please choose another type";
      return nullptr;
    default:
      PRINT_ERROR("Wrong type");
      return nullptr;
  }

  if (!options.registrator_options.empty()) {
    matcher->InitWithXml(options.registrator_options);
  }
  if (verbose) {
    matcher->PrintOptions();
  }
  matcher->InitWithOptions();
  return matcher;
}

template std::shared_ptr<Interface<pcl::PointXYZI>> CreateMatcher(
    const MatcherOptions& options, bool verbose);

}  // namespace registrator
}  // namespace static_map
