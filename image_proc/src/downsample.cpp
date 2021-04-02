// Copyright 2021 AIT Institute of Technology GmbH
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_proc/downsample.hpp"
#include <sensor_msgs/image_encodings.hpp>

namespace enc = sensor_msgs::image_encodings;

namespace image_proc
{

DownsampleNode::DownsampleNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("DownsampleNode", options)
{
  // Create image pub
  pub_image_ = image_transport::create_publisher(this, "downsampled", rmw_qos_profile_sensor_data);
  // Create image sub
  sub_image_ = image_transport::create_subscription(
    this, "image",
    std::bind(
      &DownsampleNode::imageCb, this, std::placeholders::_1), "raw", rmw_qos_profile_sensor_data);
}

void DownsampleNode::imageCb(sensor_msgs::msg::Image::ConstSharedPtr image_msg)
{
  if (pub_image_.getNumSubscribers() < 1) {
    return;
  }

  auto bit_depth = enc::bitDepth(image_msg->encoding);
  if (16 != bit_depth) {
    RCLCPP_WARN(
      this->get_logger(),
      "16 bit image data requested, but image data from topic '%s' is %d bit",
      sub_image_.getTopic().c_str(), bit_depth);
    return;
  }
  if (!enc::isMono(image_msg->encoding)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Currently only grayscale image downsamping is supported, got encoding '%s' from topic '%s'",
      image_msg->encoding, sub_image_.getTopic().c_str());
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, "mono8");
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  pub_image_.publish(*cv_ptr->toImageMsg());
}

}  // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::DownsampleNode)
