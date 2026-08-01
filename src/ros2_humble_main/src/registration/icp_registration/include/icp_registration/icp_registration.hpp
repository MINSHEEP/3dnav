#ifndef ICP_REGISTRATION__ICP_REGISTRATION_HPP_
#define ICP_REGISTRATION__ICP_REGISTRATION_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace icp
{

/// Event-triggered, seed-based 3D relocalization.
///
/// This node intentionally publishes neither TF nor /initialpose.  AMCL remains
/// the sole owner of map->odom; a coordinator calls /relocalize and decides
/// whether to forward the returned map-frame pose to AMCL.
class IcpNode : public rclcpp::Node
{
public:
  explicit IcpNode(const rclcpp::NodeOptions & options);
  ~IcpNode() override;

  IcpNode(const IcpNode &) = delete;
  IcpNode & operator=(const IcpNode &) = delete;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace icp

#endif  // ICP_REGISTRATION__ICP_REGISTRATION_HPP_
