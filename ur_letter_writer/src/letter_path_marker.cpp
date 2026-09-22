#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;

template<typename ValueT>
ValueT readParameter(rclcpp::Node& node, const std::string& name, const ValueT& default_value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<ValueT>(name, default_value);
  }
  return node.get_parameter(name).get_value<ValueT>();
}

class LetterPathMarker : public rclcpp::Node
{
public:
  LetterPathMarker()
  : Node(
      "letter_path_marker",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    base_frame_(readParameter<std::string>(*this, "base_frame", "base_link")),
    plane_x_(readParameter<double>(*this, "plane_x", 0.30)),
    center_y_(readParameter<double>(*this, "center_y", 0.0)),
    center_z_(readParameter<double>(*this, "center_z", 0.12)),
    letter_width_(readParameter<double>(*this, "letter_width", 0.10)),
    letter_height_(readParameter<double>(*this, "letter_height", 0.14)),
    curve_points_(readParameter<int>(*this, "curve_points", 24))
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "letter_path_markers", qos);

    publishPath();
    timer_ = create_wall_timer(1s, [this]() { publishPath(); });
    RCLCPP_INFO(
      get_logger(), "Publishing the intended D path on /letter_path_markers");
  }

private:
  visualization_msgs::msg::Marker makeMarker(int id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = now();
    marker.ns = "intended_letter_d";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.006;
    marker.color.r = 0.10F;
    marker.color.g = 0.90F;
    marker.color.b = 0.20F;
    marker.color.a = 1.0F;
    return marker;
  }

  geometry_msgs::msg::Point makePoint(double y, double z) const
  {
    geometry_msgs::msg::Point point;
    point.x = plane_x_;
    point.y = y;
    point.z = z;
    return point;
  }

  void publishPath()
  {
    const double left_y = center_y_ - letter_width_ / 2.0;
    const double top_z = center_z_ + letter_height_ / 2.0;
    const double bottom_z = center_z_ - letter_height_ / 2.0;

    auto stem = makeMarker(0);
    stem.points.push_back(makePoint(left_y, top_z));
    stem.points.push_back(makePoint(left_y, bottom_z));
    publisher_->publish(stem);

    auto curve = makeMarker(1);
    const int point_count = std::max(curve_points_, 4);
    for (int index = 0; index <= point_count; ++index) {
      const double ratio = static_cast<double>(index) / point_count;
      const double angle = kPi / 2.0 - kPi * ratio;
      const double y = left_y + letter_width_ * std::cos(angle);
      const double z = center_z_ + letter_height_ / 2.0 * std::sin(angle);
      curve.points.push_back(makePoint(y, z));
    }
    publisher_->publish(curve);
  }

  std::string base_frame_;
  double plane_x_;
  double center_y_;
  double center_z_;
  double letter_width_;
  double letter_height_;
  int curve_points_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LetterPathMarker>());
  rclcpp::shutdown();
  return 0;
}
