#include "point_type_converter/point_type_converter.hpp"

#include <cmath>
#include <chrono>

namespace autoware_tartan_bridge
{

PointTypeConverter::PointTypeConverter(const rclcpp::NodeOptions & options)
: Node("point_type_converter", options)
{
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&PointTypeConverter::pointCloudCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_pointcloud", rclcpp::SensorDataQoS());
  computation_time_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("processing_time_ms", 10);

  RCLCPP_INFO(this->get_logger(), "Point type converter initialised!");
}

void PointTypeConverter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // quick check is data is empty just dont run
  if(msg->data.empty() || msg->width * msg->height == 0)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Input message empty");
    return;
  }

  auto start = std::chrono::high_resolution_clock::now();

  sensor_msgs::msg::PointCloud2 output_msg;
  output_msg.header = msg->header;
  output_msg.height = msg->height;
  output_msg.width = msg->width;
  output_msg.is_dense = msg->is_dense;
  output_msg.is_bigendian = msg->is_bigendian;
  output_msg.point_step = sizeof(PointXYZIRCAEDT);
  output_msg.row_step = output_msg.point_step * output_msg.width;

  // Define the new point fields
  output_msg.fields.resize(10);
  output_msg.fields[0].name = "x";
  output_msg.fields[0].offset = offsetof(PointXYZIRCAEDT, x);
  output_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields[0].count = 1;

  output_msg.fields[1].name = "y";
  output_msg.fields[1].offset = offsetof(PointXYZIRCAEDT, y);
  output_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields[1].count = 1;

  output_msg.fields[2].name = "z";
  output_msg.fields[2].offset = offsetof(PointXYZIRCAEDT, z);
  output_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields[2].count = 1;

  output_msg.fields[3].name = "intensity";
  output_msg.fields[3].offset = offsetof(PointXYZIRCAEDT, intensity);
  output_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
  output_msg.fields[3].count = 1;

  output_msg.fields[4].name = "return_type";
  output_msg.fields[4].offset = offsetof(PointXYZIRCAEDT, return_type);
  output_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  output_msg.fields[4].count = 1;

  output_msg.fields[5].name = "channel";
  output_msg.fields[5].offset = offsetof(PointXYZIRCAEDT, channel);
  output_msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
  output_msg.fields[5].count = 1;

  output_msg.fields[6].name = "azimuth";
  output_msg.fields[6].offset = offsetof(PointXYZIRCAEDT, azimuth);
  output_msg.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields[6].count = 1;

  output_msg.fields[7].name = "elevation";
  output_msg.fields[7].offset = offsetof(PointXYZIRCAEDT, elevation);
  output_msg.fields[7].datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields[7].count = 1;

  output_msg.fields[8].name = "distance";
  output_msg.fields[8].offset = offsetof(PointXYZIRCAEDT, distance);
  output_msg.fields[8].datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields[8].count = 1;

  output_msg.fields[9].name = "time_stamp";
  output_msg.fields[9].offset = offsetof(PointXYZIRCAEDT, time_stamp);
  output_msg.fields[9].datatype = sensor_msgs::msg::PointField::UINT32;
  output_msg.fields[9].count = 1;

  output_msg.data.resize(output_msg.row_step * output_msg.height);

  // Precompute offsets
  const size_t x_offset = 0;
  const size_t y_offset = 4;
  const size_t z_offset = 8;
  const size_t intensity_offset = 16;
  const size_t t_offset = 20;
  const size_t ring_offset = 26;
  const size_t range_offset = 32;

  // Reserve memory for output data
  // Not actually needed if we already call resize above
  //output_msg.data.reserve(msg->width * msg->height * output_msg.point_step);

  // Iterate over each point
  for (size_t i = 0; i < msg->width * msg->height; ++i) {
    PointXYZIRCAEDT new_point;
    new_point.x = *reinterpret_cast<const float *>(&msg->data[i * msg->point_step + x_offset]);
    new_point.y = *reinterpret_cast<const float *>(&msg->data[i * msg->point_step + y_offset]);
    new_point.z = *reinterpret_cast<const float *>(&msg->data[i * msg->point_step + z_offset]);
    new_point.intensity =
      *reinterpret_cast<const uint8_t *>(&msg->data[i * msg->point_step + intensity_offset]);
    new_point.return_type = 1;  // (TODO): Put the correct value
    new_point.channel =
      *reinterpret_cast<const uint16_t *>(&msg->data[i * msg->point_step + ring_offset]);
    new_point.distance =
      *reinterpret_cast<const float *>(&msg->data[i * msg->point_step + range_offset]);
    new_point.time_stamp =
      *reinterpret_cast<const uint32_t *>(&msg->data[i * msg->point_step + t_offset]);

    // Calculate azimuth and elevation based on x, y, z
    // Leaving them as zeros to optimise conversion speed (TODO: Compute these values when needed)
    new_point.azimuth = 0.0;
    new_point.elevation = 0.0;
    // new_point.azimuth = atan2(new_point.y, new_point.x);
    // new_point.elevation = atan2(new_point.z, sqrt(new_point.x * new_point.x + new_point.y *
    // new_point.y));

    // Copy the new point into the output message
    memcpy(&output_msg.data[i * output_msg.point_step], &new_point, sizeof(PointXYZIRCAEDT));
  }

  auto end = std::chrono::high_resolution_clock::now();

  publisher_->publish(output_msg);

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  double duration_ms = static_cast<double>(duration.count()) / 1000;
  std_msgs::msg::Float64 processing_time_ms;
  processing_time_ms.data = duration_ms;
  computation_time_publisher_->publish(processing_time_ms);
}

}  // namespace autoware_tartan_bridge

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware_tartan_bridge::PointTypeConverter)
