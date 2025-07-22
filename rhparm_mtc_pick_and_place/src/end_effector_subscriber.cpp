#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <array>
#include <cmath>
#include <iostream>

class EndEffectorSubscriber : public rclcpp::Node
{
public:
    EndEffectorSubscriber()
        : Node("end_effector_subscriber"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&EndEffectorSubscriber::on_timer, this));
    }

private:
    void on_timer();

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

void EndEffectorSubscriber::on_timer()
{
    const std::string source_frame = "world";
    const std::string target_frame = "link6";

    try
    {
        auto tf = tf_buffer_.lookupTransform(
            source_frame, target_frame, tf2::TimePointZero);

        auto &t = tf.transform.translation;
        auto &r = tf.transform.rotation;

        double x = r.x;
        double y = r.y;
        double z = r.z;
        double w = r.w;

        std::array<std::array<double, 3>, 3> R;

        R[0][0] = 1 - 2 * y * y - 2 * z * z;
        R[0][1] = 2 * x * y - 2 * z * w;
        R[0][2] = 2 * x * z + 2 * y * w;

        R[1][0] = 2 * x * y + 2 * z * w;
        R[1][1] = 1 - 2 * x * x - 2 * z * z;
        R[1][2] = 2 * y * z - 2 * x * w;

        R[2][0] = 2 * x * z - 2 * y * w;
        R[2][1] = 2 * y * z + 2 * x * w;
        R[2][2] = 1 - 2 * x * x - 2 * y * y;

        std::array<double, 3> v = {0.1014, 0.0, 0.0201};
        std::array<double, 3> result = {0.0, 0.0, 0.0};
        double cos_z = R[2][2];
        double angle_rad = std::acos(std::clamp(cos_z, -1.0, 1.0));
        double angle_deg = angle_rad * 180.0 / M_PI;
        for (int i = 0; i < 3; ++i)
        {
            // link 6 위치 + 행렬연산결과
            if (i == 0)
                result[i] += t.x;
            else if (i == 1)
                result[i] += t.y;
            else if (i == 2)
                result[i] += t.z;

            // ✅ R * v
            for (int j = 0; j < 3; ++j)
                result[i] += R[i][j] * v[j];
        }

        // translation 기본
        RCLCPP_INFO(get_logger(),
                    "pose → x : %.3f, y : %.3f, z : %.3f / rotation → %.1f°",
                    result[0], result[1], result[2], angle_deg);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(get_logger(), "Lookup failed: %s", ex.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndEffectorSubscriber>());
    rclcpp::shutdown();
    return 0;
}
