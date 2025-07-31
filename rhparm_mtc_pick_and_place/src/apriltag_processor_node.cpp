// src/apriltag_processor_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <rhp_apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

class ApriltagProcessor : public rclcpp::Node {
public:
  ApriltagProcessor()
  : Node("apriltag_processor_node") {
    subscription_ = this->create_subscription<rhp_apriltag_msgs::msg::AprilTagDetectionArray>(
      "/apriltag_detections", 10,
      std::bind(&ApriltagProcessor::callback, this, std::placeholders::_1));
  }

private:
  void callback(const rhp_apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    // 1) 감지된 태그 복사·정렬 (ID 오름차순)
    auto dets = msg->detections;
    std::sort(dets.begin(), dets.end(),
      [](const auto &a, const auto &b){ return a.id < b.id; });

    // 2) 레벨별 거리 범위 정의
    const double min_th[3] = {0.10, 0.120, 0.135};
    const double max_th[3] = {0.21, 0.265, 0.260};

    // 3) 최대 3개 태그를 한 줄로 출력
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < dets.size() && i < 3; ++i) {
      const auto &det = dets[i];

      // [수정] ID가 유효한 범위(1~3)에 있는지 확인
      if (det.id < 1 || det.id > 3) {
        RCLCPP_WARN(get_logger(), "Skipping invalid tag ID: %d", det.id);
        continue; // 유효하지 않은 ID는 건너뛰기
      }

      // [수정] det.id를 기반으로 올바른 배열 인덱스 계산
      int th_idx = det.id - 1;

      double x    = det.pose.pose.pose.position.x;
      double y    = det.pose.pose.pose.position.y;
      double z    = det.pose.pose.pose.position.z;
      double dist = std::sqrt(x*x + y*y);

      // [수정] 올바른 인덱스(th_idx)를 사용하여 OK/Out 판정
      const char *mark = (dist >= min_th[th_idx] && dist <= max_th[th_idx]) ? "O" : "X";

      // 정보 추가
      oss << "ID:" << det.id
          << " x:" << x
          << " y:" << y
          << " z:" << z
          << " dist:" << dist
          << " " << mark;

      if (i + 1 < dets.size() && i + 1 < 3)
        oss << "  |  ";
    }

    // 한 줄로 출력
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  rclcpp::Subscription<rhp_apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApriltagProcessor>());
  rclcpp::shutdown();
  return 0;
}
