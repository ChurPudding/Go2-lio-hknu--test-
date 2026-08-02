// indoor_pose_subscriber.cpp
// -----------------------------------------------------------------------------
// /indoor/base_pose 를 구독해 로봇의 현재 위치·자세를 얻는 최소 예제.
//
// 이 하나만 구독하면 위치추정이 끝납니다. AMCL 같은 별도 위치추정은 필요 없습니다.
//
// 핵심은 covariance 확인입니다.
//   LIO 는 실패해도 조용히 틀린 좌표를 계속 내보냅니다. 복도 실험에서 7x6 m 를
//   도는데 출발점에서 52 m 벗어난 회차가 있었습니다. 아래 검사가 없으면 로봇은
//   자신 있게 벽으로 갑니다.
//
// 빌드:
//   cd ~/ros2_ws && colcon build --packages-select indoor_pose_subscriber
//   source install/setup.bash
//   ros2 run indoor_pose_subscriber indoor_pose_subscriber
// -----------------------------------------------------------------------------

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

// covariance[0] 이 이 값을 넘으면 위치를 믿을 수 없다.
// 정상 0.01, 이상 1e6 이므로 100 이면 넉넉하다.
static constexpr double COV_THRESHOLD = 100.0;

// 지도 정보 (results/indoor_map_inflated.yaml 과 같아야 한다)
static constexpr double MAP_RESOLUTION = 0.10;    // m/칸
static constexpr double MAP_ORIGIN_X   = -22.2443;
static constexpr double MAP_ORIGIN_Y   = -6.3021;

class IndoorPoseSubscriber : public rclcpp::Node
{
public:
  IndoorPoseSubscriber() : Node("indoor_pose_subscriber")
  {
    // LIO 는 sensor data QoS(BEST_EFFORT)로 발행한다. 맞춰 주지 않으면
    // 구독은 되는데 메시지가 한 개도 오지 않는다.
    auto qos = rclcpp::SensorDataQoS();

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/indoor/base_pose", qos,
      std::bind(&IndoorPoseSubscriber::onPose, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "/indoor/base_pose 구독 시작");
  }

private:
  void onPose(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // ---- 1. 신뢰도 확인 (반드시) ------------------------------------------
    if (msg->pose.covariance[0] > COV_THRESHOLD) {
      if (healthy_) {
        RCLCPP_ERROR(get_logger(),
                     "위치 추정 실패 (covariance=%.1f) — 즉시 정지",
                     msg->pose.covariance[0]);
        healthy_ = false;
      }
      stopRobot();
      return;
    }
    if (!healthy_) {
      RCLCPP_WARN(get_logger(), "위치 신뢰도 회복");
      healthy_ = true;
    }

    // ---- 2. 위치와 헤딩 ----------------------------------------------------
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double z = msg->pose.pose.position.z;

    const auto & q = msg->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // ---- 3. 지도 격자 좌표로 변환 ------------------------------------------
    const int col = static_cast<int>((x - MAP_ORIGIN_X) / MAP_RESOLUTION);
    const int row = static_cast<int>((y - MAP_ORIGIN_Y) / MAP_RESOLUTION);

    // ---- 4. 여기서 A* 를 돌리시면 됩니다 ------------------------------------
    // planPath(row, col, goal_row, goal_col);

    if (++count_ % 15 == 0) {   // 약 1초에 한 번만 출력
      RCLCPP_INFO(get_logger(),
                  "위치 (%.2f, %.2f, %.2f) m  heading %.1f deg  격자 (%d, %d)",
                  x, y, z, yaw * 180.0 / M_PI, row, col);
    }
  }

  void stopRobot()
  {
    // 여기에 정지 명령을 넣으십시오.
    // 예: 속도 0 을 발행하거나, 경로 추종을 중단
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  bool healthy_{true};
  int  count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndoorPoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
