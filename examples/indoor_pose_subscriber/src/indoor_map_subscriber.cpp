// =============================================================================
//  indoor_map_subscriber.cpp
//
//  실내 자율주행에서 A* 경로계획에 필요한 두 가지를 받아오는 예제입니다.
//
//      /indoor/map        점유격자 지도  (어디가 막혔는지)
//      /indoor/base_pose  로봇 위치·자세 (지금 어디에 있는지)
//
//  둘은 같은 좌표계(indoor_map)를 쓰므로 그대로 격자 좌표로 바꿔 A* 에
//  넣으시면 됩니다.
//
//  ---------------------------------------------------------------------------
//  전체 구조
//
//      로봇 라이다·IMU
//            │
//            ▼
//      [Point-LIO]  위치추정 + 지도작성          ← 효신 담당
//            │
//            ├──> /indoor/base_pose  (15 Hz)     ← 이 파일에서 구독
//            └──> /indoor/map        (1회 latch) ← 이 파일에서 구독
//                       │
//                       ▼
//                   A* 경로계획                   ← 팀원A 담당 (여기부터)
//                       │
//                       ▼
//                   속도 명령 → 로봇
//
//  ---------------------------------------------------------------------------
//  빌드
//      cd ~/ros2_ws
//      colcon build --packages-select indoor_pose_subscriber
//      source install/setup.bash
//      ros2 run indoor_pose_subscriber indoor_map_subscriber
// =============================================================================

#include <memory>       // std::make_shared
#include <vector>       // std::vector
#include <cmath>        // std::atan2, M_PI

#include "rclcpp/rclcpp.hpp"                       // ROS2 C++ 기본
#include "nav_msgs/msg/occupancy_grid.hpp"         // 지도 메시지
#include "nav_msgs/msg/odometry.hpp"               // 위치 메시지


// =============================================================================
//  상수
// =============================================================================

// 위치 신뢰도 임계값.
//
//   /indoor/base_pose 의 pose.covariance[0] 에 위치 신뢰도가 실려 옵니다.
//     정상  0.01      (표준편차 0.1 m 라는 뜻)
//     이상  1000000.0 (사실상 무한 = 이 좌표를 믿으면 안 됨)
//
//   두 값의 차이가 1억 배이므로 임계값 100 이면 넉넉합니다.
//
//   왜 필요한가:
//     LIO(라이다 위치추정)는 실패해도 조용히 틀린 좌표를 계속 내보냅니다.
//     실제로 7×6 m 복도를 도는 실험에서 출발점으로부터 52 m 벗어난 회차가
//     있었습니다. 이 검사가 없으면 로봇은 자신 있게 벽으로 걸어갑니다.
static constexpr double COV_THRESHOLD = 100.0;

// 점유 판정 임계값.
//
//   OccupancyGrid 의 data 는 0~100 의 점유 확률(%)입니다.
//   50 이상이면 장애물로 봅니다. Nav2 기본값과 같습니다.
static constexpr int8_t OCCUPIED_THRESHOLD = 50;


// =============================================================================
//  노드
// =============================================================================
class IndoorMapSubscriber : public rclcpp::Node
{
public:
  IndoorMapSubscriber()
  : Node("indoor_map_subscriber")     // 노드 이름. ros2 node list 에 이렇게 뜹니다
  {
    // -------------------------------------------------------------------------
    //  구독 1 — 지도
    //
    //  QoS 를 반드시 transient_local() 로 맞춰야 합니다.
    //
    //  지도는 크고 변하지 않으므로 발행 쪽에서 **한 번만 보내고 붙잡아 둡니다**
    //  (latched). 기본 QoS(VOLATILE)로 구독하면 이미 지나간 메시지를 받을 수
    //  없어서, 노드는 정상적으로 떠 있는데 콜백이 한 번도 불리지 않습니다.
    //  원인을 찾기 어려운 흔한 실수입니다.
    //
    //      rclcpp::QoS(1)          큐 깊이 1 (최신 하나만 있으면 됨)
    //        .transient_local()    늦게 붙어도 마지막 메시지를 받는다
    //        .reliable()           유실되면 재전송한다 (지도는 놓치면 안 됨)
    //
    //  발행 쪽(map_publisher.py)도 같은 설정입니다. QoS 가 어긋나면 연결
    //  자체가 성립하지 않습니다.
    // -------------------------------------------------------------------------
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/indoor/map",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&IndoorMapSubscriber::onMap, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    //  구독 2 — 로봇 위치
    //
    //  이쪽은 SensorDataQoS() 입니다. 지도와 정반대 성격이기 때문입니다.
    //
    //      BEST_EFFORT   유실돼도 재전송하지 않는다
    //      depth 5       오래된 것은 버린다
    //
    //  위치는 15 Hz 로 계속 새로 오므로, 한 개 놓치는 것보다 **늦게 도착하는
    //  것이 더 나쁩니다.** 0.07 초 뒤면 어차피 새 값이 옵니다.
    //
    //  발행 쪽(robot_pose.py)이 sensor data QoS 로 내므로 여기서 기본
    //  QoS(RELIABLE)를 쓰면 역시 연결이 안 됩니다.
    // -------------------------------------------------------------------------
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/indoor/base_pose",
      rclcpp::SensorDataQoS(),
      std::bind(&IndoorMapSubscriber::onPose, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "구독 시작: /indoor/map, /indoor/base_pose");
    RCLCPP_INFO(get_logger(), "지도를 기다리는 중...");
  }

private:
  // ===========================================================================
  //  지도 콜백 — 보통 한 번만 불립니다
  // ===========================================================================
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    //  info 에 격자의 기하 정보가 들어 있습니다.
    //
    //      width       가로 칸 수      예: 370
    //      height      세로 칸 수      예: 332
    //      resolution  한 칸의 크기[m] 예: 0.10  (10 cm)
    //      origin      격자 (0,0) 칸의 왼쪽 아래 모서리가 실좌표 어디인지
    //
    //  origin 이 (-22.24, -6.30) 이라는 것은, 격자 왼쪽 아래 구석이 로봇
    //  시작점 기준 서쪽 22 m, 남쪽 6 m 지점이라는 뜻입니다. 로봇이 시작점에서
    //  출발해 사방으로 돌아다녔기 때문에 원점이 음수입니다.
    // -------------------------------------------------------------------------
    width_  = static_cast<int>(msg->info.width);
    height_ = static_cast<int>(msg->info.height);
    res_    = msg->info.resolution;
    ox_     = msg->info.origin.position.x;
    oy_     = msg->info.origin.position.y;

    // -------------------------------------------------------------------------
    //  data 는 1차원 배열입니다. 2차원으로 보려면 직접 계산해야 합니다.
    //
    //      data[row * width + col]
    //
    //  값 규약 (nav_msgs/OccupancyGrid 표준)
    //        0    자유 공간
    //      100    장애물
    //       -1    미지 (아직 안 가본 곳)
    //
    //  주의 — 이 지도의 row 0 은 **아래쪽**입니다.
    //  같은 지도를 .pgm 파일로 열면 위아래가 뒤집혀 있습니다. 이미지 형식은
    //  위에서 아래로 저장하는 관례가 있어서인데, OccupancyGrid 는 수학 좌표계
    //  관례를 따릅니다. 파일로 읽으실 거라면 뒤집어야 하고, 토픽으로 받으시면
    //  그럴 필요가 없습니다.
    //
    //  미지(-1)를 어떻게 볼지는 선택입니다. 여기서는 **장애물로** 봅니다.
    //  가본 적 없는 곳으로 A* 가 경로를 내면 실제로는 벽일 수 있어 위험합니다.
    //  탐색 주행을 하실 거라면 반대로 자유로 보셔야 합니다.
    // -------------------------------------------------------------------------
    occ_.assign(msg->data.size(), false);

    int n_occupied = 0;
    for (size_t i = 0; i < msg->data.size(); ++i) {
      const int8_t v = msg->data[i];
      occ_[i] = (v < 0) || (v >= OCCUPIED_THRESHOLD);
      if (occ_[i]) ++n_occupied;
    }
    have_map_ = true;

    RCLCPP_INFO(get_logger(),
                "지도 수신: %d x %d 칸, %.2f m/칸 (실제 %.1f x %.1f m)",
                width_, height_, res_, width_ * res_, height_ * res_);
    RCLCPP_INFO(get_logger(),
                "  원점 (%.4f, %.4f) m,  장애물 %d 칸 (%.1f%%)",
                ox_, oy_, n_occupied,
                100.0 * n_occupied / static_cast<double>(occ_.size()));
    RCLCPP_INFO(get_logger(),
                "  ※ 이 지도는 로봇 폭 25 cm 만큼 이미 부풀려져 있습니다.");
  }

  // ===========================================================================
  //  위치 콜백 — 약 15 Hz 로 계속 불립니다
  // ===========================================================================
  void onPose(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 지도가 아직 안 왔으면 격자 좌표를 계산할 수 없습니다.
    if (!have_map_) {
      return;
    }

    // -------------------------------------------------------------------------
    //  1단계 — 위치를 믿어도 되는지 확인
    //
    //  **이 검사를 빼지 마십시오.** 아래 표는 실측값입니다.
    //
    //      환경                 시작점 복귀 오차 (참값 0.19 m)
    //      실내 복도 (성공)      0.24 m
    //      실내 복도 (실패)      18 ~ 52 m      ← 같은 데이터인데 회차마다 갈림
    //      실외 운동장          110 m ~ 27 km
    //
    //  Point-LIO 는 같은 녹화본을 여러 번 돌려도 성공할 때와 실패할 때가
    //  있습니다. 원인이 아직 규명되지 않았고, 이 covariance 검사가 현재로선
    //  유일한 방어선입니다.
    //
    //  THROTTLE 은 같은 로그를 2초에 한 번만 찍으라는 뜻입니다. 15 Hz 로
    //  들어오는 콜백에서 매번 찍으면 화면이 넘칩니다.
    // -------------------------------------------------------------------------
    if (msg->pose.covariance[0] > COV_THRESHOLD) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "위치 추정 실패 (covariance=%.1f) — 로봇을 정지시켜야 합니다",
        msg->pose.covariance[0]);

      stopRobot();
      return;
    }

    // -------------------------------------------------------------------------
    //  2단계 — 위치와 자세 꺼내기
    //
    //  position 은 indoor_map 프레임 기준 미터 단위입니다.
    //  z 는 평지에서 거의 0 이므로 2D 경로계획에서는 안 쓰셔도 됩니다.
    // -------------------------------------------------------------------------
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    // -------------------------------------------------------------------------
    //  헤딩(yaw) 은 쿼터니언에서 뽑습니다.
    //
    //  쿼터니언 (x, y, z, w) 은 3차원 회전을 나타내는 4개 숫자입니다.
    //  짐벌락이 없고 보간이 자연스러워 로봇공학에서 표준으로 씁니다.
    //  대신 사람이 직접 읽을 수 없어서 각도로 바꿔야 합니다.
    //
    //  정식 변환은 이렇습니다.
    //
    //      yaw = atan2( 2(w·z + x·y),  1 - 2(y² + z²) )
    //
    //  로봇이 평지에 있어 roll·pitch 가 0 에 가까우면 아래로 줄일 수 있습니다.
    //
    //      yaw ≈ 2·atan2(z, w)
    //
    //  다만 계단이나 경사에서는 오차가 생기므로 정식 변환을 쓰는 편이 안전합니다.
    // -------------------------------------------------------------------------
    const auto & q = msg->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // -------------------------------------------------------------------------
    //  3단계 — 실좌표를 격자 좌표로
    //
    //  지도와 위치가 **같은 원점을 공유**하기 때문에 변환이 이렇게 간단합니다.
    //  둘 다 LIO 가 동시에 만들어낸 결과라서 서로 어긋날 일이 없습니다.
    //
    //      col = (x - origin.x) / resolution
    //      row = (y - origin.y) / resolution
    //
    //  예: x = 0.0, origin.x = -22.2443, resolution = 0.10
    //      col = (0.0 + 22.2443) / 0.10 = 222
    //
    //  static_cast<int> 는 소수점 이하를 버립니다(내림). 격자 칸 번호이므로
    //  이게 맞습니다. 반올림하면 칸 경계에서 한 칸씩 밀립니다.
    // -------------------------------------------------------------------------
    int row, col;
    toGrid(x, y, row, col);

    // -------------------------------------------------------------------------
    //  4단계 — 여기서 A* 를 돌리시면 됩니다
    //
    //  필요한 것이 다 준비됐습니다.
    //      occupied(row, col)   해당 칸이 막혔는지
    //      width_, height_      격자 범위
    //      (row, col)           현재 위치
    //
    //  경로가 나오면 toWorld() 로 실좌표로 되돌려 로봇에게 보내시면 됩니다.
    // -------------------------------------------------------------------------
    //
    //   auto path = planAStar(row, col, goal_row_, goal_col_);
    //   followPath(path);

    // 로그는 1초에 한 번 정도만 (15 Hz 중 15번째마다)
    if (++count_ % 15 == 0) {
      RCLCPP_INFO(get_logger(),
                  "위치 (%.2f, %.2f) m,  heading %.1f°  →  격자 (%d, %d)%s",
                  x, y, yaw * 180.0 / M_PI, row, col,
                  occupied(row, col) ? "  [주의: 장애물 칸 위]" : "");
    }
  }

  // ===========================================================================
  //  보조 함수
  // ===========================================================================

  // 실좌표(m) → 격자 좌표(칸)
  void toGrid(double x, double y, int & row, int & col) const
  {
    col = static_cast<int>((x - ox_) / res_);
    row = static_cast<int>((y - oy_) / res_);
  }

  // 격자 좌표(칸) → 실좌표(m).  칸의 **중심**을 돌려줍니다.
  //
  //   +0.5 를 더하는 이유: 격자 (3,5) 는 한 점이 아니라 10×10 cm 영역입니다.
  //   그 영역의 왼쪽 아래 모서리가 아니라 중심을 목표로 삼아야 로봇이
  //   칸 가장자리를 스치지 않습니다.
  void toWorld(int row, int col, double & x, double & y) const
  {
    x = ox_ + (col + 0.5) * res_;
    y = oy_ + (row + 0.5) * res_;
  }

  // 해당 칸이 막혔는지.
  //
  //   지도 밖은 **막힌 것으로** 봅니다. A* 가 지도 밖으로 경로를 내는 것을
  //   막기 위해서입니다. 범위 검사를 여기 한 곳에 모아두면 호출하는 쪽에서
  //   매번 확인하지 않아도 되고, 배열 범위 초과 사고도 막힙니다.
  bool occupied(int row, int col) const
  {
    if (row < 0 || col < 0 || row >= height_ || col >= width_) {
      return true;
    }
    return occ_[static_cast<size_t>(row) * width_ + col];
  }

  // 정지 명령을 넣으실 자리입니다.
  //
  //   여기에 속도 0 발행이나 경로 추종 중단을 넣으시면 됩니다.
  //   비워두면 아무 일도 일어나지 않으므로 반드시 채워 주십시오.
  void stopRobot()
  {
    // 예: cmd_pub_->publish(zero_velocity);
  }

  // ===========================================================================
  //  멤버 변수
  //
  //   뒤에 밑줄(_)을 붙이는 것은 ROS2 코딩 규약입니다. 지역 변수와 구분됩니다.
  // ===========================================================================
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      pose_sub_;

  std::vector<bool> occ_;          // 격자. true = 장애물
  int    width_{0};                // 가로 칸 수
  int    height_{0};               // 세로 칸 수
  int    count_{0};                // 로그 빈도 조절용
  double res_{0.1};                // 한 칸 크기 [m]
  double ox_{0.0};                 // 격자 원점 x [m]
  double oy_{0.0};                 // 격자 원점 y [m]
  bool   have_map_{false};         // 지도를 받았는지
};


// =============================================================================
//  진입점
// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                                  // ROS2 초기화
  rclcpp::spin(std::make_shared<IndoorMapSubscriber>());     // 콜백 대기 (Ctrl+C 까지)
  rclcpp::shutdown();                                        // 정리
  return 0;
}
