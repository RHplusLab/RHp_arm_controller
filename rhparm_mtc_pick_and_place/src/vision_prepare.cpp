#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <thread>

// M_PI (3.14159...) 같은 수학 상수를 사용하기 위해 필요
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 로거 설정
static const rclcpp::Logger LOGGER = rclcpp::get_logger("simple_joint_move_node");

int main(int argc, char** argv)
{
    // 1. ROS2 및 노드 초기화
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("simple_joint_move_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // MoveGroupInterface를 사용하려면 별도의 스레드에서 Executor를 실행해야 합니다.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spin_thread = std::make_unique<std::thread>([&executor]() { executor.spin(); });

    // 2. MoveGroupInterface 설정
    // 플래닝 그룹 이름은 사용 중인 로봇의 SRDF 파일에 정의된 이름과 일치해야 합니다. (예: "arm", "panda_arm")
    // 기존 코드에서 "arm"을 사용했으므로 그대로 사용합니다.
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "arm");

    // 3. 목표 조인트 각도 설정
    RCLCPP_INFO(LOGGER, "Setting target joint values...");

    // 먼저 현재 조인트 값들을 가져옵니다.
    std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

    // 로봇의 조인트 이름들을 가져옵니다. (디버깅용)
    const std::vector<std::string>& joint_names = move_group_interface.getJointNames();

    // 조인트가 6개라고 가정하고 3번째 조인트의 각도를 변경합니다.
    // *** 중요: `joint_names[2]`가 실제 3번 모터의 조인트 이름이 맞는지 확인하세요. ***
    // 로봇 설정에 따라 인덱스(2)가 달라질 수 있습니다.
    if (joint_group_positions.size() > 2)
    {
        // 3번 조인트의 목표 각도를 -90도(라디안)로 설정
        joint_group_positions[2] = -M_PI * 4.0 / 12.0;
        RCLCPP_INFO(LOGGER, "Targeting joint '%s' to -75 degrees.", joint_names[2].c_str());
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "The 'arm' group has fewer than 3 joints. Cannot set target.");
        rclcpp::shutdown();
        spin_thread->join();
        return 1;
    }

    // MoveGroup에 목표 조인트 상태를 설정합니다.
    move_group_interface.setJointValueTarget(joint_group_positions);

    // 4. 플래닝 및 실행
    RCLCPP_INFO(LOGGER, "Planning and moving to the target...");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        // 플래닝에 성공하면 로봇을 실제로 움직입니다.
        moveit::core::MoveItErrorCode move_result = move_group_interface.move();
        if (move_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(LOGGER, "Movement executed successfully!");
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Movement failed with error code: %d", move_result.val);
        }
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Failed to plan the motion to the target.");
    }

    // 5. 종료
    rclcpp::shutdown();
    spin_thread->join(); // 스레드가 안전하게 종료될 때까지 대기
    RCLCPP_INFO(LOGGER, "Node has shut down.");
    return 0;
}
