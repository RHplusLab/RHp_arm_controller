#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <rhp_apriltag_msgs/msg/april_tag_detection_array.hpp> // AprilTag message header
#include <chrono>
#include <thread>
#include <atomic>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

// Use a dedicated logger
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void setupPlanningScene();
  void doTask();
  void calculation();

private:
  void apriltagCallback(const rhp_apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  mtc::Task createTask();

  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
  rclcpp::Subscription<rhp_apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr a_tag_sub_;

  double x_coord_ = 0.0;
  double y_coord_ = 0.0;
  double place_coord;
  int gripper_angle;
  std::atomic<bool> task_triggered_{false}; // Flag to ensure the task runs only once
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  RCLCPP_INFO(LOGGER, "Waiting for AprilTag detections...");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  a_tag_sub_ = node_->create_subscription<rhp_apriltag_msgs::msg::AprilTagDetectionArray>(
      "/apriltag_detections", qos,
      std::bind(&MTCTaskNode::apriltagCallback, this, std::placeholders::_1));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::apriltagCallback(const rhp_apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    // If the task has already been triggered, do nothing.
    if (task_triggered_.load()) {
        return;
    }

    const auto& detections = msg->detections;
    for (const auto& detection : detections)
    {
        if (detection.id == 1)
        {
            // Set the flag to true to prevent this block from running again.
            task_triggered_ = true;

            // Store the coordinates from the detected tag
            x_coord_ = detection.pose.pose.pose.position.x;
            y_coord_ = detection.pose.pose.pose.position.y;

            RCLCPP_INFO(LOGGER, "Captured Tag 0 at [x: %f, y: %f]. Starting Pick and Place.", x_coord_, y_coord_);

            // Unsubscribe to stop receiving messages
            a_tag_sub_.reset();

            // Run the MTC task sequence
            calculation();
            setupPlanningScene();
            doTask();

            RCLCPP_INFO(LOGGER, "Task finished. Shutting down.");
            return;
        }
    }
}

void MTCTaskNode::calculation()
{
  double distance = std::sqrt(x_coord_ * x_coord_ + y_coord_ * y_coord_);

    if (0.11 <= distance && distance < 0.16) {
        gripper_angle = 70;
        place_coord = 0.11;
    } else if (0.16 <= distance && distance < 0.19) {
        gripper_angle = 60;
        place_coord = 0.13;
    } else if (0.19 <= distance && distance < 0.20) {
        gripper_angle = 50;
        place_coord = 0.135;
    }
    else {
        rclcpp::shutdown();  // 노드 종료
        return;
    }
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.04, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = x_coord_; // Use captured x-coordinate
  pose.position.y = y_coord_; // Use captured y-coordinate
  pose.position.z = 0.02 + 0.001;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(10 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "gripper_base";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", "end_effector");
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.00005);


  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));


  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });


    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.001, 0.2);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose"); // name
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose"); // name, value
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 24); // angle_step
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(-gripper_angle* M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().x() = 0.045;

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      std::map<std::string, double> goal_joints = {
        {"slider_1", 0.019}
      };
      stage->setGoal(goal_joints);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.2);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, sampling_planner } });

    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = 0.0;
      target_pose_msg.pose.position.y = -place_coord;
      target_pose_msg.pose.position.z = 0.020 + 0.001;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);

      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("rest");
    task.add(std::move(stage));
  }
  return task;
}

int main(int argc, char** argv)
{
  // 1. ROS 초기화
  rclcpp::init(argc, argv);

  // 2. 노드 생성
  // NodeOptions를 여기서 직접 설정하여 파라미터 문제를 원천 차단합니다.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  // 3. 실행기(Executor) 생성 및 노드 추가
  // MultiThreadedExecutor는 여러 콜백을 동시에 처리할 수 있어 안정적입니다.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_task_node->getNodeBaseInterface());

  // 4. 카운트다운
  for (int i = 3; i > 0; --i) {
      RCLCPP_INFO(LOGGER, "Capturing in %d...", i);
      // rclcpp::ok()를 확인하여 중간에 종료 신호가 오면 멈춥니다.
      if (!rclcpp::ok()) {
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  if (rclcpp::ok()) {
    RCLCPP_INFO(LOGGER, "Capturing pose now! Waiting for topic...");
  }

  // 5. 실행기 실행 (Spin)
  // 콜백 함수 내부에서 rclcpp::shutdown()이 호출될 때까지 여기서 대기합니다.
  executor.spin();

  // 6. ROS 종료
  rclcpp::shutdown();
  return 0;
}
