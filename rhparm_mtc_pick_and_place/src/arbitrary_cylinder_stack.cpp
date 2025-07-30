#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <std_msgs/msg/string.hpp> // topic 발행

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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions &options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

  void calculation();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(int level);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  const double gap = 0.00001; // 고정 위치
  const double cylinder_height = 0.03;

  double x_coord[3];
  double y_coord[3];
  const double place_ycoord = 0.13;
  const double place_zcoord[3] = {gap + cylinder_height * 0.5, 2*gap + cylinder_height * 1.5, 3*gap + cylinder_height * 2.5};
  double gripper_angle[3];

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grasp_strategy_publisher_; // Publisher 추가
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
  // 런치 인자에서 값을 받아오도록 파라미터 가져오기
  x_coord[0] = node_->get_parameter("x1_coord").get_parameter_value().get<double>();
  y_coord[0] = node_->get_parameter("y1_coord").get_parameter_value().get<double>();
  x_coord[1] = node_->get_parameter("x2_coord").get_parameter_value().get<double>();
  y_coord[1] = node_->get_parameter("y2_coord").get_parameter_value().get<double>();
  x_coord[2] = node_->get_parameter("x3_coord").get_parameter_value().get<double>();
  y_coord[2] = node_->get_parameter("y3_coord").get_parameter_value().get<double>();

  RCLCPP_INFO(LOGGER, "Received x1_coord: %f", x_coord[0]);
  RCLCPP_INFO(LOGGER, "Received y1_coord: %f", y_coord[0]);
  RCLCPP_INFO(LOGGER, "Received x2_coord: %f", x_coord[1]);
  RCLCPP_INFO(LOGGER, "Received y2_coord: %f", y_coord[1]);
  RCLCPP_INFO(LOGGER, "Received x3_coord: %f", x_coord[2]);
  RCLCPP_INFO(LOGGER, "Received y3_coord: %f", y_coord[2]);

  // Publisher 초기화
  grasp_strategy_publisher_ = node_->create_publisher<std_msgs::msg::String>("/grasp_strategy", 10);
}

void MTCTaskNode::calculation()
{
  double distance[3];
  distance[0] = std::sqrt(x_coord[0] * x_coord[0] + y_coord[0] * y_coord[0]);
  distance[1] = std::sqrt(x_coord[1] * x_coord[1] + y_coord[1] * y_coord[1]);
  distance[2] = std::sqrt(x_coord[2] * x_coord[2] + y_coord[2] * y_coord[2]);

  RCLCPP_INFO(LOGGER, "Distance1: %f", distance[0]);
  RCLCPP_INFO(LOGGER, "Distance2: %f", distance[1]);
  RCLCPP_INFO(LOGGER, "Distance3: %f", distance[2]);

  // 1층
  if (0.10 <= distance[0] && distance[0] < 0.145) gripper_angle[0] = 70.0;
  else if (0.145 <= distance[0] && distance[0] < 0.16) gripper_angle[0] = 65.0;
  else if (0.16 <= distance[0] && distance[0] < 0.18) gripper_angle[0] = 60.0;
  else if (0.18 <= distance[0] && distance[0] <= 0.21) gripper_angle[0] = 55.0;
  else {
      RCLCPP_ERROR(LOGGER, "Invalid distance for Distance1: %f", distance[0]);
      rclcpp::shutdown(); // 노드 종료
      return;
  }

  // 2층
  if (0.12 <= distance[1] && distance[1] < 0.175) gripper_angle[1] = 60.0;
  else if (0.175 <= distance[1] && distance[1] < 0.195) gripper_angle[1] = 55.0;
  else if (0.195 <= distance[1] && distance[1] < 0.210) gripper_angle[1] = 50.0;
  else if (0.210 <= distance[1] && distance[1] < 0.225) gripper_angle[1] = 45.0;
  else if (0.225 <= distance[1] && distance[1] < 0.245) gripper_angle[1] = 35.0;
  else if (0.245 <= distance[1] && distance[1] <= 0.265) gripper_angle[1] = 26.0;
  else {
    RCLCPP_ERROR(LOGGER, "Invalid distance for Distance2: %f", distance[1]);
    rclcpp::shutdown(); // 노드 종료
    return;
  }

  // 3층
  if (0.135 <= distance[2] && distance[2] < 0.210) gripper_angle[2] = 50.0;
  else if (0.210 <= distance[2] && distance[2] < 0.235) gripper_angle[2] = 45.0;
  else if (0.235 <= distance[2] && distance[2] <= 0.240) gripper_angle[2] = 37.0;
  else {
    RCLCPP_ERROR(LOGGER, "Invalid distance for Distance3: %f", distance[2]);
    rclcpp::shutdown(); // 노드 종료
    return;
  }
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object1;
  object1.id = "object1";
  object1.header.frame_id = "world";
  object1.primitives.resize(1);
  object1.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object1.primitives[0].dimensions = {cylinder_height, 0.02};

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = x_coord[0];    // 런치 인자로부터 받은 값 사용
  pose1.position.y = y_coord[0];    // 런치 인자로부터 받은 값 사용
  pose1.position.z = cylinder_height * 0.5 + gap; // 땅바닥에 붙음
  pose1.orientation.w = 1.0;
  object1.pose = pose1;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object1);
  // -----------------
  moveit_msgs::msg::CollisionObject object2;
  object2.id = "object2";
  object2.header.frame_id = "world";
  object2.primitives.resize(1);
  object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object2.primitives[0].dimensions = {cylinder_height, 0.02};

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = x_coord[1];    // 런치 인자로부터 받은 값 사용
  pose2.position.y = y_coord[1];    // 런치 인자로부터 받은 값 사용
  pose2.position.z = cylinder_height * 0.5 + gap; // 땅바닥에 붙음
  pose2.orientation.w = 1.0;
  object2.pose = pose2;
  psi.applyCollisionObject(object2);
  // -----------------
  moveit_msgs::msg::CollisionObject object3;
  object3.id = "object3";
  object3.header.frame_id = "world";
  object3.primitives.resize(1);
  object3.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object3.primitives[0].dimensions = {cylinder_height, 0.02};

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = x_coord[2];    // 런치 인자로부터 받은 값 사용
  pose3.position.y = y_coord[2];    // 런치 인자로부터 받은 값 사용
  pose3.position.z = cylinder_height * 0.5 + gap; // 땅바닥에 붙음
  pose3.orientation.w = 1.0;
  object3.pose = pose3;
  psi.applyCollisionObject(object3);
}

void MTCTaskNode::doTask()
{
  const int MAX_PLAN_ATTEMPTS = 5; // 최대 재시도 횟수

  for (int level = 1; level <= 3; ++level)
  {
    RCLCPP_INFO(LOGGER, "=== Start task for object%d ===", level);

    task_ = createTask(level);

    try
    {
      task_.init();
    }
    catch (const mtc::InitStageException &e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed for object" << level << ": " << e);
      continue; // 초기화 실패 시 다음 물체로 넘어감
    }

    // --- Plan 재시도 로직 ---
    bool plan_success = false;
    int plan_attempts = 0;
    while (plan_attempts < MAX_PLAN_ATTEMPTS && !plan_success && rclcpp::ok())
    {
      plan_attempts++;
      RCLCPP_INFO(LOGGER, "Planning attempt %d/%d for object%d...", plan_attempts, MAX_PLAN_ATTEMPTS, level);

      // plan()의 결과가 성공 코드(SUCCESS)인지 확인하여 결과를 bool 타입으로 저장
      plan_success = (task_.plan(10 /* max_solutions */) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

      if (!plan_success && plan_attempts < MAX_PLAN_ATTEMPTS) {
        RCLCPP_WARN(LOGGER, "Planning failed. Retrying in 1 second... ⏳");
        rclcpp::sleep_for(std::chrono::seconds(1)); // 재시도 전 1초 대기
      }
    }

    // 재시도 후에도 plan에 실패하면 다음 물체로 넘어감
    if (!plan_success)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed for object" << level << " after " << plan_attempts << " attempts. Skipping.");
      continue;
    }

    // --- Plan 성공 시 Execute ---
    RCLCPP_INFO(LOGGER, "Planning successful! Executing task for object%d... ✅", level);
    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed for object" << level);
      continue;
    }

    RCLCPP_INFO(LOGGER, "=== Successfully executed object%d ===", level);
  }
  RCLCPP_INFO(LOGGER, "Task executed successfully.");
  rclcpp::shutdown(); // 노드 종료
  return;
}

mtc::Task MTCTaskNode::createTask(int level)
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto &arm_group_name = "arm";
  const auto &hand_group_name = "hand";
  const auto &hand_frame = "gripper_base";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", "end_effector");
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.00005);

  std::string object_name = "object" + std::to_string(level);
  mtc::Stage *attach_object_stage = nullptr;


  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));


  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick", mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
  stage_move_to_pick->setTimeout(15.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.001, 0.2);
      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

      /****************************************************
       *                Generate Grasp Pose               *
       ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object_name);
      stage->setAngleDelta(M_PI / 24);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(-gripper_angle[level - 1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();

      if ((level == 2 || level == 3) && gripper_angle[level - 1] < 40) {
        auto msg = std_msgs::msg::String();
        msg.data = "z_down";
        grasp_strategy_publisher_->publish(msg);
        RCLCPP_INFO(LOGGER, "grasp_frame_transform.translation : z_down");
        grasp_frame_transform.translation().x() = 0.055;
        grasp_frame_transform.translation().z() = -0.006;
      } else {
        auto msg = std_msgs::msg::String();
        msg.data = "z_zero";
        grasp_strategy_publisher_->publish(msg);
        RCLCPP_INFO(LOGGER, "grasp_frame_transform.translation : z_zero");
        grasp_frame_transform.translation().x() = 0.055;
        grasp_frame_transform.translation().z() = -0.003;
      }

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    // -- allow collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(object_name,
                             task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      std::map<std::string, double> goal_joints = {{"slider_1", 0.019}};
      stage->setGoal(goal_joints);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
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

  // ======== move to place ========
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place", mtc::stages::Connect::GroupPlannerVector{
                           {arm_group_name, sampling_planner},
                           {hand_group_name, sampling_planner}});
  stage_move_to_place->setTimeout(15.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("descend object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.010, 0.2);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "descend_object");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // -- place pose & ComputeIK
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_name);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = 0.0;
      target_pose_msg.pose.position.y = -place_ycoord;
      target_pose_msg.pose.position.z = place_zcoord[level - 1];
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(object_name);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      // 목표 joint 값 정의
      std::map<std::string, double> goal_joints = {
          {"slider_1", 0.026}};
      stage->setGoal(goal_joints);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(object_name,
                             task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                             false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name, hand_frame);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  if (level == 3) {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("rest");
    task.add(std::move(stage));
  }


  return task;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // 이 옵션은 명령줄에서 전달된 파라미터를 자동으로 노드에 선언하도록 합니다.
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                   {
  executor.add_node(mtc_task_node->getNodeBaseInterface());
  executor.spin();
  executor.remove_node(mtc_task_node->getNodeBaseInterface()); });

  mtc_task_node->calculation();
  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
