#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

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
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  double x1_coord_;
  double y1_coord_;
  double x2_coord_;
  double y2_coord_;
  double x3_coord_;
  double y3_coord_;
  double place_coord_y;
  double place_coord_z[3];
  int gripper_angle[3];
  const double cylinder_height = 0.03;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
  // 런치 인자에서 값을 받아오도록 파라미터 가져오기
  x1_coord_ = node_->get_parameter("x1_coord").get_parameter_value().get<double>();
  y1_coord_ = node_->get_parameter("y1_coord").get_parameter_value().get<double>();
  x2_coord_ = node_->get_parameter("x2_coord").get_parameter_value().get<double>();
  y2_coord_ = node_->get_parameter("y2_coord").get_parameter_value().get<double>();
  x3_coord_ = node_->get_parameter("x3_coord").get_parameter_value().get<double>();
  y3_coord_ = node_->get_parameter("y3_coord").get_parameter_value().get<double>();

  RCLCPP_INFO(LOGGER, "Received x1_coord: %f", x1_coord_);
  RCLCPP_INFO(LOGGER, "Received y1_coord: %f", y1_coord_);
  RCLCPP_INFO(LOGGER, "Received x2_coord: %f", x2_coord_);
  RCLCPP_INFO(LOGGER, "Received y2_coord: %f", y2_coord_);
  RCLCPP_INFO(LOGGER, "Received x3_coord: %f", x3_coord_);
  RCLCPP_INFO(LOGGER, "Received y3_coord: %f", y3_coord_);
}

void MTCTaskNode::calculation()
{
  double distance1 = std::sqrt(x1_coord_ * x1_coord_ + y1_coord_ * y1_coord_);
  double distance2 = std::sqrt(x2_coord_ * x2_coord_ + y2_coord_ * y2_coord_);
  double distance3 = std::sqrt(x3_coord_ * x3_coord_ + y3_coord_ * y3_coord_);

  RCLCPP_INFO(LOGGER, "Distance1: %f", distance1);
  RCLCPP_INFO(LOGGER, "Distance2: %f", distance2);
  RCLCPP_INFO(LOGGER, "Distance3: %f", distance3);

  if (0.11 <= distance1 && distance1 < 0.16)
  {
    gripper_angle[0] = 70;
    place_coord_y = 0.11;
    place_coord_z[0] = 0.0;
  }
  else if (0.16 <= distance1 && distance1 < 0.19)
  {
    gripper_angle[0] = 60;
    place_coord_y = 0.13;
    place_coord_z[0] = 0.0;
  }
  else if (0.19 <= distance1 && distance1 < 0.20)
  {
    gripper_angle[0] = 50;
    place_coord_y = 0.135;
    place_coord_z[0] = 0.0;
  }
  else
  {
    rclcpp::shutdown(); // 노드 종료
    return;
  }

  if (0.11 <= distance2 && distance2 < 0.16)
  {
    gripper_angle[1] = 70;
    place_coord_z[1] = cylinder_height;
  }
  else if (0.16 <= distance2 && distance2 < 0.19)
  {
    gripper_angle[1] = 60;
    place_coord_z[1] = cylinder_height;
  }
  else if (0.19 <= distance2 && distance2 < 0.20)
  {
    gripper_angle[1] = 50;
    place_coord_z[1] = cylinder_height;
  }
  else
  {
    rclcpp::shutdown(); // 노드 종료
    return;
  }

  if (0.11 <= distance3 && distance3 < 0.16)
  {
    gripper_angle[2] = 70;
    place_coord_z[2] = cylinder_height*2;
  }
  else if (0.16 <= distance3 && distance3 < 0.19)
  {
    gripper_angle[2] = 60;
    place_coord_z[2] = cylinder_height*2;
  }
  else if (0.19 <= distance3 && distance3 < 0.20)
  {
    gripper_angle[2] = 50;
    place_coord_z[2] = cylinder_height*2;
  }
  else
  {
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
  pose1.position.x = x1_coord_;    // 런치 인자로부터 받은 값 사용
  pose1.position.y = y1_coord_;    // 런치 인자로부터 받은 값 사용
  pose1.position.z = cylinder_height/2 + 0.001; // 땅바닥에 붙음
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
  pose2.position.x = x2_coord_;    // 런치 인자로부터 받은 값 사용
  pose2.position.y = y2_coord_;    // 런치 인자로부터 받은 값 사용
  pose2.position.z = cylinder_height/2 + 0.001; // 땅바닥에 붙음
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
  pose3.position.x = x3_coord_;    // 런치 인자로부터 받은 값 사용
  pose3.position.y = y3_coord_;    // 런치 인자로부터 받은 값 사용
  pose3.position.z = cylinder_height/2 + 0.001; // 땅바닥에 붙음
  pose3.orientation.w = 1.0;
  object3.pose = pose3;
  psi.applyCollisionObject(object3);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(2 /* max_solutions */))
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

  const auto &arm_group_name = "arm";
  const auto &hand_group_name = "hand";
  const auto &hand_frame = "gripper_base";

  // Set task properties
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

  for (int level = 1; level <= 3; ++level)
  {
    std::string object_name = "object" + std::to_string(level); // obect1, object2, object3

    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));

    mtc::Stage *attach_object_stage =
        nullptr; // Forward attach_object_stage to place pose generator

    // This is an example of SerialContainer usage. It's not strictly needed here.
    // In fact, `task` itself is a SerialContainer by default.
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
      grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef", "group", "ik_frame"});

      {

        auto stage =
            std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
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
        // Sample grasp pose
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose"); // name
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose"); // name, value
        stage->setPreGraspPose("open");
        stage->setObject(object_name);
        stage->setAngleDelta(M_PI / 24);             // angle_step
        stage->setMonitoredStage(current_state_ptr); // Hook into current state

        // This is the transform from the object frame to the end-effector frame
        Eigen::Isometry3d grasp_frame_transform;
        Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(-gripper_angle[level - 1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
        grasp_frame_transform.linear() = q.matrix();
        grasp_frame_transform.translation().x() = 0.05;

        // Compute IK

        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        grasp->insert(std::move(wrapper));
      }

      {

        auto stage =
            std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
        stage->allowCollisions(object_name,
                               task.getRobotModel()
                                   ->getJointModelGroup(hand_group_name)
                                   ->getLinkModelNamesWithCollisionGeometry(),
                               true);
        grasp->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        stage->setGroup(hand_group_name);
        // 목표 joint 값 정의
        std::map<std::string, double> goal_joints = {
            {"slider_1", 0.019}};
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
        auto stage =
            std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setMinMaxDistance(0.0, 0.2);
        stage->setIKFrame(hand_frame);
        stage->properties().set("marker_ns", "lift_object");

        // Set upward direction
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
          mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner},
                                                   {hand_group_name, sampling_planner}});

      stage_move_to_place->setTimeout(5.0);
      stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage_move_to_place));
    }

    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
      place->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef", "group", "ik_frame"});

      /****************************************************
       *                Generate Place Pose               *
       ***************************************************/
      {
        // Sample place pose
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(object_name);

        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "world";
        target_pose_msg.pose.position.x = 0.0;
        target_pose_msg.pose.position.y = -place_coord_y;
        target_pose_msg.pose.position.z = place_coord_z[level - 1] + 0.001;
        target_pose_msg.pose.orientation.w = 1.0;
        stage->setPose(target_pose_msg);
        stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

        // Compute IK
        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
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
        stage->setGoal("open");
        place->insert(std::move(stage));
      }

      {
        auto stage =
            std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
        stage->allowCollisions(object_name,
                               task.getRobotModel()
                                   ->getJointModelGroup(hand_group_name)
                                   ->getLinkModelNamesWithCollisionGeometry(),
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

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setGoal("rest");
      task.add(std::move(stage));
    }
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
