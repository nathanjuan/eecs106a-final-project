#include <abb_libegm/egm_controller_interface.h>
#include <abb_robot_sandbox/ControlAction.h>
#include <actionlib/server/simple_action_server.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

class EGMController {
public:
  EGMController()
      : p_nh_("~"), freq_(30.0), egm_port_(6511), egm_rate_(250.0),
        action_server_(nh_, "controller",
                       boost::bind(&EGMController::ExecuteCallback, this, _1),
                       false) {
    action_server_.start();
  }

  ~EGMController() {
    if (egm_interface_) {
      delete egm_interface_;
      // egm_interface_ = nullptr;
      egm_interface_ = 0;
    }

    io_service_.stop();
    thread_group_.join_all();
  }

  bool Init() {
    if (!p_nh_.getParam("freq", freq_))
      return false;

    if (!p_nh_.getParam("egm/port", egm_port_))
      return false;

    if (!p_nh_.getParam("egm/rate", egm_rate_))
      return false;

    ROS_INFO_STREAM("[EGMController] Initializing EGM interface on port "
                    << egm_port_ << "...");
    egm_interface_ =
        new abb::egm::EGMControllerInterface(io_service_, egm_port_);

    if (!egm_interface_->isInitialized()) {
      ROS_ERROR_STREAM("[EGMController] EGM interface initializion failed "
                       "(port already bound?)");
      return false;
    }

    thread_group_.create_thread(
        boost::bind(&boost::asio::io_service::run, &io_service_));
    ROS_INFO_STREAM("...done!");

    std::string joint_states_topic;
    if (!p_nh_.getParam("topics/joint_states", joint_states_topic))
      return false;
    joints_pub_ =
        nh_.advertise<sensor_msgs::JointState>(joint_states_topic, 1000);

    return true;
  }

  void ExecuteCallback(const actionlib::SimpleActionServer<
                       abb_robot_sandbox::ControlAction>::GoalConstPtr &goal) {
    // Wait for EGM communcations to start.
    ROS_INFO("[EGMController] Waiting for EGM communcations...");
    bool wait = true;
    while (ros::ok() && wait) {
      if (action_server_.isPreemptRequested()) {
        ROS_WARN("[EGMController] Preempted!");
        action_server_.setPreempted();
        return;
      }

      if (egm_interface_->isConnected()) {
        if (egm_interface_->getStatus().rapid_execution_state() ==
            abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED) {
          ROS_WARN("[EGMController] RAPID execution state is UNDEFINED!");
        } else {
          wait = egm_interface_->getStatus().rapid_execution_state() !=
                 abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
        }
      }

      ros::Duration(0.5).sleep();
    }

    // Execute the trajectory.
    abb::egm::wrapper::Input input;
    abb::egm::wrapper::Output output;
    int sequence_number = 0;
    int first_sequence_number = -1;
    double time = 0.0;
    bool success = true;
    while (ros::ok()) {
      if (action_server_.isPreemptRequested()) {
        ROS_WARN("[EGMController] Preempted!");
        action_server_.setPreempted();
        success = false;
        break;
      }

      if (egm_interface_->waitForMessage(500)) {
        egm_interface_->read(&input);
        sequence_number = input.header().sequence_number();

        if (first_sequence_number < 0) {
          first_sequence_number = sequence_number;
          ROS_INFO_STREAM("[EGMController] First sequence number is "
                          << first_sequence_number);
        }

        time = static_cast<double>(sequence_number - first_sequence_number) /
               egm_rate_;

        if (goal->trajectory.points.back().time_from_start.toSec() <= time) {
          ROS_INFO("[EGMController] Reached end of trajectory!");
          break;
        }

        // Get the waypoint to send to robot.
        int index = -1;
        for (int i = 0; i < goal->trajectory.points.size() - 1; ++i) {
          const trajectory_msgs::JointTrajectoryPoint &point =
              goal->trajectory.points[i];
          const trajectory_msgs::JointTrajectoryPoint &next_point =
              goal->trajectory.points[i + 1];
          if (point.time_from_start.toSec() <= time &&
              time < next_point.time_from_start.toSec()) {
            index = i;
            break;
          }
        }

        if (index < 0) {
          ROS_WARN("[EGMController] Failed to get waypoint!");
          success = false;
          break;
        }

        const trajectory_msgs::JointTrajectoryPoint &point =
            goal->trajectory.points[index];

        output.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(
            input.feedback().robot().joints().position());
        for (int i = 0; i < 6; ++i) {
          output.mutable_robot()
              ->mutable_joints()
              ->mutable_position()
              ->set_values(i, angles::to_degrees(point.positions[i]));
        }

        egm_interface_->write(output);
      }
    }

    ROS_INFO("[EGMController] Trajectory execution complete!");
    abb_robot_sandbox::ControlResult result;
    if (success)
      result.result = abb_robot_sandbox::ControlResult::SUCCESS;
    else
      result.result = abb_robot_sandbox::ControlResult::FAILURE;

    action_server_.setSucceeded(result);
  }

  void Run() {
    sensor_msgs::JointState joints_msg;
    joints_msg.header.seq = 0;
    for (int i = 0; i < 6; ++i) {
      joints_msg.name.push_back("joint_" + boost::to_string(i + 1));
      joints_msg.position.push_back(0);
      joints_msg.velocity.push_back(0);
      joints_msg.effort.push_back(0);
    }

    ros::Rate rate(freq_);
    while (ros::ok()) {
      // TODO Update joints?

      joints_msg.header.stamp = ros::Time::now();
      joints_msg.header.seq += 1;
      joints_pub_.publish(joints_msg);
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle p_nh_;
  double freq_;
  int egm_port_;
  double egm_rate_;
  ros::Publisher joints_pub_;

  actionlib::SimpleActionServer<abb_robot_sandbox::ControlAction>
      action_server_;

  boost::asio::io_service io_service_;
  boost::thread_group thread_group_;
  abb::egm::EGMControllerInterface *egm_interface_;
  int seq_num_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "egm_controller");

  EGMController controller;
  if (!controller.Init()) {
    ROS_ERROR("[EGMController] Failed to initialize node!");
    return -1;
  }

  controller.Run();

  return 0;
}
