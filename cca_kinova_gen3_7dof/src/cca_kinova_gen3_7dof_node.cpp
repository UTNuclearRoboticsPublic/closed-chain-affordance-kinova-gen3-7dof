/*************************************/
// Author: Crasun Jans
// Description:
// This node enables users to plan, visualize, and execute robot joint trajectories for specified tasks. The planning
// process utilizes the Closed-chain Affordance model, as described in the paper:
// "A closed-chain approach to generating affordance joint trajectories for robotic manipulators."
//
// Usage Instructions:
// 1. The framework requires only two inputs: planner configuration and task description. See repo README.md Task
// Examples section for task-description examples.
/*************************************/
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>
#include <chrono>
#include <thread>

class CcaRobot : public cca_ros::CcaRos
{
  public:
    explicit CcaRobot(const std::string &node_name, const rclcpp::NodeOptions &node_options)
        : cca_ros::CcaRos(node_name, node_options)
    {
    }

    // Function to run the planner for a given task and/or execute that task on the robot
    bool run(const cca_ros::PlanningRequest &planning_request)
    {

	cca_ros::PlanningResponse response = this->plan(planning_request);
        motion_status_ = response.status;
	return response.result.success;
    }
    // Function overload to plan multiple tasks at once
    bool run(const std::vector<cca_ros::PlanningRequest> &planning_requests)
    {

	cca_ros::PlanningResponse response = this->plan(planning_requests);
        motion_status_ = response.status;
	return response.result.success;
    }

    // Function to block until the robot completes the planned trajectory
    void block_until_trajectory_execution()
    {
        rclcpp::Rate loop_rate(4);
        auto start_time = std::chrono::steady_clock::now();

        while (*motion_status_ != cca_ros::Status::SUCCEEDED)
        {
            if (*motion_status_ == cca_ros::Status::UNKNOWN)
            {
                RCLCPP_ERROR(this->get_node()->get_logger(), "Motion was interrupted mid-execution.");
                auto current_time = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() > 60)
                {
                    RCLCPP_ERROR(this->get_node()->get_logger(), "Timeout waiting for motion to complete.");
                    return;
                }
            }
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_node()->get_logger(), "Exiting due to ROS signal");
                return;
            }
            loop_rate.sleep();
        }
    }

  private:
    std::shared_ptr<cca_ros::Status> motion_status_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<CcaRobot>("cca_ros", node_options);

    // Spin the node so joint states can be read
    std::jthread spinner_thread([node]() {rclcpp::spin(node->get_node()); });

    /// REQUIRED INPUT: Task description. For quick start, the following block provides an example task description to
    /// do a simple linear motion along the z-axis from the current robot configuration. Edit as needed. See this
    /// package's demo folder or repo README.md for various other examples that cover motions including rotation, screw,
    /// cartesian goal, ee orientation jog, etc. It is also possible to plan multiple of these tasks together as a long
    /// joint trajectory.
    ///------------------------------------------------------------------///
    cca_ros::PlanningRequest req;
    req.planning_group = "manipulator";

    // Task description
    // Specify planning type
    req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);
    // Affordance info
    req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    req.task_description.affordance_info.axis = affordance_util::axis_to_vec(affordance_util::Axis::Z);
    req.task_description.affordance_info.location = affordance_util::axis_to_vec(affordance_util::Axis::ORIGIN);
    // Goal
    req.task_description.goal.affordance = 0.1; // 10cm

    // Run CCA planner and executor
    if (node->run(req))
    {
        RCLCPP_INFO(node->get_node()->get_logger(), "Successfully executed %s tasks", req.planning_group.c_str());
        node->block_until_trajectory_execution(); // Optionally, block until execution

    }
    else
    {
        RCLCPP_ERROR(node->get_node()->get_logger(), "CCA action failed for the %s group", req.planning_group.c_str());
    }

    ///------------------------------------------------------------------///

    rclcpp::shutdown();
    return 0;
}
