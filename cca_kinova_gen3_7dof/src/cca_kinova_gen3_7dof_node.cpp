/*
 * Programmatic CCA Cartesian Goal Planner
 *
 * Uses the intended CCA ROS interface:
 *     cca_ros::CcaRos::plan(PlanningRequest)
 *
 * No custom ROS service is involved.
 *
 * Goal coordinates are expressed in the planning group's configured
 * reference frame. For this SCRAT2 configuration:
 *     planning group: right_arm
 *     reference frame: right_base_link
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>


// ============================================================
// USER CONFIGURATION: EDIT YOUR CARTESIAN GOAL HERE
// ============================================================

// Target position in right_base_link, in meters.
constexpr double GOAL_X = 0.40;
constexpr double GOAL_Y = -0.30;
constexpr double GOAL_Z = 0.48;

// Target orientation as a quaternion (x, y, z, w).
// This example is identity orientation.
// Replace with your desired normalized quaternion.
constexpr double GOAL_QX = 0.99;
constexpr double GOAL_QY = -0.11;
constexpr double GOAL_QZ = 0.0;
constexpr double GOAL_QW = 0.02;

// false: plan and visualize, but do not move the robot.
// true: plan, validate, and execute the trajectory.
constexpr bool EXECUTE_TRAJECTORY = true;

// CCA planning settings.
constexpr int TRAJECTORY_DENSITY = 20;
constexpr int IK_MAX_ITERATIONS = 1000;
constexpr double PLANNING_ACCURACY = 0.10;


// ============================================================
// CCA NODE
// ============================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    // CcaRos loads the robot-specific planning group and kinematic
    // configuration from the parameters supplied by the launch file.
    auto cca = std::make_shared<cca_ros::CcaRos>(
        "cca_ros", node_options);

    auto logger = cca->get_node()->get_logger();

    RCLCPP_INFO(logger, "CCA Cartesian goal planner started.");

    // CcaRos needs its ROS node spinning to receive joint states,
    // TF information, and other ROS callbacks.
    std::jthread spinner_thread(
        [cca]()
        {
            rclcpp::spin(cca->get_node());
        });

    // --------------------------------------------------------
    // Validate the requested orientation.
    // --------------------------------------------------------

    Eigen::Quaterniond target_orientation(
        GOAL_QW,
        GOAL_QX,
        GOAL_QY,
        GOAL_QZ);

    if (!std::isfinite(target_orientation.norm()) ||
        target_orientation.norm() < 1e-8)
    {
        RCLCPP_ERROR(logger, "Invalid target quaternion.");
        rclcpp::shutdown();
        return 1;
    }

    target_orientation.normalize();

    // --------------------------------------------------------
    // Construct the CCA planning request.
    // --------------------------------------------------------

    cca_ros::PlanningRequest request;

    request.planning_group = "right_arm";

    request.task_description =
        cc_affordance_planner::TaskDescription(
            cc_affordance_planner::PlanningType::CARTESIAN_GOAL);

    // CCA expects a homogeneous transform representing the
    // desired Cartesian pose.
    //
    // The transform is expressed in the configured reference
    // frame: right_base_link for this planning group.
    request.task_description.goal.canonical_pose =
        Eigen::Matrix4d::Identity();

    request.task_description.goal.canonical_pose
        .block<3, 3>(0, 0) =
            target_orientation.toRotationMatrix();

    request.task_description.goal.canonical_pose
        .block<3, 1>(0, 3) =
            Eigen::Vector3d(GOAL_X, GOAL_Y, GOAL_Z);

    request.task_description.trajectory_density =
        TRAJECTORY_DENSITY;

    // Optional planner configuration.
    request.planner_config.ik_max_itr =
        IK_MAX_ITERATIONS;

    request.planner_config.accuracy =
        PLANNING_ACCURACY;

    // Use CCA's automatically retrieved current robot state.
    // Do not populate request.start_state manually.
    request.visualize_trajectory = true;
    request.execute_trajectory = EXECUTE_TRAJECTORY;

    request.execution_timeout = std::chrono::seconds(90);

    // --------------------------------------------------------
    // Print the requested goal.
    // --------------------------------------------------------

    RCLCPP_INFO(
        logger,
        "Planning Cartesian goal for group: %s",
        request.planning_group.c_str());

    RCLCPP_INFO(
        logger,
        "Goal position in right_base_link: "
        "[%.4f, %.4f, %.4f]",
        GOAL_X, GOAL_Y, GOAL_Z);

    RCLCPP_INFO(
        logger,
        "Goal quaternion (x,y,z,w): "
        "[%.4f, %.4f, %.4f, %.4f]",
        target_orientation.x(),
        target_orientation.y(),
        target_orientation.z(),
        target_orientation.w());

    RCLCPP_INFO(
        logger,
        "Execute trajectory: %s",
        EXECUTE_TRAJECTORY ? "true" : "false");

    // --------------------------------------------------------
    // Call CCA directly.
    // --------------------------------------------------------

    RCLCPP_INFO(logger, "Calling CCA planner...");

    cca_ros::PlanningResponse response = cca->plan(request);

    if (!response.result.success)
    {
        RCLCPP_ERROR(
            logger,
            "CCA planning/validation/execution request failed.");

        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(logger, "CCA returned a successful result.");

    // --------------------------------------------------------
    // Wait for execution to finish if execution was requested.
    // --------------------------------------------------------

    if (EXECUTE_TRAJECTORY)
    {
        RCLCPP_INFO(logger, "Waiting for robot execution...");

        const auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(65);

        while (rclcpp::ok())
        {
            const cca_ros::Status status = *response.status;

            if (status == cca_ros::Status::SUCCEEDED)
            {
                RCLCPP_INFO(
                    logger,
                    "CCA trajectory execution completed successfully.");
                break;
            }

            if (status == cca_ros::Status::FAILED)
            {
                RCLCPP_ERROR(
                    logger,
                    "CCA trajectory execution failed.");
                rclcpp::shutdown();
                return 1;
            }

            if (std::chrono::steady_clock::now() - start_time > timeout)
            {
                RCLCPP_ERROR(
                    logger,
                    "Timed out waiting for CCA execution.");
                cca->cancel_execution();
                rclcpp::shutdown();
                return 1;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(250));
        }
    }
    else
    {
        RCLCPP_INFO(
            logger,
            "Planning and visualization completed. "
            "Robot was not commanded to move.");
    }

    rclcpp::shutdown();
    return 0;
}