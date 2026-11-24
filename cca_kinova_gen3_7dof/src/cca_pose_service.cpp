#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cca_ros/cca_ros.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "cca_kinova_gen3_7dof/srv/plan_to_pose.hpp"


class CcaPoseService : public rclcpp::Node
{
public:
    explicit CcaPoseService(
        std::shared_ptr<cca_ros::CcaRos> cca)
        : Node(
            "cca_pose_service",
            rclcpp::NodeOptions().use_global_arguments(false)
        ),
        cca_(std::move(cca))
    {
        // Reject joint-state samples that have been sitting in the
        // service's cache for longer than this duration.
        joint_state_max_age_sec_ =
            this->declare_parameter<double>(
                "joint_state_max_age_sec", 0.5);

        if (joint_state_max_age_sec_ <= 0.0)
        {
            throw std::invalid_argument(
                "joint_state_max_age_sec must be positive.");
        }

        // Read the same topic configured for CCA.
        std::string joint_states_topic;

        if (!cca_->get_node()->get_parameter(
                "cca_joint_states_topic",
                joint_states_topic))
        {
            throw std::runtime_error(
                "Could not read CCA parameter cca_joint_states_topic.");
        }

        // Cache the latest joint-state message independently.
        // We use this snapshot to explicitly populate CCA's start state.
        joint_state_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                joint_states_topic,
                rclcpp::QoS(1000),
                std::bind(
                    &CcaPoseService::joint_state_callback,
                    this,
                    std::placeholders::_1
                )
            );

        service_ = this->create_service<
            cca_kinova_gen3_7dof::srv::PlanToPose>(
                "/cca_kinova/plan_to_pose",
                std::bind(
                    &CcaPoseService::plan_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                )
            );

        RCLCPP_INFO(
            this->get_logger(),
            "CCA pose service ready."
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Monitoring joint states on '%s'. Maximum cached age: %.3f s.",
            joint_states_topic.c_str(),
            joint_state_max_age_sec_
        );
    }


private:

    using PlanToPose =
        cca_kinova_gen3_7dof::srv::PlanToPose;

    using JointState =
        sensor_msgs::msg::JointState;

    std::shared_ptr<cca_ros::CcaRos> cca_;

    rclcpp::Service<PlanToPose>::SharedPtr service_;

    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;

    std::mutex joint_state_mutex_;

    JointState::SharedPtr latest_joint_state_;

    std::chrono::steady_clock::time_point
        joint_state_receive_time_;

    double joint_state_max_age_sec_{0.5};


    // ----------------------------------------------------------
    // Joint-state subscriber
    // ----------------------------------------------------------

    void joint_state_callback(
        const JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);

        latest_joint_state_ = msg;

        // Use local receipt time rather than comparing ROS timestamps.
        // This also works when simulation time is enabled.
        joint_state_receive_time_ =
            std::chrono::steady_clock::now();
    }


    // ----------------------------------------------------------
    // Get a fresh robot start state in CCA's configured joint order
    // ----------------------------------------------------------

    bool get_current_start_state(
        Eigen::VectorXd &start_state,
        std::string &error_message)
    {
        JointState::SharedPtr msg;

        std::chrono::steady_clock::time_point receive_time;

        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);

            if (!latest_joint_state_)
            {
                error_message =
                    "No joint-state message has been received yet.";
                return false;
            }

            msg = latest_joint_state_;
            receive_time = joint_state_receive_time_;
        }

        const double age_sec =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - receive_time
            ).count();

        if (age_sec > joint_state_max_age_sec_)
        {
            std::ostringstream stream;

            stream << "Joint-state message is stale. Age = "
                   << std::fixed << std::setprecision(3)
                   << age_sec << " s; maximum allowed age = "
                   << joint_state_max_age_sec_ << " s.";

            error_message = stream.str();
            return false;
        }

        if (msg->name.size() != msg->position.size())
        {
            error_message =
                "JointState name and position arrays have different sizes.";
            return false;
        }

        // Build a name-to-position lookup from the received message.
        std::unordered_map<std::string, double> positions_by_name;

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (!std::isfinite(msg->position[i]))
            {
                error_message =
                    "JointState contains a non-finite position for joint '" +
                    msg->name[i] + "'.";
                return false;
            }

            positions_by_name[msg->name[i]] = msg->position[i];
        }

        // This is the authoritative order CCA uses for right_arm.
        const auto &cca_joint_names =
            cca_->get_joint_names("right_arm");

        if (cca_joint_names.empty())
        {
            error_message =
                "CCA returned an empty joint list for right_arm.";
            return false;
        }

        start_state.resize(
            static_cast<Eigen::Index>(cca_joint_names.size())
        );

        // Reorder the received positions to match CCA's robot model.
        for (size_t i = 0; i < cca_joint_names.size(); ++i)
        {
            const auto it =
                positions_by_name.find(cca_joint_names[i]);

            if (it == positions_by_name.end())
            {
                error_message =
                    "JointState is missing required CCA joint '" +
                    cca_joint_names[i] + "'.";
                return false;
            }

            start_state[static_cast<Eigen::Index>(i)] =
                it->second;
        }

        if (!start_state.allFinite())
        {
            error_message =
                "The extracted CCA start state contains non-finite values.";
            return false;
        }

        return true;
    }


    // ----------------------------------------------------------
    // Log the exact start state passed to CCA
    // ----------------------------------------------------------

    void log_start_state(
        const Eigen::VectorXd &start_state)
    {
        const auto &joint_names =
            cca_->get_joint_names("right_arm");

        std::ostringstream stream;

        stream << "Using this start state for CCA right_arm:";

        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            stream << "\n  "
                   << joint_names[i]
                   << " = "
                   << std::fixed << std::setprecision(5)
                   << start_state[static_cast<Eigen::Index>(i)];
        }

        RCLCPP_INFO(
            this->get_logger(),
            "%s",
            stream.str().c_str()
        );
    }


    // ----------------------------------------------------------
    // Log the planner's trajectory classification
    // ----------------------------------------------------------

    bool is_partial_trajectory(
        const cca_ros::PlanningResponse &cca_response)
    {
        const auto description =
            cca_response.result.cca_result.trajectory_description;

        switch (description)
        {
            case cc_affordance_planner::TrajectoryDescription::FULL:

                RCLCPP_INFO(
                    this->get_logger(),
                    "CCA trajectory classification: FULL."
                );

                return false;


            case cc_affordance_planner::TrajectoryDescription::PARTIAL:

                RCLCPP_WARN(
                    this->get_logger(),
                    "CCA trajectory classification: PARTIAL. "
                    "Generated %zu joint-space waypoints.",
                    cca_response.result.cca_result.joint_trajectory.size()
                );

                return true;


            default:

                RCLCPP_WARN(
                    this->get_logger(),
                    "CCA trajectory classification: UNSET. "
                    "The planner may have failed before producing "
                    "a trajectory result."
                );

                return false;
        }
    }


    // ----------------------------------------------------------
    // Main service callback
    // ----------------------------------------------------------

    void plan_callback(
        const std::shared_ptr<PlanToPose::Request> request,
        std::shared_ptr<PlanToPose::Response> response)
    {
        // --------------------------------------------------
        // 1. Validate input frame
        // --------------------------------------------------

        const auto &pose_msg = request->target_pose;

        if (pose_msg.header.frame_id != "right_base_link")
        {
            response->success = false;

            response->message =
                "Target must be expressed in right_base_link. "
                "TF2 transformation is not implemented.";

            RCLCPP_ERROR(
                this->get_logger(),
                "%s",
                response->message.c_str()
            );

            return;
        }


        // --------------------------------------------------
        // 2. Extract and validate position and orientation
        // --------------------------------------------------

        const auto &pose = pose_msg.pose;

        Eigen::Vector3d position(
            pose.position.x,
            pose.position.y,
            pose.position.z
        );

        Eigen::Quaterniond orientation(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        );

        if (!position.allFinite() ||
            !orientation.coeffs().allFinite() ||
            orientation.norm() < 1e-6)
        {
            response->success = false;
            response->message = "Invalid target pose.";
            return;
        }

        orientation.normalize();


        // --------------------------------------------------
        // 3. Construct homogeneous transformation
        // --------------------------------------------------

        Eigen::Matrix4d target_pose =
            Eigen::Matrix4d::Identity();

        target_pose.block<3, 3>(0, 0) =
            orientation.toRotationMatrix();

        target_pose.block<3, 1>(0, 3) =
            position;

        RCLCPP_INFO(
            this->get_logger(),
            "Received Cartesian goal in right_base_link: "
            "x=%.3f, y=%.3f, z=%.3f",
            position.x(),
            position.y(),
            position.z()
        );


        // --------------------------------------------------
        // 4. Get a fresh, correctly ordered robot start state
        // --------------------------------------------------

        Eigen::VectorXd start_state;

        std::string start_state_error;

        if (!get_current_start_state(
                start_state,
                start_state_error))
        {
            response->success = false;

            response->message =
                "Cannot plan: " + start_state_error;

            RCLCPP_ERROR(
                this->get_logger(),
                "%s",
                response->message.c_str()
            );

            return;
        }

        log_start_state(start_state);


        // --------------------------------------------------
        // 5. Construct CCA planning request
        // --------------------------------------------------

        cca_ros::PlanningRequest planning_request;

        planning_request.planning_group = "right_arm";

        planning_request.task_description =
            cc_affordance_planner::TaskDescription(
                cc_affordance_planner::PlanningType::CARTESIAN_GOAL
            );

        planning_request.task_description.goal.canonical_pose =
            target_pose;

        planning_request.task_description.trajectory_density = 20;

        planning_request.planner_config.ik_max_itr = 2000;

        // IMPORTANT:
        // Pass the exact joint-state snapshot logged above.
        // CCA will use this vector rather than independently
        // reading another robot joint-state message.
        planning_request.start_state.robot = start_state;

        // This Cartesian goal does not request a gripper trajectory.
        // Preserve the default "no gripper goal" sentinel.
        planning_request.start_state.gripper =
            std::numeric_limits<double>::quiet_NaN();

        planning_request.execute_trajectory =
            request->execute_trajectory;

        // Do not set execute_partial_trajectory here.
        // Leave CCA's default behavior unchanged.
        planning_request.visualize_trajectory = true;

        planning_request.execution_timeout =
            std::chrono::seconds(60);


        // --------------------------------------------------
        // 6. Call CCA
        // --------------------------------------------------

        RCLCPP_INFO(
            this->get_logger(),
            "Calling CCA Cartesian planner..."
        );

        cca_ros::PlanningResponse cca_response;

        try
        {
            cca_response = cca_->plan(planning_request);
        }
        catch (const std::exception &e)
        {
            response->success = false;

            response->message =
                std::string("CCA exception: ") + e.what();

            RCLCPP_ERROR(
                this->get_logger(),
                "%s",
                response->message.c_str()
            );

            return;
        }


        // --------------------------------------------------
        // 7. Report FULL / PARTIAL / UNSET
        // --------------------------------------------------

        const bool partial =
            is_partial_trajectory(cca_response);


        // --------------------------------------------------
        // 8. Check planning and validation result
        // --------------------------------------------------

        if (!cca_response.result.success)
        {
            response->success = false;

            response->message = partial
                ? "CCA generated a PARTIAL trajectory, but the "
                  "overall planning/validation operation failed."
                : "CCA failed to generate or validate a trajectory.";

            RCLCPP_ERROR(
                this->get_logger(),
                "%s",
                response->message.c_str()
            );

            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "CCA successfully generated and validated a %s trajectory.",
            partial ? "PARTIAL" : "FULL"
        );


        // --------------------------------------------------
        // 9. Optionally wait for execution
        // --------------------------------------------------

        if (request->execute_trajectory)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Waiting for trajectory execution..."
            );

            const auto start_time =
                std::chrono::steady_clock::now();

            while (rclcpp::ok())
            {
                const auto status = *cca_response.status;

                if (status == cca_ros::Status::SUCCEEDED)
                {
                    break;
                }

                if (status == cca_ros::Status::FAILED)
                {
                    response->success = false;

                    response->message =
                        "CCA trajectory execution failed.";

                    RCLCPP_ERROR(
                        this->get_logger(),
                        "%s",
                        response->message.c_str()
                    );

                    return;
                }

                const auto elapsed =
                    std::chrono::steady_clock::now() - start_time;

                if (elapsed > std::chrono::seconds(65))
                {
                    response->success = false;

                    response->message =
                        "Timed out waiting for trajectory execution.";

                    RCLCPP_ERROR(
                        this->get_logger(),
                        "%s",
                        response->message.c_str()
                    );

                    return;
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(50)
                );
            }

            if (!rclcpp::ok())
            {
                response->success = false;

                response->message =
                    "ROS shutdown during execution.";

                return;
            }
        }


        // --------------------------------------------------
        // 10. Return result
        // --------------------------------------------------

        response->success = true;

        if (request->execute_trajectory)
        {
            response->message = partial
                ? "CCA PARTIAL trajectory successfully executed."
                : "CCA FULL trajectory successfully executed.";
        }
        else
        {
            response->message = partial
                ? "CCA PARTIAL trajectory successfully planned and validated."
                : "CCA FULL trajectory successfully planned and validated.";
        }

        RCLCPP_INFO(
            this->get_logger(),
            "%s",
            response->message.c_str()
        );
    }
};


// ==========================================================
// Main
// ==========================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // CCA node: receives joint states, TF, and planning parameters.
    rclcpp::NodeOptions options;

    auto planner_node =
        std::make_shared<rclcpp::Node>(
            "cc_affordance_planner_ros",
            options
        );

    auto cca =
        std::make_shared<cca_ros::CcaRos>(planner_node);

    // Spin CCA independently so its subscriptions, TF listener,
    // and action clients continue processing during service calls.
    std::thread planner_thread(
        [planner_node]()
        {
            rclcpp::spin(planner_node);
        }
    );

    // Separate node exposes the custom service and joint-state monitor.
    auto service_node =
        std::make_shared<CcaPoseService>(cca);

    RCLCPP_INFO(
        service_node->get_logger(),
        "CCA Cartesian planning service is running."
    );

    rclcpp::spin(service_node);

    rclcpp::shutdown();

    if (planner_thread.joinable())
    {
        planner_thread.join();
    }

    return 0;
}