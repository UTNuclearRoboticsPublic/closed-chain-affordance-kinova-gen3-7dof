#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <functional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

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
        cca_(cca)
    {
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
    }


private:

    std::shared_ptr<cca_ros::CcaRos> cca_;

    rclcpp::Service<
        cca_kinova_gen3_7dof::srv::PlanToPose
    >::SharedPtr service_;


    void plan_callback(
        const std::shared_ptr<
            cca_kinova_gen3_7dof::srv::PlanToPose::Request
        > request,

        std::shared_ptr<
            cca_kinova_gen3_7dof::srv::PlanToPose::Response
        > response)
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
                "TODO: Implement TF2 transformation.";

            RCLCPP_ERROR(
                this->get_logger(),
                "%s",
                response->message.c_str()
            );

            return;
        }


        // --------------------------------------------------
        // 2. Extract position and orientation
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
            "Received Cartesian goal: "
            "x=%.3f, y=%.3f, z=%.3f",
            position.x(),
            position.y(),
            position.z()
        );


        // --------------------------------------------------
        // 4. Construct CCA planning request
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

        planning_request.execute_trajectory =
            request->execute_trajectory;

        planning_request.visualize_trajectory = true;

        planning_request.execution_timeout =
            std::chrono::seconds(60);


        // --------------------------------------------------
        // 5. Call CCA
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
        // 6. Check planning result
        // --------------------------------------------------

        if (!cca_response.result.success)
        {
            response->success = false;
            response->message = "CCA failed to generate trajectory.";

            RCLCPP_ERROR(
                this->get_logger(),
                "CCA planning failed."
            );

            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "CCA successfully generated trajectory."
        );


        // --------------------------------------------------
        // 7. Optionally wait for execution
        // --------------------------------------------------

        if (request->execute_trajectory)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Waiting for trajectory execution..."
            );

            auto start_time =
                std::chrono::steady_clock::now();

            while (rclcpp::ok())
            {
                auto status = *cca_response.status;

                if (status == cca_ros::Status::SUCCEEDED)
                {
                    break;
                }

                if (status == cca_ros::Status::FAILED)
                {
                    response->success = false;
                    response->message =
                        "CCA trajectory execution failed.";

                    return;
                }

                auto elapsed =
                    std::chrono::steady_clock::now() - start_time;

                if (elapsed > std::chrono::seconds(65))
                {
                    response->success = false;
                    response->message =
                        "Timed out waiting for trajectory execution.";

                    return;
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(50)
                );
            }

            if (!rclcpp::ok())
            {
                response->success = false;
                response->message = "ROS shutdown during execution.";
                return;
            }
        }


        // --------------------------------------------------
        // 8. Return result
        // --------------------------------------------------

        response->success = true;

        response->message =
            request->execute_trajectory
            ? "CCA trajectory successfully executed."
            : "CCA trajectory successfully planned.";

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


    // Spin CCA independently so its subscriptions continue
    // processing while our service callback calls cca_->plan().
    std::thread planner_thread(
        [planner_node]()
        {
            rclcpp::spin(planner_node);
        }
    );


    // Separate node exposes our custom service.
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