#ifndef REALSENSE2_CAMERA_ROS_PARAM_BACKEND_H
#define REALSENSE2_CAMERA_ROS_PARAM_BACKEND_H

#include <rclcpp/rclcpp.hpp>

namespace realsense2_camera
{
    class ParametersBackend
    {
        public:
            ParametersBackend(rclcpp::Node& node) : 
                _node(node),
                _logger(rclcpp::get_logger("RealSenseCameraNode"))
                {};
            ~ParametersBackend();
            void add_on_set_parameters_callback(rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType callback);


        private:
            rclcpp::Node& _node;
            rclcpp::Logger _logger;
            std::shared_ptr<void> _ros_callback;
    };
}

#endif
