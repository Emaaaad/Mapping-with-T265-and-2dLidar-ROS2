#include "realsense2_camera/base_realsense_node.h"
#include "realsense2_camera/t265_realsense_node.h"
#include "realsense2_camera/dynamic_params.h"
#include "realsense2_camera/realsense_node_factory.h"
#include "realsense2_camera/ros_param_backend.h"
#include "realsense2_camera/constants.h"


namespace realsense2_camera
{
    void ParametersBackend::add_on_set_parameters_callback(rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType callback)
    {
        rclcpp::Node::OnParametersSetCallbackType prev_callback = _node.set_on_parameters_set_callback(callback);
        if (prev_callback)
        {
            rclcpp::Node::OnParametersSetCallbackType prev_callback = _node.set_on_parameters_set_callback(prev_callback);
            std::stringstream msg;
            msg << "Cannot set another callback to current node: " << _node.get_name();
            throw std::runtime_error(msg.str());
        }
    }

    ParametersBackend::~ParametersBackend()
    {
    }
}
