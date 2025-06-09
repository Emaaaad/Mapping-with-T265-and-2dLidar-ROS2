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
        _ros_callback = _node.add_on_set_parameters_callback(callback);
    }

    ParametersBackend::~ParametersBackend()
    {
        if (_ros_callback)
        {
            _node.remove_on_set_parameters_callback((rclcpp::node_interfaces::OnSetParametersCallbackHandle*)(_ros_callback.get()));
            _ros_callback.reset();
        }
    }
}
