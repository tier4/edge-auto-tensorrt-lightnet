#include "tensorrt_lightnet_node.hpp"

namespace tensorrt_lightnet
{
    TrtLightNetNode::TrtLightNetNode(const rclcpp::NodeOptions &node_options)
        : Node("tensorrt_lightnet", node_options)
    {
        using std::placeholders::_1;
        using std::chrono_literals::operator""ms;

        auto declare_parameter_with_description =
            [this](std::string name, auto default_val, std::string description = "")
        {
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = description;
            return this->declare_parameter(name, default_val, param_desc);
        };

        std::string model_cfg = declare_parameter_with_description("model_cfg", "", "The path for .cfg file");
        std::string model_weights = declare_parameter_with_description("model_weights", "", "The path for .weights file");

        trt_lightnet_ = std::make_unique<tensorrt_lightnet::TrtLightNet>(model_cfg, model_weights);

        timer_ = rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&TrtLightNetNode::onConnect, this));

        image_pub_ = image_transport::create_publisher(this, "~/out/image");
    }

    void TrtLightNetNode::onConnect()
    {
        using std::placeholders::_1;
        image_sub_ = image_transport::create_subscription(
            this, "~/in/image", std::bind(&TrtLightNetNode::onImage, this, _1), "raw",
            rmw_qos_profile_sensor_data);
    }

    void TrtLightNetNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        cv_bridge::CvImagePtr in_image_ptr;
        in_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // {in_image_ptr->image} が std::vector<cv::Mat> 型になっているので注意
        if (!trt_lightnet_->doInference({in_image_ptr->image}))
        {
            RCLCPP_WARN(this->get_logger(), "Inference failed.");
            return;
        }
        image_pub_.publish(in_image_ptr.toImageMsg());
    }

} // namespace tensorrt_lightnet

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tensorrt_lightnet::TrtLightNetNode)