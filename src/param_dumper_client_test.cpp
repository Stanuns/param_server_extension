#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class ParamDumperclient : public rclcpp::Node {
public:
    ParamDumperclient() : Node("param_dumper_client_test") {
        std::string package_name = "base_driver";
        std::string config_dir = ament_index_cpp::get_package_share_directory(package_name) + "/config";
        std::filesystem::create_directories(config_dir);
        std::string output_path = config_dir + "/base.yaml";
        dump_parameters("hm_base_driver_node", output_path);
    }

private:
    void dump_parameters(const std::string& node_name, const std::string& output_file) {
        if (std::find(this->get_node_names().begin(), this->get_node_names().end(), node_name) == this->get_node_names().end()) {
            RCLCPP_ERROR(this->get_logger(), "Node '%s' not found!", node_name.c_str());
            return;
        }

        auto list_client = this->create_client<rcl_interfaces::srv::ListParameters>("/" + node_name + "/list_parameters");
        auto get_client = this->create_client<rcl_interfaces::srv::GetParameters>("/" + node_name + "/get_parameters");

        // List parameters
        if (!list_client->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "list_parameters service unavailable");
            return;
        }

        auto list_request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
        auto list_future = list_client->async_send_request(list_request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), list_future) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to list parameters");
            return;
        }

        auto param_names = list_future.get()->result.names;
        if (param_names.empty()) {
            RCLCPP_WARN(this->get_logger(), "No parameters found for %s", node_name.c_str());
            return;
        }

        // Get parameter values
        auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        get_request->names = param_names;
        auto get_future = get_client->async_send_request(get_request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), get_future) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            YAML::Node params;
            params[node_name]["ros__parameters"] = YAML::Node(YAML::NodeType::Map);

            for (size_t i = 0; i < param_names.size(); ++i) {
                params[node_name]["ros__parameters"][param_names[i]] = parameter_to_yaml(get_future.get()->values[i]);
            }

            std::ofstream f(output_file);
            f << params;
            RCLCPP_INFO(this->get_logger(), "Parameters dumped to %s", output_file.c_str());
        }
    }

    YAML::Node parameter_to_yaml(const rcl_interfaces::msg::ParameterValue& param_value) {
        switch (param_value.type) {
            case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
                return YAML::Node(param_value.bool_value);
            case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
                return YAML::Node(param_value.integer_value);
            case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
                return YAML::Node(param_value.double_value);
            case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
                return YAML::Node(param_value.string_value);
            case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
                return YAML::Node(param_value.byte_array_value);
            case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
                return YAML::Node(param_value.bool_array_value);
            case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
                return YAML::Node(param_value.integer_array_value);
            case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
                return YAML::Node(param_value.double_array_value);
            case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
                return YAML::Node(param_value.string_array_value);
            default:
                return YAML::Node();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto dumper = std::make_shared<ParamDumperclient>();
    rclcpp::shutdown();
    return 0;
}