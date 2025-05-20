#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <dirent.h>
#include <vector>
#include <string>
#include <set>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robot_interfaces/srv/param_server.hpp"
#include <rclcpp/parameter_client.hpp>


/***
 *                
 * service /service_param 
 *  
 */
using namespace std;
class ParamMgmtServer : public rclcpp::Node {
public:
    ParamMgmtServer() : Node("param_mgmt_server") {

        service_ps_ = this->create_service<robot_interfaces::srv::ParamServer>(
            "/service_param",
            std::bind(&ParamMgmtServer::handle_request_ps, this, std::placeholders::_1, std::placeholders::_2));
        

        RCLCPP_INFO(this->get_logger(), "Service /service_param is ready.");
    }

private:

    void handle_request_ps(
        const std::shared_ptr<robot_interfaces::srv::ParamServer::Request> request,
        std::shared_ptr<robot_interfaces::srv::ParamServer::Response> response) {
            
          uint32_t cmd_type = request->cmd_type;
          if(cmd_type == 1){//get params

            switch (request->cmd_name) {
              case 1:  // Version
                  get_chassis_sw_version(response);    
                  break;
              case 2:  // Speed
                  get_max_speed_param(response);
                  break;
              case 3:  // Charge pose
                  try {
                      ;
                  } catch (const std::exception& e) {
                      response->err_code = 500;
                      response->err_msg = std::string("Error in Get Charge Pose param: ") + e.what();
                      RCLCPP_ERROR(get_logger(), "Get Charge Pose param error: %s", e.what());
                      break;
                  }
                  response->err_code = 0x00;
                  response->err_msg = "Get Charge Pose param successfully";
                  break;
              case 4:  // Charge threshold
                  try{
                      ;
                  }catch (const std::exception& e) {
                      response->err_code = 500;
                      response->err_msg = std::string("Error in Get Charge Threshold param: ") + e.what();
                      RCLCPP_ERROR(get_logger(), "Get Charge Threshold param error: %s", e.what());
                      break;
                  } catch (...) {
                      response->err_code = 500;
                      response->err_msg = "Unknown error in Get Charge Threshold param";
                      RCLCPP_ERROR(get_logger(), "Unknown error in Get Charge Threshold param");
                      break;
                  }
                  response->err_code = 0x00;
                  response->err_msg = "Get Charge Threshold param successfully";
                  break;
              case 5:  // Battery level
                  get_battery_level(response);  
                  break;
              default:
                  response->err_code = 500;
                  response->err_msg = "Invalid cmd_name";
                  break;
            }

          }else if(cmd_type == 2){//set params and dump params
            switch (request->cmd_name) {
                case 2:  // Set Max Speed
                    set_max_speed_param(request, response);
                    break;
                case 3:  // Set Charge pose
                    break;
                case 4:  // Set Charge threshold
                    break;
                default:

                    break;

            }
        }

    }

    void get_battery_level(std::shared_ptr<robot_interfaces::srv::ParamServer::Response> response){
        RCLCPP_INFO(this->get_logger(), "start to get_battery_level");
        auto node = rclcpp::Node::make_shared("temp_get_param_node");
        
        // Declare parameter client
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/hm_base_driver_node");
        
        // Wait for service to be available
        if (!param_client->wait_for_service(std::chrono::seconds(1))) {
            response->err_code = 500;
            response->err_msg = "Parameter service not available";
            return;
        }

        try {
            // Get the parameter value
            auto b_level = param_client->get_parameter<float>("battery_level");
            response->battery_level = b_level;
            response->err_code = 0x00;
            response->err_msg = "Get battery level param successfully";
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
            response->err_code = 501;
            response->err_msg = "Parameter battery_level not declared";
        } catch (const std::exception & e) {
            response->err_code = 502;
            response->err_msg = std::string("Error getting battery level param: ") + e.what();
        }
    }

    void get_chassis_sw_version(std::shared_ptr<robot_interfaces::srv::ParamServer::Response> response){
        RCLCPP_INFO(this->get_logger(), "start to get_chassis_sw_version");
        auto node = rclcpp::Node::make_shared("temp_get_param_node");
        
        // Declare parameter client
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/hm_base_driver_node");
        
        // Wait for service to be available
        if (!param_client->wait_for_service(std::chrono::seconds(1))) {
            response->err_code = 500;
            response->err_msg = "Parameter service not available";
            return;
        }

        try {
            // Get the parameter value
            auto v_mcu = param_client->get_parameter<string>("version_mcu");
            auto v_main_control_sw = param_client->get_parameter<string>("version_main_control_sw");
            response->version_list.clear();
            response->version_list.push_back(v_mcu);
            response->version_list.push_back(v_main_control_sw);
            response->err_code = 0x00;
            response->err_msg = "Get sw version param successfully";
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
            response->err_code = 501;
            response->err_msg = "Parameter version_mcu or version_main_control_sw not declared";
        } catch (const std::exception & e) {
            response->err_code = 502;
            response->err_msg = std::string("Error getting sw version param: ") + e.what();
        }

    }

    void get_max_speed_param(std::shared_ptr<robot_interfaces::srv::ParamServer::Response> response) {
        RCLCPP_INFO(this->get_logger(), "start to get_max_speed_param");

        auto node = rclcpp::Node::make_shared("temp_get_param_node");
        
        // Declare parameter client
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/hm_base_driver_node");
        
        // Wait for service to be available
        if (!param_client->wait_for_service(std::chrono::seconds(1))) {
            response->err_code = 500;
            response->err_msg = "Parameter service not available";
            return;
        }

        try {
            // Get the parameter value
            auto max_linear = param_client->get_parameter<float>("max_linear_speed");
            auto max_angular = param_client->get_parameter<float>("max_angular_speed");
            response->speed_list.clear();
            response->speed_list.push_back(max_linear);
            response->speed_list.push_back(max_angular);
            response->err_code = 0x00;
            response->err_msg = "Get max speed param successfully";
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
            response->err_code = 501;
            response->err_msg = "Parameter max_linear_speed or max_angular_speed not declared";
        } catch (const std::exception & e) {
            response->err_code = 502;
            response->err_msg = std::string("Error getting max speed param: ") + e.what();
        }
    }

    void set_max_speed_param(const std::shared_ptr<robot_interfaces::srv::ParamServer::Request> request,
        std::shared_ptr<robot_interfaces::srv::ParamServer::Response> response) {
        auto node = rclcpp::Node::make_shared("temp_set_param_node");

        //Set parameters
        if (request->speed_list.empty()) {
            response->err_code = 500;
            response->err_msg = "Request speed_list is empty";
            return;
        }
        float max_linear_speed = request->speed_list[0];
        float max_angular_speed = request->speed_list[1];
        // Create a parameter client for the target node
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
            node,
            "hm_base_driver_node");
        if (!param_client->wait_for_service(std::chrono::seconds(1))) {
            response->err_code = 501;
            response->err_msg = "node /hm_base_driver_node Parameter service not available";
            return;
        }
        // Set the parameter
        auto result = param_client->set_parameters({
            rclcpp::Parameter("max_linear_speed", max_linear_speed),
            rclcpp::Parameter("max_angular_speed", max_angular_speed)
        });
        if (!result[0].successful) {
            response->err_code = 502;
            response->err_msg = "Failed to set parameter";
            return;
        }

        //Dump parameters
        std::string package_name = "base_driver";
        std::string config_dir = ament_index_cpp::get_package_share_directory(package_name) + "/config";
        std::filesystem::create_directories(config_dir);
        std::string output_path = config_dir + "/base.yaml";
        RCLCPP_INFO(this->get_logger(), "start to call dump_param()");
        dump_param("/hm_base_driver_node", output_path, response);
    
        // response->err_code = 0x00;
        // response->err_msg = "Set /hm_base_driver_node parameters and Dump successfully";
    }

    void dump_param(const std::string& node_name, const std::string& output_file,
        std::shared_ptr<robot_interfaces::srv::ParamServer::Response> response) {
        // Add nullptr check for 'this'
        if (!this) {
            RCLCPP_ERROR(rclcpp::get_logger("param_mgmt_server"), "Invalid object state!");
            response->err_code = 503;
            response->err_msg = "Internal server error";
            return;
        }    

        // auto node_list = rclcpp::Node::make_shared("temp_dump_param_list_node");
        // auto node_get = rclcpp::Node::make_shared("temp_dump_param_get_node");
        auto node_dump_param = std::make_shared<rclcpp::Node>("temp_dump_param_node");
        // auto node_get = std::make_shared<rclcpp::Node>("temp_dump_param_get_node");

        // auto node_temp = rclcpp::Node::make_shared("temp_get_node_names");
        std::vector<std::string> node_names;
        try {
            node_names = this->get_node_names();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get node names: %s", e.what());
            response->err_code = 504;
            response->err_msg = "Failed to query node names";
            return;
        }
        if (std::find(node_names.begin(), node_names.end(), node_name) == node_names.end()) {
            RCLCPP_ERROR(this->get_logger(), "Node '%s' not found!", node_name.c_str());
            response->err_code = 505;
            response->err_msg = std::string("Node ")+ node_name + std::string("not found!");
            return;
        }

        // Create executors
        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        auto list_client = node_dump_param->create_client<rcl_interfaces::srv::ListParameters>(node_name + "/list_parameters");
        auto get_client = node_dump_param->create_client<rcl_interfaces::srv::GetParameters>(node_name + "/get_parameters");

        // List parameters
        if (!list_client->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "Dump parameters, list_parameters service unavailable");
            response->err_code = 506;
            response->err_msg = "Dump parameters, list_parameters service unavailable";
            return;
        }

        auto list_request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
        executor->add_node(node_dump_param);
        auto list_future = list_client->async_send_request(list_request);
        
        if (executor->spin_until_future_complete(list_future, 3s) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Dump parameters, Failed to list parameters");
            response->err_code = 507;
            response->err_msg = "Dump parameters, Failed to list parameters";
            executor->remove_node(node_dump_param);
            return;
        }

        auto param_names = list_future.get()->result.names;
        executor->remove_node(node_dump_param);
        if (param_names.empty()) {
            RCLCPP_WARN(this->get_logger(), "Dump parameters, No parameters found for %s", node_name.c_str());
            response->err_code = 508;
            response->err_msg = std::string("Dump parameters, No parameters found for ") + node_name;
            return;
        }

        // Get parameter values
        auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        get_request->names = param_names;
        executor->add_node(node_dump_param);
        auto get_future = get_client->async_send_request(get_request);

        if (executor->spin_until_future_complete(get_future, 3s) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            YAML::Node params;
            params[node_name]["ros__parameters"] = YAML::Node(YAML::NodeType::Map);

            auto param_values = get_future.get()->values;
            for (size_t i = 0; i < param_names.size(); ++i) {
                params[node_name]["ros__parameters"][param_names[i]] = parameter_to_yaml(param_values[i]);
            }

            std::ofstream fout(output_file);
            fout << params;
            RCLCPP_INFO(this->get_logger(), "Parameters dumped to %s", output_file.c_str());

            response->err_code = 0x00;
            response->err_msg = "Set /hm_base_driver_node parameters and Dump successfully";
            executor->remove_node(node_dump_param);
            return;
        }
        
        executor->remove_node(node_dump_param);
        response->err_code = 509;
        response->err_msg = "Dump parameters, Cannot get param_values";
        // node_param.reset();
        RCLCPP_ERROR(get_logger(), "Dump parameters error, Cannot get param_values");
       
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

    rclcpp::Service<robot_interfaces::srv::ParamServer>::SharedPtr service_ps_;
    std::string maps_dir;
    std::string params_dir;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParamMgmtServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
