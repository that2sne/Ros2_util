#include "ebase_parameter/ebase_parameter.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <typeinfo>
#include <vector>

using ebase::system::EbaseParameter;
using ebase::system::ParameterDeclaration;

using namespace std::chrono_literals;
EbaseParameter::EbaseParameter()
{
}

/**
 * @brief
 * @details
 * @param node
 */

template<typename T>
void EbaseParameter::DeclareParameter(T parameList, std::vector<std::string> & paramNames)
{
  for (auto o : parameList) {
    node_->declare_parameter<decltype(o.default_value)>(o.name, o.default_value);
    paramNames.push_back(o.name);
  }
}

void EbaseParameter::ParameterServiceInit(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  std::vector<std::string> parameters;

  DeclareParameter(GetIntDeclaration(), parameters);
  DeclareParameter(GetDoubleDeclaration(), parameters);
  DeclareParameter(GetStringDeclaration(), parameters);
  DeclareParameter(GetBoolDeclaration(), parameters);

  DeclareParameter(GetByteArrayDeclaration(), parameters);
  DeclareParameter(GetBoolArrayDeclaration(), parameters);
  DeclareParameter(GetIntArrayDeclaration(), parameters);
  DeclareParameter(GetDoubleArrayDeclaration(), parameters);
  DeclareParameter(GetStringArrayDeclaration(), parameters);

  auto param_values = node_->get_parameters(parameters);
  SetParamValue(param_values);

  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_);
  while (!parameters_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }

  auto param_event_callback =
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void {
      if (event->node.compare(this->node_->get_fully_qualified_name()) != 0) {
        return;
      }

      for (auto & changed_parameter_msg : event->changed_parameters) {
        auto changed_parameter = rclcpp::Parameter::from_parameter_msg(changed_parameter_msg);
        RCLCPP_INFO(
          node_->get_logger(), "%s %s type:%d", changed_parameter.get_name().c_str(),
          changed_parameter.value_to_string().c_str(), changed_parameter.get_type());
        SetParamValue(changed_parameter);
      }
    };
  parameter_event_sub_ = parameters_client_->on_parameter_event(param_event_callback);
}

void EbaseParameter::SetParamValue(const std::vector<rclcpp::Parameter> & param)
{
  for (const auto & p : param) {
    std::lock_guard<std::mutex> lg(parm_m_);
    SetParamValue(p);
  }
}
void EbaseParameter::SetParamValue(const rclcpp::Parameter & param)
{
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      float_type_[param.get_name()] = param.as_double();
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      int_type_[param.get_name()] = param.as_int();
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL:
      bool_type_[param.get_name()] = param.as_bool();
      break;

    case rclcpp::ParameterType::PARAMETER_STRING:
      string_type_[param.get_name()] = param.as_string();
      break;

    // array
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      array_byte_type_[param.get_name()] = param.as_byte_array();
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      array_bool_type_[param.get_name()] = param.as_bool_array();
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      array_int_type_[param.get_name()] = param.as_integer_array();
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      array_double_type_[param.get_name()] = param.as_double_array();
      break;
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      array_string_type_[param.get_name()] = param.as_string_array();
      break;
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
    default:
      break;
  }
}
int64_t EbaseParameter::GetInt(std::string name)
{
  std::lock_guard<std::mutex> lg(parm_m_);
  auto value = int_type_[name];
  return value;
}
double EbaseParameter::GetDouble(std::string name)
{
  std::lock_guard<std::mutex> lg(parm_m_);
  auto value = float_type_[name];
  return value;
}
std::string EbaseParameter::GetString(std::string name)
{
  std::lock_guard<std::mutex> lg(parm_m_);
  auto value = string_type_[name];
  return value;
}
bool EbaseParameter::GetBool(std::string name)
{
  std::lock_guard<std::mutex> lg(parm_m_);
  auto value = bool_type_[name];
  return value;
}

std::vector<uint8_t> EbaseParameter::GetByteArray(std::string name)
{
  std::lock_guard<std::mutex> lg(parm_m_);
  auto value = array_byte_type_[name];
  return value;
}
std::vector<int64_t> EbaseParameter::GetIntArray(std::string name)
{
  std::lock_guard<std::mutex> lg(parm_m_);
  auto value = array_int_type_[name];
  return value;
}
std::vector<double> EbaseParameter::GetDoubleArray(std::string name)
{
  std::lock_guard<std::mutex> lg(parm_m_);
  auto value = array_double_type_[name];
  return value;
}
std::vector<std::string> EbaseParameter::GetStringArray(std::string name)
{
  std::lock_guard<std::mutex> lg(parm_m_);
  auto value = array_string_type_[name];
  return value;
}
std::vector<bool> EbaseParameter::GetBoolArray(std::string name)
{
  std::lock_guard<std::mutex> lg(parm_m_);
  auto value = array_bool_type_[name];
  return value;
}

std::vector<ParameterDeclaration<int64_t>> EbaseParameter::GetIntDeclaration()
{
  std::vector<ParameterDeclaration<int64_t>> vec;
  return vec;
}
std::vector<ParameterDeclaration<double>> EbaseParameter::GetDoubleDeclaration()
{
  std::vector<ParameterDeclaration<double>> vec;
  return vec;
}
std::vector<ParameterDeclaration<std::string>> EbaseParameter::GetStringDeclaration()
{
  std::vector<ParameterDeclaration<std::string>> vec;
  return vec;
}
std::vector<ParameterDeclaration<bool>> EbaseParameter::GetBoolDeclaration()
{
  std::vector<ParameterDeclaration<bool>> vec;
  return vec;
}

std::vector<ParameterDeclaration<std::vector<uint8_t>>> EbaseParameter::GetByteArrayDeclaration()
{
  std::vector<ParameterDeclaration<std::vector<uint8_t>>> vec;
  return vec;
}
std::vector<ParameterDeclaration<std::vector<bool>>> EbaseParameter::GetBoolArrayDeclaration()
{
  std::vector<ParameterDeclaration<std::vector<bool>>> vec;
  return vec;
}
std::vector<ParameterDeclaration<std::vector<int64_t>>> EbaseParameter::GetIntArrayDeclaration()
{
  std::vector<ParameterDeclaration<std::vector<int64_t>>> vec;
  return vec;
}
std::vector<ParameterDeclaration<std::vector<double>>> EbaseParameter::GetDoubleArrayDeclaration()
{
  std::vector<ParameterDeclaration<std::vector<double>>> vec;
  return vec;
}
std::vector<ParameterDeclaration<std::vector<std::string>>>
EbaseParameter::GetStringArrayDeclaration()
{
  std::vector<ParameterDeclaration<std::vector<std::string>>> vec;
  return vec;
}
