#ifndef EBASE_PARAMETER__EBASE_PARAMETER_HPP_
#define EBASE_PARAMETER__EBASE_PARAMETER_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/parameter_client.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>
namespace ebase
{
namespace system
{
template<typename T>
struct ParameterDeclaration
{
  std::string name;
  T default_value;
  bool have_description = false;
  std::string description = std::string{};
  T from_value = T{};
  T step = T{};
  T to_value = T{};
};

class EbaseParameter
{
public:
  /**
   * @brief Construct a new Node Parameter object
   */
  EbaseParameter();

  int64_t GetInt(std::string name);
  double GetDouble(std::string name);
  std::string GetString(std::string name);
  bool GetBool(std::string name);

  std::vector<uint8_t> GetByteArray(std::string name);
  std::vector<int64_t> GetIntArray(std::string name);
  std::vector<double> GetDoubleArray(std::string name);
  std::vector<std::string> GetStringArray(std::string name);
  std::vector<bool> GetBoolArray(std::string name);

  void ParameterServiceInit(rclcpp::Node::SharedPtr node);

protected:
  virtual std::vector<ParameterDeclaration<int64_t>> GetIntDeclaration();
  virtual std::vector<ParameterDeclaration<double>> GetDoubleDeclaration();
  virtual std::vector<ParameterDeclaration<std::string>> GetStringDeclaration();
  virtual std::vector<ParameterDeclaration<bool>> GetBoolDeclaration();

  virtual std::vector<ParameterDeclaration<std::vector<uint8_t>>> GetByteArrayDeclaration();
  virtual std::vector<ParameterDeclaration<std::vector<bool>>> GetBoolArrayDeclaration();
  virtual std::vector<ParameterDeclaration<std::vector<int64_t>>> GetIntArrayDeclaration();
  virtual std::vector<ParameterDeclaration<std::vector<double>>> GetDoubleArrayDeclaration();
  virtual std::vector<ParameterDeclaration<std::vector<std::string>>> GetStringArrayDeclaration();

private:
  template<typename T>
  void DeclareParameter(T parameList, std::vector<std::string> & paramNames);

  void SetParamValue(const std::vector<rclcpp::Parameter> & param);
  void SetParamValue(const rclcpp::Parameter & param);

  rclcpp::Node::SharedPtr node_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  std::map<std::string, int64_t> int_type_;
  std::map<std::string, double> float_type_;
  std::map<std::string, bool> bool_type_;
  std::map<std::string, std::string> string_type_;

  std::map<std::string, std::vector<uint8_t>> array_byte_type_;
  std::map<std::string, std::vector<bool>> array_bool_type_;
  std::map<std::string, std::vector<int64_t>> array_int_type_;
  std::map<std::string, std::vector<double>> array_double_type_;
  std::map<std::string, std::vector<std::string>> array_string_type_;

  std::mutex parm_m_;
};

template<>
inline void EbaseParameter::DeclareParameter<std::vector<ParameterDeclaration<int64_t>>>(
  std::vector<ParameterDeclaration<int64_t>> parameList, std::vector<std::string> & paramNames)
{
  for (auto o : parameList) {
    if (o.have_description) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      rcl_interfaces::msg::IntegerRange range;
      range.from_value = o.from_value;
      range.step = o.step;
      range.to_value = o.to_value;
      descriptor.integer_range.push_back(range);
      descriptor.description = o.description;
      node_->declare_parameter<decltype(o.default_value)>(o.name, o.default_value, descriptor);
    } else {
      node_->declare_parameter<decltype(o.default_value)>(o.name, o.default_value);
    }
    paramNames.push_back(o.name);
  }
}
template<>
inline void EbaseParameter::DeclareParameter<std::vector<ParameterDeclaration<double>>>(
  std::vector<ParameterDeclaration<double>> parameList, std::vector<std::string> & paramNames)
{
  for (auto o : parameList) {
    if (o.have_description) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      rcl_interfaces::msg::FloatingPointRange range;
      range.from_value = o.from_value;
      range.step = o.step;
      range.to_value = o.to_value;
      descriptor.description = o.description;
      descriptor.floating_point_range.push_back(range);
      node_->declare_parameter<decltype(o.default_value)>(o.name, o.default_value, descriptor);
    } else {
      node_->declare_parameter<decltype(o.default_value)>(o.name, o.default_value);
    }
    paramNames.push_back(o.name);
  }
}

}  // namespace system
}  // namespace ebase

#endif  // EBASE_PARAMETER__EBASE_PARAMETER_HPP_
