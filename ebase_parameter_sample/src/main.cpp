#include "ebase_parameter/ebase_parameter_factory.hpp"
#include "ebase_parameter_sample/sample_interface.hpp"
#include "ebase_parameter_sample/sample_parameter.hpp"
#include <rclcpp/node.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
namespace iterf = ebase::samples::functions;
/*!
 * @class TestNode
 * @brief parameter interface sample을 보여주기 위한 node
 * @details
 */
class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node("TestNode")
  {
    timer_ = this->create_wall_timer(
      5s,
      [&]()
      {
        RCLCPP_INFO(this->get_logger(), "TestInt : %d", iterf::GetIntParam());
        RCLCPP_INFO(this->get_logger(), "TestDouble : %lf", iterf::GetDoubleParam());
        RCLCPP_INFO(this->get_logger(), "TestString : %s", iterf::GetStringParam().c_str());
        RCLCPP_INFO(this->get_logger(), "TestBool : %d", iterf::GetBoolParam());
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

using ebase::samples::SampleParameter;
using ebase::system::EbaseParameterFactory;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestNode>();

  // parameter 서비스 초기화
  EbaseParameterFactory<SampleParameter>::get()->ParameterServiceInit(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
