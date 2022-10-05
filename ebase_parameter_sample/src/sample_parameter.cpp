#include "ebase_parameter_sample/sample_parameter.hpp"

#include <string>
#include <vector>

using ebase::system::ParameterDeclaration;
using ebase::samples::SampleParameter;

std::vector<ParameterDeclaration<int64_t>> SampleParameter::GetIntDeclaration()
{
  std::vector<ParameterDeclaration<int64_t>> value;
  value.push_back(ParameterDeclaration<int64_t>{"TestInt", 10});

  value.push_back(
    ParameterDeclaration<int64_t>{"TestIntDes", 10, true, "int description", 0, 1, 255});
  return value;
}
std::vector<ParameterDeclaration<double>> SampleParameter::GetDoubleDeclaration()
{
  std::vector<ParameterDeclaration<double>> value;
  value.push_back(ParameterDeclaration<double>{"TestDouble", 2.2});

  value.push_back(
    ParameterDeclaration<double>{"TestDoubleDes", 0.5, true, "double description", 0.0, 0.1, 10.0});
  return value;
}
std::vector<ParameterDeclaration<std::string>> SampleParameter::GetStringDeclaration()
{
  std::vector<ParameterDeclaration<std::string>> value;
  value.push_back(ParameterDeclaration<std::string>{"TestString", "HelloWorld"});
  return value;
}
std::vector<ParameterDeclaration<bool>> SampleParameter::GetBoolDeclaration()
{
  std::vector<ParameterDeclaration<bool>> value;
  value.push_back(ParameterDeclaration<bool>{"TestBool", false});
  return value;
}

std::vector<ParameterDeclaration<std::vector<int64_t>>> SampleParameter::GetIntArrayDeclaration()
{
  std::vector<ParameterDeclaration<std::vector<int64_t>>> value;
  value.push_back(
    ParameterDeclaration<std::vector<int64_t>>{
    "TestIntArray", std::vector<int64_t>{1L, 10L, 22L, 55L}});
  return value;
}

std::vector<ParameterDeclaration<std::vector<std::string>>>
SampleParameter::GetStringArrayDeclaration()
{
  std::vector<ParameterDeclaration<std::vector<std::string>>> value;
  value.push_back(
    ParameterDeclaration<std::vector<std::string>>{
    "TestIntString", std::vector<std::string>{"str1", "str2", "str3", "str4"}});
  return value;
}
