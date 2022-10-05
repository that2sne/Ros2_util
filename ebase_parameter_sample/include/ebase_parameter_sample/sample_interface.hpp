#ifndef EBASE_PARAMETER_SAMPLE__SAMPLE_INTERFACE_HPP_
#define EBASE_PARAMETER_SAMPLE__SAMPLE_INTERFACE_HPP_

#include <string>

#include "ebase_parameter/ebase_parameter_factory.hpp"
#include "ebase_parameter_sample/sample_parameter.hpp"

namespace ebase
{
namespace samples
{
using ebase::system::EbaseParameterFactory;
using ebase::samples::SampleParameter;
using PH = EbaseParameterFactory<SampleParameter>;
inline namespace functions
{
inline int GetIntParam()
{
  return PH::get()->GetInt("TestInt");
}

inline double GetDoubleParam()
{
  return PH::get()->GetDouble("TestDouble");
}

inline std::string GetStringParam()
{
  return PH::get()->GetString("TestString");
}

inline bool GetBoolParam()
{
  return PH::get()->GetBool("TestBool");
}
}
}  // namespace samples
}  // namespace ebase
#endif  // EBASE_PARAMETER_SAMPLE__SAMPLE_INTERFACE_HPP_
