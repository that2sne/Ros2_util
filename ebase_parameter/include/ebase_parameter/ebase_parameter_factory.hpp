#ifndef EBASE_PARAMETER__EBASE_PARAMETER_FACTORY_HPP_
#define EBASE_PARAMETER__EBASE_PARAMETER_FACTORY_HPP_

#include "ebase_parameter/ebase_parameter.hpp"

#include <memory>

namespace ebase
{
namespace system
{

template<class T>
class EbaseParameterFactory
{
public:
  static std::shared_ptr<EbaseParameter> get();
};

template<class T>
std::shared_ptr<EbaseParameter> EbaseParameterFactory<T>::get()
{
  static std::shared_ptr<EbaseParameter> obj;
  if (obj == nullptr) {
    obj = std::make_shared<T>();
  }
  return obj;
}

}  // namespace system
}  // namespace ebase
#endif  // EBASE_PARAMETER__EBASE_PARAMETER_FACTORY_HPP_
