#ifndef EBASE_PARAMETER_SAMPLE__SAMPLE_PARAMETER_HPP_
#define EBASE_PARAMETER_SAMPLE__SAMPLE_PARAMETER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ebase_parameter/ebase_parameter.hpp"

namespace ebase
{
namespace samples
{
using ebase::system::EbaseParameter;
using ebase::system::ParameterDeclaration;
/*!
 * @class SampleParameter
 * @brief EbaseParameter를 상속 받는 샘플 클래스
 * @details EbaseParameter를 상속받아 필요한 가상함수를 재정의함.
 *  각 가상함수는 타입별 parameter의 정의를 리턴함.
 *  사용하지 않는 타입은 구현하지 않아도 됨.
 */
class SampleParameter : public EbaseParameter
{
protected:
  virtual std::vector<ParameterDeclaration<int64_t>> GetIntDeclaration();
  virtual std::vector<ParameterDeclaration<double>> GetDoubleDeclaration();
  virtual std::vector<ParameterDeclaration<std::string>> GetStringDeclaration();
  virtual std::vector<ParameterDeclaration<bool>> GetBoolDeclaration();

  virtual std::vector<ParameterDeclaration<std::vector<int64_t>>> GetIntArrayDeclaration();
  virtual std::vector<ParameterDeclaration<std::vector<std::string>>> GetStringArrayDeclaration();
};
}  // namespace samples
}  // namespace ebase
#endif  // EBASE_PARAMETER_SAMPLE__SAMPLE_PARAMETER_HPP_
