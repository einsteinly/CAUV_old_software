#include "../../nodeFactory.h"

#include "addMultNode.h"
#include "divideNode.h"
#include "clampNode.h"
#include "valueInputNode.h"

namespace cauv{
namespace imgproc{

template<> DEFINE_NFR(MathAddMultNode<int>, NodeType::MathAddMultInt);
template<> DEFINE_NFR(MathAddMultNode<float>, NodeType::MathAddMultFloat);
template<> DEFINE_NFR(MathDivideNode<int>, NodeType::MathDivideInt);
template<> DEFINE_NFR(MathDivideNode<float>, NodeType::MathDivideFloat);
template<> DEFINE_NFR(ClampNode<int>, NodeType::ClampInt);
template<> DEFINE_NFR(ClampNode<float>, NodeType::ClampFloat);
template<> DEFINE_NFR(ValueInputNode<int32_t>, NodeType::IntInput);
template<> DEFINE_NFR(ValueInputNode<float>, NodeType::FloatInput);
template<> DEFINE_NFR(ValueInputNode<bool>, NodeType::BoolInput);

// not really maths, but seemed to belong here...
template<> DEFINE_NFR(ValueInputNode<std::string>, NodeType::StringInput);

} // namespace imgproc
} // namespace cauv

