#include "../nodeFactory.h"

#include "divideNode.h"
#include "addMultNode.h"

namespace cauv{
namespace imgproc{

template<> DEFINE_NFR(MathAddMultNode<int>, NodeType::MathAddMultInt);
template<> DEFINE_NFR(MathAddMultNode<float>, NodeType::MathAddMultFloat);
template<> DEFINE_NFR(MathDivideNode<int>, NodeType::MathDivideInt);
template<> DEFINE_NFR(MathDivideNode<float>, NodeType::MathDivideFloat);
template<> DEFINE_NFR(ClampNode<int>, NodeType::ClampInt);
template<> DEFINE_NFR(ClampNode<float>, NodeType::ClampFloat);

} // namespace imgproc
} // namespace cauv


