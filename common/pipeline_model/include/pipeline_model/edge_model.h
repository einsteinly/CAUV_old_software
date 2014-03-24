#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include <functional>
#include <XmlRpcValue.h>
#include "param_model.h"

namespace cauv {
namespace pipeline_model {

class OutputModel;

//input model: an input, and the corresponding output
class InputModel : public BoundParamModel {
    public:
    InputModel (const ParamModel &model, const boost::shared_ptr<NodeModel> node)
        : BoundParamModel(model, node),
          input(nullptr) {};
    using BoundParamModel::BoundParamModel;
    OutputModel *input;
};

//ouput model: an output, and the corresponding inputs
class OutputModel : public BoundParamModel {
    public:
    using BoundParamModel::BoundParamModel;
    std::vector<std::reference_wrapper<InputModel>> outputs;
};

}
}
