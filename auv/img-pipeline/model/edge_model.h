#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include <functional>
#include <XmlRpcValue.h>
#include "param_model.h"

namespace cauv {

class OutputModel;

class InputModel : public BoundParamModel {
    public:
    InputModel (const ParamModel &model, const boost::shared_ptr<NodeModel> node)
        : BoundParamModel(model, node),
          input(nullptr) {};
    using BoundParamModel::BoundParamModel;
    OutputModel *input;
};

class OutputModel : public BoundParamModel {
    public:
    using BoundParamModel::BoundParamModel;
    std::vector<std::reference_wrapper<InputModel>> outputs;
};

}
