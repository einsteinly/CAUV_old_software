#include "variants.h"

using namespace cauv;
using namespace cauv::gui;

template <> std::string id_to_name::operator()( AutopilotID::e & operand ) const
{
    switch (operand){
    case AutopilotID::Bearing: return "bearing";
    case AutopilotID::Depth: return "depth";
    case AutopilotID::Pitch: return "pitch";
    }
    return "unknown";
}
