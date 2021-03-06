#include <boost/make_shared.hpp>
#include <common/cauv_node.h>
#include <debug/cauv_debug.h>
#include <sensors/sensors.h>
#include <vector>
#include <map>
#include <string>
#include <cstdio>

namespace cauv{
struct chip_name_comp
{
  bool operator()(sensors_chip_name n1, sensors_chip_name n2) const
  {
    return (n1.addr < n2.addr);
  }
};

typedef std::map<sensors_chip_name, 
        std::vector<const sensors_feature_data*>, chip_name_comp> 
        chip_features;


class TempNode : public CauvNode
{
  public:
    TempNode(const std::string& f);
    ~TempNode();
    int SetConfigFile(const std::string& f);
    void GetChipNames();
    void GetFeatureData(sensors_chip_name chip_name);
  private:
    std::string f__;
    FILE *file__;
    chip_features chips;
};
} //end namespace cauv
