#include <boost/make_shared.hpp>
#include <common/cauv_node.h>
#include <common/cauv_global.h>
#include <debug/cauv_debug.h>
#include <sensors/sensors.h>
#include <vector>
#include <string>
#include <cstdio>

typedef std::vector<float> floatvec;

class TempNode : public CauvNode
{
  public:
    TempNode(std::string f);
    ~TempNode();
    int SetConfigFile(std::string f);
  protected:
    virtual void onRun();
  private:
    floatvec temps();
    std::string f__;
    FILE *file__;
    std::vector<sensors_chip_name*> sensors;
    std::vector<sensors_feature_data> features;
};
