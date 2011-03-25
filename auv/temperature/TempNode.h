#include <boost/make_shared.hpp>
#include <common/cauv_node.h>
#include <common/cauv_global.h>
#include <debug/cauv_debug.h>
#include <sensors/sensors.h>
#include <vector>
#include <string>
#include <cstdio>

typedef std::vector<double> doublevec;

class TempNode : public CauvNode
{
  public:
    TempNode(std::string f);
    ~TempNode();
    int SetConfigFile(std::string f);
    void GetChipNames();
    void GetFeatureData(sensors_chip_name *chip_name);
  protected:
    virtual void onRun();
  private:
    doublevec temps();
    std::string f__;
    FILE *file__;
    std::map<sensors_chip_name*, std::vector<sensors_feature_data*> > chips;
};
