#include "TempNode.h"

TempNode::TempNode(std::string f):CauvNode("temp")
{
  SetConfigFile(f);
}

TempNode::~TempNode()
{
  sensors_cleanup();
  fclose(file__);
}

int TempNode::SetConfigFile(std::string f)
{
  f__ = f;
  file__ = fopen(f__.c_str(),"r");
  if(file__ == NULL)
  {
    debug() << "lm_sensors config file not found!";
    return 1;
  }
  if(sensors_init(file__) != 0)
  {
    debug() << "sensors config file appears to be buggered.";
  }
  
  return 0;
}

void TempNode::GetChipNames()
{
  chips.clear();
  int nr = 0;
  const sensors_chip_name *chip_name;
  while(chip_name = sensors_get_detected_chips(&nr))
  {
    GetFeatureData(*chip_name);
  }
}

void TempNode::GetFeatureData(sensors_chip_name chip_name)
{
  int n1, n2;
  n1 = 0;
  n2 = 0;
  const sensors_feature_data *feature_data;
  std::vector<const sensors_feature_data*> features;
  while(feature_data = sensors_get_all_features(chip_name, &n1, &n2))
  {
    features.push_back(feature_data);
  }
  chips[chip_name] = features;
}
   
