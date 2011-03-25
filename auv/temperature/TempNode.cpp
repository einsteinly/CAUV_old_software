#include "TempNode.h"

TempNode::TempNode(std::string f):CauvNode("temp")
{
  SetConfigFile(f);
}

TempNode::~TempNode()
{
  sensors_cleanup();
  fclose(f__);
}

int TempNode::SetConfigFile(std::string f)
{
  f__ = f;
  file__ = fopen(f__,'r');
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

floatvec TempNode::temps()
{
  int nr = 0;
  const sensors_chip_name *chip_name;
  while(chip_name = sensors_get_detected_chips(&nr))
  {
    sensors.push_back(chip_name);
  }
  
  std::vector<sensors_chip_name*>::iterator it;
  for(it = sensors.begin() ; it != sensors.end(); it++)
  {
   
