import sensors
import argparse
import time
from cauv import messaging
from cauv.node import Node



## Sends messages repeatedly with time gap to be specified(default:2sec)

def main():
    node = Node('temperaturemonitor.py')
    parser = argparse.ArgumentParser(description="Get temperature of CPU cores.")
    parser.add_argument("--interval", type = int, default = 2, help = 'Sets time interval between each update on CPU temperature')
    args = parser.parse_args()

    sensors.init()
    while True:        
        for chip in sensors.iter_detected_chips():
            ##print '%s at %s:' % (chip, chip.adapter_name)
            temperatures = []
            for feature in chip:
                temperatures.append(feature.get_value())
            node.send(messaging.CPUTemperatureMessage(temperatures[0],temperatures[1]))       
        time.sleep(args.interval)
        #node.send(messaging.CPUTemperatureMessage(10,10)) ##temporary line to see if messaging is working (no sensors on virtual machine)
    node.stop()
    sensors.cleanup()

if __name__ == "__main__":
    main()

