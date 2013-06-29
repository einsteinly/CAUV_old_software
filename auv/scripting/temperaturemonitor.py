#!/usr/bin/env python2.7
import argparse
import time
from cauv import messaging
from cauv.node import Node



## Sends messages repeatedly with time gap to be specified(default:2sec)

def main():
    node = Node('temperaturemonitor.py')
    parser = argparse.ArgumentParser(description="Get temperature of CPU cores.")
    parser.add_argument("--interval", type = int, default = 2, help = 'Sets time interval between each update on CPU temperature')
    parser.add_argument("--source", choices = ["pysensors", "sysfs"], default = "sysfs")
    args = parser.parse_args()

    if args.source == "sysfs":
        while True:
            with open('/sys/class/thermal/thermal_zone0/temp') as temp_file:
                temp = int(temp_file.read().strip()) / 1000.0
                node.send(messaging.CPUTemperatureMessage(temp,-1))
                time.sleep(args.interval)
    else:
        import sensors
        sensors.init()
        while True:        
            for chip in sensors.iter_detected_chips():
                ##print '%s at %s:' % (chip, chip.adapter_name)
                temperatures = []
                for feature in chip:
                    temperatures.append(feature.get_value())
                node.send(messaging.CPUTemperatureMessage(temperatures[0],temperatures[1]))       
            time.sleep(args.interval)
        node.stop()
        sensors.cleanup()

if __name__ == "__main__":
    main()

