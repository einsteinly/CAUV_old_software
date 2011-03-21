import cauv
import cauv.messaging as messaging
import cauv.node

import threading
import logging
import time

log_file = 'bat_log.log'

motor_conversion_factor = 80.0/127.0 #Watts 80 wats max power, 127 max motor demand
lights_consumption = 32#Watts
computer_consumption = 30#Watts 1.5 amps at 19 volts
constant_power_use = lights_consumption+computer_consumption #Watts
battery_total = 89.1*2 #Watt hours
#
def estimatePowerUse(motor_power_list, time):
    try:
        return ((sum(motor_power_list)/len(motor_power_list))*motor_conversion_factor+constant_power_use)*time/3600
    except ZeroDivisionError:
        print "Error, no data"
        return 0

class motorStateLogger(messaging.BufferedMessageObserver):
    def __init__(self, node):
        messaging.BufferedMessageObserver.__init__(self)
        node.join("gui")
        node.addObserver(self)
        self.node = node
        self.motor_demand_log_lock = threading.Lock()
        self.motor_demand_log = []
    def onMotorStateMessage(self, m):
        self.motor_demand_log_lock.acquire()
        self.motor_demand_log.append([time.time(),abs(m.demand.prop)+abs(m.demand.hstern)+abs(m.demand.vstern)+abs(m.demand.hbow)+abs(m.demand.vbow)])
        self.motor_demand_log_lock.release()
#
class slowLogger():
    def __init__(self, filename, frequency, logger, node):
        #open log, or create new
        self.frequency = frequency
        self.last_log_time = time.time()
        self.logger = logger
        self.total_usage = 0
        self.log_file = open(filename,'r+')
        self.node = node
    def load_from_file(self):
        for line in self.log_file.readlines():
            self.total_usage = float(line.split('-')[2])
    def run(self):
        while True:
            self.logger.motor_demand_log_lock.acquire()
            current_time = time.time()
            motor_log = [x[1] for x in self.logger.motor_demand_log]
            self.logger.motor_demand_log = []
            self.logger.motor_demand_log_lock.release()
            interval = current_time-self.last_log_time
            estimated_power_use = estimatePowerUse(motor_log, interval)
            self.total_usage += estimated_power_use
            self.log_file.write(str(current_time)+'-'+str(estimated_power_use)+'-'+str(self.total_usage)+'\n')
            self.log_file.flush()
            print "Total power use: "+str(self.total_usage)+" Watt Hours, estimated current use: "+str(estimated_power_use/interval*3600)+" Watts"
            fraction_remaining = 1-self.total_usage/battery_total
            print "Estimated battery remaining: "+str(fraction_remaining*100)+'%'
            self.node.send(messaging.BatteryUseMessage(estimated_power_use, self.total_usage, fraction_remaining), "telemetry")
            self.last_log_time = current_time
            print "sleeping"
            time.sleep(self.frequency)
        
#
def reset():
    log = open(log_file, 'w')
    log.truncate()
    log.close

def run():
    monitor_node = cauv.node.Node('pybatmon')
    broadcast_node = cauv.node.Node('pybatbrd')
    state_logger = motorStateLogger(monitor_node)
    slow_logger = slowLogger(log_file, 2, state_logger, broadcast_node)
    slow_logger.load_from_file()
    slow_logger.run()
    
