#!/usr/bin/env python

import cauv
import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, info

import threading
import time
import optparse
import traceback

log_file = 'bat_log.log'

#useful constants
motor_conversion_factor = 80.0/127.0 #Watts 80 watts max power, 127 max motor demand
light_conversion_factor = 32.0/255 #Watts 32 watts max power, 255 max demand
voltage_conversion = 5.69E-3 #number->volts
computer_consumption = 30.0#Watts 1.5 amps at 19 volts
battery_total = 89.1*4 #Watt hours

#function to estimate power use
def estimateMotorUse(logs, current_time, last_log_time):
    power_used = 0
    motor_log = {}
    for motor in motor_ids:
        #get the last messages for each motor
        with logs[0].motor_demand_log_lock[motor]:
            motor_log[motor] = logs[0].motor_demand_log[motor]
            #add a point at the current time
            if len(motor_log[motor]):
                motor_log[motor].append((current_time, motor_log[motor][-1][1]))
                logs[0].motor_demand_log[motor] = [motor_log[motor][-1]] #reset the log, leaving the last value to start the next one
        try:
            for x in range(len(motor_log[motor])-1):
                power_used += motor_conversion_factor*abs(motor_log[motor][x+1][1]+motor_log[motor][x][1])*(motor_log[motor][x+1][0]-motor_log[motor][x][0])/7200.0 #average power * time in hours
        except IndexError:
            debug("Error, no data for "+motor)
    return power_used
        
def estimateComputerUse(logs, current_time, last_log_time):
    return computer_consumption*(current_time-last_log_time)/3600

def estimateCharge(logs, current_time, last_log_time):
    return 0
        
def estimateLightUse(logs, current_time, last_log_time):
    power_used = 0
    light_log = {}
    for light in light_ids:
        with logs[0].light_log_lock[light]:
            light_log[light] = logs[0].light_log[light]
            #add a point at the current time
            if len(light_log[light]):
                light_log[light].append((current_time, light_log[light][-1][1]))
                logs[0].light_log[light] = [light_log[light][-1]] #reset the log, leaving the last value to start the next one
        try:
            for x in range(len(light_log[light])-1):
                power_used += light_conversion_factor*light_log[light][x][1]*(light_log[light][x+1][0]-light_log[light][x][0])/3600 #power * time
        except IndexError:
            debug("Error, no data for "+light)
    return power_used

modules_dict = {
                'm': estimateMotorUse,
                'l': estimateLightUse,
                'c': estimateComputerUse,
                'v': estimateCharge
                }

#message logger
motor_ids=['Prop','VBow','HBow','VStern','HStern']
light_ids=['Forward', 'Down']
battery_ids=['Main']
class messageLogger(messaging.BufferedMessageObserver):
    def __init__(self, node):
        messaging.BufferedMessageObserver.__init__(self)
        node.join("gui")
        node.join("external")
        node.addObserver(self)
        self.node = node
        self.motor_demand_log_lock = dict([(x,threading.Lock()) for x in motor_ids])
        self.motor_demand_log = dict([(x,[]) for x in motor_ids])
        self.light_log_lock = dict([(x,threading.Lock()) for x in light_ids])
        self.light_log = dict([(x,[(time.time(),0)]) for x in light_ids])
        self.battery_log_lock = dict([(x,threading.Lock()) for x in battery_ids])
        self.battery_log = dict([(x,[(time.time(),0)]) for x in battery_ids])
    def onBatteryStatusMessage(self, m):
        info('Battery Voltage: %gV' % (m.voltage * voltage_conversion))
        with self.battery_log_lock['Main']:
            self.battery_log['Main'].append((time.time(),m.voltage * voltage_conversion))
    def onMotorStateMessage(self, m):
        with self.motor_demand_log_lock[str(m.motorId)]:
            self.motor_demand_log[str(m.motorId)].append((time.time(),m.speed))
    def onLightMessage(self, m):
        with self.light_log_lock[str(m.lightId)]:
            self.light_log[str(m.lightId)].append((time.time(),m.intensity))
        
loggers_dict = {
                'm' : (messageLogger,),
                'l' : (messageLogger,),
                'c' : tuple(),
                'v' : (messageLogger,),
                }

#actual logger that calculates power use, saves to file and sends messages
class slowLogger():
    def __init__(self, filename, frequency, node, modules):
        #set initial values
        self.frequency = frequency
        self.node = node
        self.last_log_time = time.time()
        self.total_usage = 0
        self.modules = modules
        self.loggers = {}
        logger_lookup = {}
        for m in modules:
            logger_list = []
            for logger in loggers_dict[m]:
                if not logger in logger_lookup:
                    logger_lookup[logger] = (logger(node))
                logger_list.append(logger_lookup[logger])
            self.loggers[m] = logger_list
        #open log file
        self.log_file = open(filename,'r+')
    def load_from_file(self):
        #set the total usage to the last total usage recorded in the log
        for line in self.log_file.readlines():
            self.total_usage = float(line.split('-')[2])
    def run(self):
        while True:
            estimated_power_use = 0
            current_time = time.time()
            for m in self.modules:
                estimated_power_use += modules_dict[m](self.loggers[m], current_time, self.last_log_time)
            #calculate the power since the last loop
            interval = current_time-self.last_log_time
            self.total_usage += estimated_power_use
            fraction_remaining = 1-self.total_usage/battery_total
            self.log_file.write(str(current_time)+'-'+str(estimated_power_use)+'-'+str(self.total_usage)+'\n')
            self.log_file.flush() #force write to file
            info("Total power use: "+str(self.total_usage)+" Watt Hours, estimated current use: "+str(estimated_power_use/interval*3600)+" Watts, estimated battery remaining: "+str(fraction_remaining*100)+'%')
            self.node.send(messaging.BatteryUseMessage(estimated_power_use/interval*3600, self.total_usage, fraction_remaining), "telemetry")
            self.last_log_time = current_time
            #sleep
            time.sleep(self.frequency)
        
#reset log
def reset():
    log = open(log_file, 'w')
    log.truncate()
    log.close

def run(modules):
    #set up nodes
    node = cauv.node.Node('pybatmon')
    #set up loggers
    slow_logger = slowLogger(filename=log_file, frequency=2, node=node, modules=modules)
    slow_logger.load_from_file()
    slow_logger.run()
    
if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-r', '--reset', dest='reset', default=False,
                 action='store_true', help="create or reset battery log")
    p.add_option('-e', '--enable', type='string', dest='enable', default='',
                 action='store', help="enable estimates for these modules")
    p.add_option('-d', '--disable', type='string', dest='disable', default='',
                 action='store', help="disable estimates for these modules")
    
    """
    modules
    m  -  motors
    l  -  lights
    c  -  computer
    v  -  voltage
    """
    modules = set(['m','l','c','v']) #default
    opts, args = p.parse_args()
    enable = opts.enable.split(',') if len(opts.enable) else []
    disable = opts.disable.split(',') if len(opts.disable) else []
    for opt in enable:
        modules.add(opt)
    for opt in disable:
        try:
            modules.remove(opt)
        except KeyError:
            print 'Module '+opt+' does not exist'
    
    if opts.reset:
        reset()
    else:
        print 'Modules '+str(modules)+' enabled'
        run(modules)
