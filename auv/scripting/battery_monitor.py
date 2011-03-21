import cauv
import cauv.messaging as messaging
import cauv.node

import threading
import time
import optparse

log_file = 'bat_log.log'

motor_conversion_factor = 80.0/127.0 #Watts 80 wats max power, 127 max motor demand
lights_consumption = 32#Watts
computer_consumption = 30#Watts 1.5 amps at 19 volts
constant_power_use = lights_consumption+computer_consumption #Watts
battery_total = 89.1*2 #Watt hours

#function to estimate power use
def estimateMotorPowerUse(motor_log):
    power_used = 0
    for motor in motor_log:
        try:
            for x in range(len(motor_log[motor])-1):
                power_used += (motor_log[motor][x+1][1]+motor_log[motor][x][1])/(motor_log[motor][x+1][0]-motor_log[motor][x][0])
        except IndexError:
            print "Error, no data for "+motor
    return power_used
        
def estimateConstantPowerUse(interval):
    return constant_power_use*interval/3600

motor_ids=['prop','vbow','hbow','vstern','hstern']
#message logger
class motorStateLogger(messaging.BufferedMessageObserver):
    def __init__(self, node):
        messaging.BufferedMessageObserver.__init__(self)
        node.join("gui")
        node.addObserver(self)
        self.node = node
        self.motor_demand_log_lock = dict([(x,threading.Lock()) for x in motor_ids])
        self.motor_demand_log = dict([(x,[]) for x in motor_ids])
    def onMotorStateMessage(self, m):
        print 'received motor message'
        self.motor_demand_log_lock[m.MotorID].acquire()
        self.motor_demand_log[m.MotorId].append([time.time(),m.speed])
        self.motor_demand_log_lock[m.MotorID].release()

#actual logger that calculates power use, saves to file and sends messages
class slowLogger():
    def __init__(self, filename, frequency, logger, node):
        #set initial values
        self.frequency = frequency
        self.logger = logger
        self.last_log_time = time.time()
        self.total_usage = 0
        self.node = node
        #open log file
        self.log_file = open(filename,'r+')
    def load_from_file(self):
        #set the total usage to the last total usage recorded in the log
        for line in self.log_file.readlines():
            self.total_usage = float(line.split('-')[2])
    def run(self):
        while True:
            estimated_power_use = 0
            motor_log = {}
            for motor in motor_ids:
                #get the last messages for each motor
                self.logger.motor_demand_log_lock[motor].acquire()
                current_time = time.time() #set the time of getting the log
                motor_log[motor] = self.logger.motor_demand_log[motor]
                if len(self.logger.motor_demand_log[motor]):
                    self.logger.motor_demand_log[motor] = [self.logger.motor_demand_log[motor][-1]] #reset the log, leaving the last value to start the next one
                self.logger.motor_demand_log_lock[motor].release()
            #calculate the power since the last loop
            estimated_power_use += estimateMotorPowerUse(motor_log)
            current_time = time.time()
            interval = current_time-self.last_log_time
            estimated_power_use += estimateConstantPowerUse(interval)
            self.last_log_time = current_time
            self.total_usage += estimated_power_use
            fraction_remaining = 1-self.total_usage/battery_total
            self.log_file.write(str(current_time)+'-'+str(estimated_power_use)+'-'+str(self.total_usage)+'\n')
            self.log_file.flush() #force write to file
            print "Total power use: "+str(self.total_usage)+" Watt Hours, estimated current use: "+str(estimated_power_use/interval*3600)+" Watts, estimated battery remaining: "+str(fraction_remaining*100)+'%'
            self.node.send(messaging.BatteryUseMessage(estimated_power_use, self.total_usage, fraction_remaining), "telemetry")
            self.last_log_time = current_time
            #sleep
            time.sleep(self.frequency)
        
#reset log
def reset():
    log = open(log_file, 'w')
    log.truncate()
    log.close

def run():
    #set up nodes
    monitor_node = cauv.node.Node('pybatmon')
    broadcast_node = cauv.node.Node('pybatbrd')
    #set up loggers
    state_logger = motorStateLogger(monitor_node)
    slow_logger = slowLogger(log_file, 2, state_logger, broadcast_node)
    slow_logger.load_from_file()
    slow_logger.run()
    
if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-r', '--reset', dest='reset', default=False,
                 action='store_true', help="create or reset battery log")
    
    opts, args = p.parse_args()
    
    if opts.reset:
        reset()
    else:
        run()
