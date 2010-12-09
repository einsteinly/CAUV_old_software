
import math

class MovingAverage():
       
    def __init__(self, side, tolerance=0.1, maxcount=500, st_on = 1):

        self.tolerance = tolerance
        self.maxcount = maxcount
        self.side = side
        self.st_on = st_on
        self.sample = 0
        self.movingMean=0
        self.movingError=0
        self.count = 0
        self.trigger = 0

    def update(self, sample):
        self.sample = sample
    
        if self.count==0:
            self.movingMean = sample
            self.count+=1
        else:
            
            if self.side == 'upper':                                   #if the lower send of the mean is sifnificant
                if sample>self.movingMean+self.tolerance:           #if sample is outside of the upper tolerance range
                    self.trigger = 1                                #set the trigger falg
                else:
                    self.trigger = 0                                #if not, clear trigger flag
                    self.movingMean = (self.count*self.movingMean+sample)/(self.count+1) #add sample to moving average
                    self.movingError = (self.count*self.movingError+math.fabs(sample-self.movingMean))/(self.count+1) #update movingError
                    
                        
                
            elif self.side == 'lower':                             #if the lower send of the mean is sifnificant
                if sample<self.movingMean-self.tolerance:           #if sample is outside of the lower tolerance range
                    self.trigger = 1
                else:
                    self.trigger = 0                                #if not, clear trigger flag
                    self.movingMean = (self.count*self.movingMean+sample)/(self.count+1) #add sample to moving average
                    self.movingError = (self.count*self.movingError+math.fabs(sample-self.movingMean))/(self.count+1) #update movingError

            elif self.side == 'both':                              #if the both upper and lower send of the mean is sifnificant
                if math.fabs(sample-self.movingMean)>self.tolerance:           #if sample is outside of the tolerance range
                    self.trigger = 1
                else:
                    self.trigger = 0                                #if not, clear trigger flag
                    self.movingMean = (self.count*self.movingMean+sample)/(self.count+1) #add sample to moving average
                    self.movingError = (self.count*self.movingError+math.fabs(sample-self.movingMean))/(self.count+1) #update movingError

                        
            if self.count < self.maxcount-2:                    #update count if not reached maxcount
                self.count += 1

            elif self.movingError!=0 & self.st_on==1:
                self.tolerance = 4*self.movingError
