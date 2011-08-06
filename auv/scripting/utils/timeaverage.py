from utils.ordereddict import OrderedDict
import time

class TimeAverage:

    def __init__(self, duration):
        self.duration = duration
        self.entries = OrderedDict()
        
    def setDuration(self, duration):
        self.duration = duration
        
    def update(self, value):
        now = time.time()
    
        # add the new value in
        self.entries[time.time()] = value
        
        # delete expired values
        for key in self.entries.keys():
            if key < (now - self.duration):
                del self.entries[key]
            else:
                #ordered map so once we've found a current item we can stop
                break
                
        # work out the average
        total = 0
        for value in self.entries.values():
            total += value
            
        return total / len(self.entries)
    
    def reset(self):
        self.entries.clear()



if __name__ == '__main__':
    av = TimeAverage(1)
    print av.update(2.0)
    print av.update(3.0)
    print av.update(4.0)
    time.sleep(2)
    print av.update(1.0)
    print av.update(2.0)
    print av.update(3.0)
    print av.update(4.0)
