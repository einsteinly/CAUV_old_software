import time


class expWindow(object):
    def __init__(self, n, alpha):
        self.n = n
        self.alpha = alpha
    def get_window(self):
        return tuple([self.alpha ** i for i in xrange(0, self.n)])

class PIDController(object):
    def __init__(self, (Kp, Ki, Kd), d_window = None, err_clamp = 1e9):
        # the most recent end of the window is the start
        self.err = 0
        self.last_time = None
        self.derrs = []
        self.ierr = 0
        self.derr = 0
        self.d_window = d_window
        if d_window is None:
            self.derr_window = [1]
        else:
            self.derr_window = d_window.get_window()
        self.err_clamp = err_clamp
        if abs(sum(self.derr_window)) < 1e-30:
            raise Exception('Bad Window')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
    
    def calcDErr(self):
        # apply self.derr_window over self.derrs
        renorm = sum(self.derr_window)
        keep_derrs = max(10, len(self.derr_window))
        self.derrs = self.derrs[-keep_derrs:]
        denormed = 0
        for i, derr in enumerate(self.derrs):
            if i == len(self.derr_window):
                break
            denormed += derr * self.derr_window[-i]
        return denormed / renorm
    
    def setKpid(self, (Kp, Ki, Kd)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
    def update(self, err):
        now = time.time()
        last_err = self.err
        self.err = err
        if self.last_time is not None:
            self.derrs.append((self.err - last_err) / (now - self.last_time))
        else:
            self.derrs.append(0)
        self.derr = self.calcDErr()
        
        self.last_time = now
        
        self.ierr += self.err
        
        if self.ierr > self.err_clamp:
            self.ierr = self.err_clamp
        elif self.ierr < -self.err_clamp:
            self.ierr = -self.err_clamp
    
        return self.Kp * self.err +\
               self.Ki * self.ierr +\
               self.Kd * self.derr

    def to_dict(self):
        return {"Kp": self.Kp,
                "Ki": self.Ki,
                "Kd": self.Kd,
                "clamp": self.err_clamp}

    def from_dict(self, _dict):
        self.Kp = _dict["Kp"]
        self.Ki = _dict["Ki"]
        self.Kd = _dict["Kd"]
        self.err_clamp = _dict["clamp"]
