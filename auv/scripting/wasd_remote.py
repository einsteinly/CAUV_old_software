#!/usr/bin/env python2.7

# CAUV Modules
import cauv
import cauv.messaging as msg
import cauv.node as node
import cauv.control as control
from cauv.debug import debug, warning, error, info

# Standard Library Modules
import Tkinter as tk

Depth_Inc = 0.1
Bearing_Inc = 2
Pitch_Inc = 0.2 
Prop_Value = 80
Strafe = 40

class WASDRemote(msg.MessageObserver):
    def __init__(self, node):
        msg.MessageObserver.__init__(self)
        self.node = node
        self.node.join('gui')
        self.node.join('telemetry')
        self.node.addObserver(self)        
        self.auv = control.AUV(node)
        self.motor_state = {}
        self.motor_state[msg.MotorID.Prop] = 0
        self.motor_state[msg.MotorID.HBow] = 0
        self.motor_state[msg.MotorID.VBow] = 0
        self.motor_state[msg.MotorID.HStern] = 0
        self.motor_state[msg.MotorID.VStern] = 0
        self.bearing = 0
        self.pitch = 0
        self.depth = 0
        self.strafe = 0
        self.prop = 0
        self.last_telemetry = None

        self.tk = tk.Tk()
        self.tk.bind_all('<Key>', self.onKey) 
        #self.tk.withdraw()
        self.frame = tk.Frame(self.tk)
        self.frame.grid()
        self.motorlabel = tk.Label(self.frame, text=self.motorText())
        self.motorlabel.grid(row=1, column=1)
        
        self.telemetrylabel = tk.Label(self.frame, text=self.telemetryText())
        self.telemetrylabel.grid(row=2, column=1)
        
        self.demandlabel = tk.Label(self.frame, text=self.demandText())
        self.demandlabel.grid(row=2, column=1)

        self.display_tick()

    def onMotorStateMessage(self, m):
        debug('motor state: %s' % m)
        self.motor_state[m.motorId] = m.speed

    def onTelemetryMessage(self, m):
        self.last_telemetry = m

    def motorText(self):
        r = ''
        for k,v in self.motor_state.iteritems():
            r += '%s=%s ' % (k,v)
        return r
    
    def telemetryText(self):
        r = str(self.last_telemetry)
        if r.find('{') != -1:
            r = r[r.find('{'):r.rfind('}')]
        return r

    def demandText(self):
        return 'demand: bearing = %g, pitch = %g, depth = %g' % (
                    self.bearing, self.pitch, self.depth
                )

    def run(self):
        debug('main loop started')
        try:
            self.tk.lift()
            self.tk.mainloop()
        except KeyboardInterrupt:
            self.tk.destroy()
        debug('main loop finished')

    def onKey(self, event):
        if event.keysym == 'Escape':
            self.tk.destroy()
        if event.char == event.keysym:
            debug('keysym %r' % event.char)
        elif len(event.char) == 1:
            debug('keysym %r (ch=%r)' % (event.keysym, event.char))
        else:
            debug('special: %r' %  event.keysym)

        if event.keysym_num == 65362: # Up
            self.depth -= Depth_Inc
        elif event.keysym_num == 65364: # Down
            self.depth += Depth_Inc
        
        if event.keysym_num == 65363: # Right
            self.strafe = Strafe
        elif event.keysym_num == 65361: # Left
            self.strafe = -Strafe
        else:
            self.strafe = 0

        if event.keysym_num == 119: # w
            self.prop = Prop_Value
        elif event.keysym_num == 115: # s
            self.prop = -Prop_Value
        else:
            self.prop = 0

        if event.keysym_num == 97: # a
            if self.auv.current_bearing is not None:
                self.bearing -= Bearing_Inc
        elif event.keysym_num == 100: # d
            if self.auv.current_bearing is not None:
                self.bearing += Bearing_Inc
            
        self.update()
      
    def update(self):
        if self.depth > 8:
            self.depth = 8
        if self.depth < -4:
            self.depth = -4
        self.auv.strafe(self.strafe)
        self.auv.prop(self.prop)
        self.auv.bearing(self.bearing)
        self.auv.depth(self.depth)
        self.updateDisplay()

    def updateDisplay(self):
        self.motorlabel.configure(text=self.motorText())
        self.motorlabel.pack()
        self.telemetrylabel.configure(text=self.telemetryText())
        self.telemetrylabel.pack()
        self.demandlabel.configure(text=self.demandText())
        self.demandlabel.pack()

    def display_tick(self):
        self.updateDisplay()
        self.tk.after(100,self.display_tick)


    
def main():
    import sys
    n = node.Node("py-wasd",sys.argv[1:])
    try:
        r = WASDRemote(n)
        r.run()
    finally:
        n.stop()

if __name__ == '__main__':
    main()

