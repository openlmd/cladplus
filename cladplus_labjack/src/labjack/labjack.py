import yaml
import math
import signal

import u3


class LabJack():
    def __init__(self):
        self.power_factor(0, 1500)
        self.count = 0
        self.setDacCount = 0
        self.go = True
        self.openu3()
        self.reg = 5000

    def load_config(self, filename):
        with open(filename, "r") as ymlfile:
            cfg = yaml.load(ymlfile)
        pwr_min = cfg['power']['min']
        pwr_max = cfg['power']['max']
        self.power_factor(pwr_min, pwr_max)

    def power_factor(self, pwr_min, pwr_max):
        self.factor = 5.0 / (pwr_max - pwr_min)
        return self.factor

    def openu3(self):
        print 'Opening LabJack...'
        try:
            self.dac = u3.U3()
            print 'Done'
        except:
            print 'The device may be not connected'

    def close(self):
        print 'Closing LabJack...'
        try:
            self.dac.closeu3()
            print 'Done'
        except:
            print 'The device could be not closed'

    def setDac(self):
        # calculate the value to put in the sin
        value = (self.setDacCount * self.step) * self.degToRad

        self.dac.writeRegister(self.reg, 2+2*math.sin(value))
        # Count measures how many successful updates occurred.
        self.count += 1

        # Lower the go flag
        self.go = False

    def output(self, value):
        self.dac.writeRegister(self.reg, value)

    def triangular(self, maxim):
        while(1):
            for k in np.linspace(0, maxim, 10):
                print k
                dacs.output(k)
                time.sleep(1)
            k = 0

    def handleSetDac(self, signum, frame):
        # This function gets called every UPDATE_INTERVAL seconds.
        # Raise the go flag.
        self.go = True

        # setDacCount measures how many times the timer went off.
        self.setDacCount += 1

    def singenerator(self, FREQUENCY, UPDATE_INTERVAL):
        print "This program will attempt to generate a sine wave with a frequency of %s Hz, updating once every %s seconds." % (FREQUENCY, UPDATE_INTERVAL)
        # Controls how fast the DAC will be updated, in seconds.
        # Points between peaks (pbp)
        pbp = (float(1)/FREQUENCY)/UPDATE_INTERVAL

        # Figure out how many degrees per update we need to go.
        self.step = float(360)/pbp

        # Stupid sin function only takes radians... but I think in degrees.
        self.degToRad = ((2*math.pi) / 360)

        signal.signal(signal.SIGALRM, self.handleSetDac)
        signal.setitimer(signal.ITIMER_REAL, UPDATE_INTERVAL,
                         UPDATE_INTERVAL)

        while (1):
            # Wait for signal to be received
             # If the dacs flag is set, set the dac.
            if dacs.go:
                self.setDac()
        signal.setitimer(signal.ITIMER_REAL, 0)

        print "# of Updates = %s, # of1 signals = %s" % (dacs.count, dacs.setDacCount)
        print "The closer the number of updates is to the number of signals, the better your waveform will be."


if __name__ == '__main__':
    import time
    import numpy as np

    # Controls how fast the DAC will be updated, in seconds.
    UPDATE_INTERVAL = 0.005
    # The frequency of the sine wave, in Hz
    FREQUENCY = 10

    dacs = LabJack()

    while(1):
        dacs.output(1)

    #dacs.triangular(4)

    #dacs.singenerator(FREQUENCY, UPDATE_INTERVAL)

    dacs.close()
