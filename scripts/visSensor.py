#!/usr/local/bin/python

import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque

import matplotlib.pyplot as plt 
import matplotlib.animation as animation

class SensorPlot:
    def __init__(self, strPort, numPoints, numSeries = 1):
        # 
        # Open serial port.
        self.ser = serial.Serial(strPort, 115200)
        self.numSeries = numSeries
        self.numPoints = numPoints
        # 
        # Create the queues.
        self.yax = [deque([0.0] * self.numPoints) for x in range(numSeries)]
    # 
    # Add to the time series.
    def addToBuf(self, buf, val):
        if len(buf) < self.numPoints:
            buf.append(val)
        else:
            buf.pop()
            buf.appendleft(val)
    # 
    # Fill objects.
    def add(self, data):
        axNum = 0
        # 
        # Skip the first index.
        itrData = iter(data)
        next(itrData)
        # 
        # Fill all axis.
        for datum in itrData:
            self.addToBuf(self.yax[axNum], datum)
            axNum += 1
    # 
    # Update.
    # def update(self, frameNum, a0, a1):
    def update(self, frameNum, axList):
        try:
            line = self.ser.readline()
            try:
                data = [float(val) for val in line.split()]
            except ValueError:
                print(line)
                return axList[0]
            # 
            # Create axis.
            axNum = 0
            if(len(data) == self.numSeries + 1):
                self.add(data)
                for ax in axList:
                    ax.set_data(range(self.numPoints), self.yax[axNum])
                    axNum += 1
            else:
                print('Unexpected length of data from serial exitting.')
        except KeyboardInterrupt:
            print('exiting')

        return axList[0]

    def close(self):
          self.ser.flush()
          self.ser.close()

def getArgs():
    parser = argparse.ArgumentParser(description="Plotter")
    parser.add_argument('--port', dest = 'port', required = False, default = '/dev/cu.usbmodem1421')
    parser.add_argument('--numPoints', dest = 'numPoints', required = False, default = 500, type = int)
    parser.add_argument('--numAxis', dest = 'numAxis', required = False, default = 3, type = int)
    parser.add_argument('--ymin', dest = 'ymin', required = False, default = -180, type = int)
    parser.add_argument('--ymax', dest = 'ymax', required = False, default = 180, type = int)
    args = parser.parse_args()
    return args

def main():

    args = getArgs()
    strPort = args.port
    # 
    # Set animation callback.
    fig = plt.figure()
    ax = plt.axes(xlim=(0, args.numPoints), ylim=(args.ymin, args.ymax))
    axList = []
    for x in range(args.numAxis):
        tempAx, = ax.plot([], [])
        axList.append(tempAx)
    anim = animation.FuncAnimation(fig, plot.update, 
                                   fargs=([axList]), 
                                   interval=50)
    plt.show()
    plot.close()

if __name__ == '__main__':
    main()
