#!/usr/local/bin/python

import sys, serial, argparse
import numpy as np
from collections import deque

import matplotlib.pyplot as plt 
import matplotlib.animation as animation

    
class AnalogPlot:
  def __init__(self, strPort, maxLen):
      self.ser = serial.Serial(strPort, 9600)

      self.ax = deque([0.0]*maxLen)
      self.ay = deque([0.0]*maxLen)
      self.maxLen = maxLen

  def addToBuf(self, buf, val):
      if len(buf) < self.maxLen:
          buf.append(val)
      else:
          buf.pop()
          buf.appendleft(val)

  def add(self, data):
      assert(len(data) == 2)
      self.addToBuf(self.ax, data[0])
      self.addToBuf(self.ay, data[1])

  def update(self, frameNum, a0, a1):
      try:
          line = self.ser.readline()
          data = [float(val) for val in line.split()]
          print data
          if(len(data) == 2):
              self.add(data)
              a0.set_data(range(self.maxLen), self.ax)
              a1.set_data(range(self.maxLen), self.ay)
      except KeyboardInterrupt:
          print('exiting')
      
      return a0, 

  def close(self):
      self.ser.flush()
      self.ser.close()

def main():
    parser = argparse.ArgumentParser(description="Plotter")

    parser.add_argument('--port', dest = 'port', required = False, default = '/dev/cu.usbmodemFA131')
    parser.add_argument('--ymin', dest = 'ymin', required = False, default = 0, type = int)
    parser.add_argument('--ymax', dest = 'ymax', required = False, default = 300, type = int)
    parser.add_argument('-v', dest = 'verbose', required = False, action="store_true")

    args = parser.parse_args()

    strPort = args.port

    print('reading from port %s...' % strPort)

    analogPlot = AnalogPlot(strPort, 100)

    print('plotting data...')

    fig = plt.figure()
    ax = plt.axes(xlim=(0, 100), ylim=(args.ymin, args.ymax))
    a0, = ax.plot([], [])
    a1, = ax.plot([], [])
    anim = animation.FuncAnimation(fig, analogPlot.update, 
                                 fargs=(a0, a1), 
                                 interval=50)

    plt.show()

    analogPlot.close()

    print('exiting.')


if __name__ == '__main__':
    main()