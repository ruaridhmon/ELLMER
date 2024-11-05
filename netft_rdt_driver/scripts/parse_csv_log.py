#! /usr/bin/env python3


import numpy 
from scipy import stats
import IPython
import csv
import os
import matplotlib.pyplot as plt

def parseFTData(directory, fileName):
    ''' Parse through the fields below and extract dictionary of force and
    torque organized by time: 
    ['%time', 'field.header.seq', 'field.header.stamp', 'field.header.frame_id',
     'field.wrench.force.x', 'field.wrench.force.y', 'field.wrench.force.z', 
     'field.wrench.torque.x', 'field.wrench.torque.y', 'field.wrench.torque.z']'''
    i = 0
    ftData = []
    with open('{}/csv/{}__netft_netft_data.csv'.format(directory, fileName), 'r+') as f:
        fread = csv.reader(f, delimiter=',')
        for row in fread:
            if i > 0:
                time = float(row[0])
                ftData.append([float(r) for r in row[4:]])
            i += 1
    return numpy.array(ftData)

def plotFTData(data):
    xval = range(len(data))
    fig, (ax0, ax1) = plt.subplots(2, 1)
  
    ax0.plot(xval, data[:, 0], 'r', label="X")
    ax0.plot(xval, data[:, 1], 'g', label="Y")
    ax0.plot(xval, data[:, 2], 'b', label="Z")
    ax0.set_ylabel('Force')
    ax0.legend() 
                
    ax1.plot(xval, data[:, 3], 'r', label="X")
    ax1.plot(xval, data[:, 4], 'g', label="Y")
    ax1.plot(xval, data[:, 5], 'b', label="Z")
    ax1.set_ylabel('Torque')
    ax1.legend()
    plt.show()
 
if __name__ == '__main__':
    data = parseFTData('tests', 'i0')
    plotFTData(data)
    IPython.embed()
