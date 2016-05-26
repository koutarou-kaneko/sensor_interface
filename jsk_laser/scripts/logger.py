#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('jsk_laser')
from jsk_laser.msg import JskLaser
##pyqtgraph lib
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg


#QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
mw = QtGui.QMainWindow()
mw.setWindowTitle('JSK Laser Demo')
mw.resize(800,800)
cw = QtGui.QWidget()
mw.setCentralWidget(cw)
l = QtGui.QVBoxLayout()
cw.setLayout(l)

pw = pg.PlotWidget(name='JSK Laser Logger')  ## giving the plots names allows us to link their axes together
l.addWidget(pw)

mw.show()

## Create an empty plot curve to be filled later, set its pen
p1 = pw.plot()
p1.setPen((0,200,0))

## Add in some extra graphics
rect = QtGui.QGraphicsRectItem(QtCore.QRectF(0, 0, 1, 5e-11))
rect.setPen(QtGui.QPen(QtGui.QColor(100, 200, 100)))
pw.addItem(rect)

pw.setLabel('left', 'Distance', units='cm')
pw.setLabel('bottom', 'Pixels', units='')
pw.setXRange(0, 256)
pw.setYRange(0, 300)

def rand(n):
    data = np.random.random(n)
    data[int(n*0.1):int(n*0.13)] += .5
    data[int(n*0.18)] += 2
    data[int(n*0.1):int(n*0.13)] *= 5
    data[int(n*0.18)] *= 20
    data *= 1e-12
    return data, np.arange(n, n+len(data)) / float(n)


def updateData():
    yd, xd = rand(10000)
    p1.setData(y=yd, x=xd)


def clicked():
    print("curve clicked")

def callback(data):
    rospy.loginfo("%f", data.distances[100]*0.01)
    data.distances = list(data.distances)
    for i in range(0, 20):
        data.distances[i] = np.asarray(data.distances[i]) * 0
    for i in range(210,271):
        data.distances[i] = np.asarray(data.distances[i]) * 0
    data.distances = np.asarray(data.distances) * 0.01
    p1.setData(data.distances)

def logger():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laser_logger', anonymous=True)

    rospy.Subscriber("/laser_data", JskLaser, callback)

    # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    logger()
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
        rospy.spin()