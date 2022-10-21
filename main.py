# GROUND STATION CODE
import sys
from pyqtgraph.Qt import QtGui, QtCore, QtWidgets
import matplotlib.pyplot as plt
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PIL import Image, ImageDraw
from communication import Communication
from dataBase import data_base
from PyQt5.QtWidgets import *
from graphs.graph_acceleration import graph_acceleration
from graphs.graph_altitude import graph_altitude
from graphs.graph_battery import graph_battery
from graphs.graph_free_fall import graph_free_fall
from graphs.graph_gyro import graph_gyro
from graphs.graph_pressure import graph_pressure
from graphs.graph_speed import graph_speed
from graphs.graph_temperature import graph_temperature
from graphs.graph_time import graph_time
import gps_class

missionStarted = 0

def startMission():
    ser.write("g")
    missionStarted = 1
    print("worked")
    
def endMission():
    ser.write("e")

pg.setConfigOption('background', (255, 255, 255))
pg.setConfigOption('foreground', (0, 0, 0))

# Interface variables
app = QtWidgets.QApplication(sys.argv)
view = pg.GraphicsView()
Layout = pg.GraphicsLayout()
view.setCentralItem(Layout)
view.show()
view.setWindowTitle('Ground Station')
view.resize(1200, 700)

# declare object for serial Communication
input("Press enter to connect")
ser = Communication()
# declare object for storage in CSV
data_base = data_base()
# Fonts for text items
font = QtGui.QFont()
font.setPixelSize(90)

# buttons style
style = "background-color:rgb(29, 185, 84);color:rgb(0,0,0);font-size:14px;"

# Declare graphs
# Button 1
proxy1 = QtWidgets.QGraphicsProxyWidget()
start_button = QtWidgets.QPushButton('Start Mission (FS1)')
start_button.setStyleSheet(style)
start_button.clicked.connect(startMission)
proxy1.setWidget(start_button)

# # Button 2
proxy2 = QtWidgets.QGraphicsProxyWidget()
end_button = QtWidgets.QPushButton('End Mission (FS4)')
end_button.setStyleSheet(style)
end_button.clicked.connect(endMission) # reconfig
proxy2.setWidget(end_button)

# time, voltage widget
proxy3 = QtWidgets.QGraphicsProxyWidget()
timeWidget = QtWidgets.QLabel('TIME')
proxy3.setWidget(timeWidget)
proxy4 = QtWidgets.QGraphicsProxyWidget()
voltWidget = QtWidgets.QLabel('VOLTAGE')
proxy4.setWidget(voltWidget)
proxy5 = QtWidgets.QGraphicsProxyWidget()
fsWidget = QtWidgets.QLabel('FS')
proxy5.setWidget(fsWidget)
timeWidget.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
voltWidget.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
fsWidget.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
timeWidget.setFont(QtGui.QFont('Arial', 24))
voltWidget.setFont(QtGui.QFont('Arial', 24))
fsWidget.setFont(QtGui.QFont('Arial', 24))

# needed graphs: temp, altitude, pressure
altitude = graph_altitude()
pressure = graph_pressure()
temperature = graph_temperature()

# GPS graph
# gps_map = gps_class.GPSVis(data_path='data.csv',
#              map_path='map.png',  # Path to map downloaded from the OSM.
#              points=(-86.6453,34.7358,-86.6320,34.7188)) # Two coordinates of the map (upper left, lower right)
# gps_map.create_image(color=(0, 0, 255), width=3)  # Set the color and the width of the GNSS tracks.
# gps_map.plot_map(output='plot')

## Setting the graphs in the layout 
# Title at top
text = """
CANSAT Team 6 Ground Station
"""
Layout.addLabel(text, col=1, colspan=21)
Layout.nextRow()

# Put vertical label on left side
Layout.addLabel('Flight information',
                angle=-90, rowspan=3)
                
Layout.nextRow()

lb = Layout.addLayout(colspan=21)
lb.addItem(proxy1)
lb.nextCol()
lb.addItem(proxy2)

Layout.nextRow()

l1 = Layout.addLayout(colspan=20, rowspan=2)
l11 = l1.addLayout(rowspan=1, border=(83, 83, 83))

# Altitude, speed
l11.addItem(altitude)
l1.nextRow()

# Acceleration, gyro, pressure, temperature
l12 = l1.addLayout(rowspan=1, border=(83, 83, 83))
l12.addItem(pressure)
l12.addItem(temperature)

# # Time, battery and free fall graphs
l2 = Layout.addLayout(border=(83, 83, 83))
l2.addItem(proxy3)
l2.nextRow()
l2.addItem(proxy4)
l2.nextRow()
l2.addItem(proxy5)

# you have to put the position of the CSV stored in the value_chain list
# that represent the date you want to visualize
def update():
    try:
        # ser.write("g")
        value_chain = []
        value_chain = ser.getData()
        altitude.update(value_chain[5])
        pressure.update(value_chain[6])
        temperature.update(value_chain[7])
        timeWidget.setText(value_chain[1])
        voltWidget.setText(value_chain[8])
        fsWidget.setText(value_chain[3])
        proxy3.setWidget(timeWidget)
        proxy4.setWidget(voltWidget)
        proxy5.setWidget(fsWidget)
        # lat.update(value_chain[x]) # get latitude from packet
        # lon.update(value_chain[y]) # get long from packet
        data_base.guardar(value_chain)
    except IndexError:
        print('starting, please wait a moment')

if(ser.isOpen()):
    input("press once ready")
    ser.write("g")
    timer = pg.QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(100)
else:
    print("something is wrong with the update call")
# Start Qt event loop unless running in interactive mode.

if __name__ == '__main__':
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()

