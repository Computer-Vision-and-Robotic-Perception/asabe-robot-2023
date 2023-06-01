import random
import time
import sys

from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np

class ASABE_GUI(object):
    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.map = QtWidgets.QMainWindow()
        self.setupUi()
        self.map.show()
    
    def setupUi(self):
        self.map.setObjectName("plantMap")
        self.map.resize(800, 480)
        self.centralwidget = QtWidgets.QWidget(self.map)
        self.centralwidget.setObjectName("centralwidget")
        # Set up printing box
        self.printWindow = QtWidgets.QTextBrowser(self.centralwidget)
        self.printWindow.setGeometry(QtCore.QRect(481, 1, 319, 450))
        self.printWindow.setObjectName("printWindow")
        # Draw
        self.button = QtWidgets.QPushButton(self.centralwidget)
        self.button.setText('start')
        self.button.move(210, 1)
        self.draw_board()
        self.map.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(self.map)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 480, 480))
        self.menubar.setObjectName("menubar")
        self.map.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(self.map)
        self.statusbar.setObjectName("statusbar")
        self.map.setStatusBar(self.statusbar)
        self.map.setWindowTitle(QtCore.QCoreApplication.translate("plantMap", "MainWindow"))
        QtCore.QMetaObject.connectSlotsByName(self.map)

    def draw_board(self):
        self.holes = []
        for i in range(1, 4):
            row = []
            for j in range(1, 24):
                hole = QtWidgets.QRadioButton(self.centralwidget)
                hole.setGeometry(QtCore.QRect(20*j - 9, 120*i - 9, 18, 18))
                hole.setText("")
                hole.setObjectName("hole%d_%d" % (j, i))
                setattr(self, "hole%d_%d" % (j, i), hole)
                row.append(hole)
            self.holes.append(row)
        self.holes = np.array(self.holes)

#         self.painter = QtGui.QPainter(self.map)
#         self.painter.setPen(QtGui.QPen(QtCore.Qt.black, 5, QtCore.Qt.SolidLine))
#         self.painter.setBrush(QtGui.QBrush(QtCore.Qt.white, QtCore.Qt.SolidPattern))
#         for i in range(1, 4):
#             for j in range(1, 24):
#                 self.painter.drawEllipse(20*j - 8, 120*i - 8, 16, 16)

    def map_new(self, column, row, color_num, info, height_num, direction_index, distance_num):
        if column is not info[0] or row is not info[1]:  # new hole
            if color_num == 1 or color_num == 2:  # a new plant
                info[0] = column
                info[1] = row
                info[2] += 1
                if color_num == 2:
                    info[3] = 1
        else:                                     # old plant
            if color_num == 2:                    # new boll
                info[3] += 1

        if direction_index == 1:
            direction = 'Left'
        elif direction_index == 2:
            direction = 'Right'

        name = self.holes[column - 1, row - 1]
        if color_num == 0:
            name.setStyleSheet("QRadioButton::indicator" "{""background-color : black""}")
        elif color_num == 1:
            name.setStyleSheet("QRadioButton::indicator" "{""background-color : green""}")
            gui.printWindow.append("Plant " + str(info[2]) + "\nHeight: N/A\nDirection: "
                                                                 "N/A\nDirection:N/A\nDistance: N/A\n")
        elif color_num == 2:
            name.setStyleSheet("QRadioButton::indicator" "{""background-color : red""}")
            gui.printWindow.append("Plant " + str(info[2]) + "\nBoll " + str(info[3]) + "\nHeight: " + str(
                height_num) + "\nDirection: " + direction + "\nDistance: " + str(distance_num) + "\n")

        return info


if __name__ == "__main__": 
    gui = ASABE_GUI()
    info = [0, 0, 0, 0]  # [row, column, plant number, boll number]
    #  test
    info = gui.map_new(1, 1, 2, info, 1.3, 1, 2.3)
    info = gui.map_new(1, 1, 2, info, 1, 1, 2)
    info = gui.map_new(1, 2, 1, info, 1, 1, 2)
    info = gui.map_new(1, 3, 0, info, 1.6, 1, 2)
    info = gui.map_new(1, 11, 2, info, 1.3, 1, 2)
    info = gui.map_new(3, 4, 1, info, 1.5, 1, 2)
    info = gui.map_new(2, 5, 0, info, 1, 1, 2)
    info = gui.map_new(2, 4, 2, info, 1.7, 1, 2)
    info = gui.map_new(3, 4, 2, info, 1.8, 1, 2)
    info = gui.map_new(3, 1, 2, info, 13, 1, 2)
    # end of test
    print(gui.button.isChecked())
    sys.exit(gui.app.exec_())
