# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'plantMap.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.
import random
import time

from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np


class Ui_plantMap(object):
    def setupUi(self, plantMap):
        plantMap.setObjectName("plantMap")
        plantMap.resize(800, 480)
        self.centralwidget = QtWidgets.QWidget(plantMap)
        self.centralwidget.setObjectName("centralwidget")

        # set up printing box
        self.printWindow = QtWidgets.QTextBrowser(self.centralwidget)
        self.printWindow.setGeometry(QtCore.QRect(500, 0, 300, 480))
        self.printWindow.setObjectName("printWindow")

        # first column
        self.hole1_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole1_1.setGeometry(QtCore.QRect(50, 15, 16, 17))
        self.hole1_1.setText("")
        self.hole1_1.setObjectName("hole1_1")
        self.hole2_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole2_1.setGeometry(QtCore.QRect(50, 60, 16, 17))
        self.hole2_1.setText("")
        self.hole2_1.setObjectName("hole2_1")
        self.hole3_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole3_1.setGeometry(QtCore.QRect(50, 105, 16, 17))
        self.hole3_1.setText("")
        self.hole3_1.setObjectName("hole3_1")
        self.hole4_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole4_1.setGeometry(QtCore.QRect(50, 150, 16, 17))
        self.hole4_1.setText("")
        self.hole4_1.setObjectName("hole4_1")
        self.hole5_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole5_1.setGeometry(QtCore.QRect(50, 195, 16, 17))
        self.hole5_1.setText("")
        self.hole5_1.setObjectName("hole5_1")
        self.hole6_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole6_1.setGeometry(QtCore.QRect(50, 240, 16, 17))
        self.hole6_1.setText("")
        self.hole6_1.setObjectName("hole6_1")
        self.hole7_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole7_1.setGeometry(QtCore.QRect(50, 285, 16, 17))
        self.hole7_1.setText("")
        self.hole7_1.setObjectName("hole7_1")
        self.hole8_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole8_1.setGeometry(QtCore.QRect(50, 330, 16, 17))
        self.hole8_1.setText("")
        self.hole8_1.setObjectName("hole8_1")
        self.hole9_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole9_1.setGeometry(QtCore.QRect(50, 375, 16, 17))
        self.hole9_1.setText("")
        self.hole9_1.setObjectName("hole9_1")
        self.hole10_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole10_1.setGeometry(QtCore.QRect(50, 420, 16, 17))
        self.hole10_1.setText("")
        self.hole10_1.setObjectName("hole10_1")
        self.hole11_1 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole11_1.setGeometry(QtCore.QRect(50, 465, 16, 17))
        self.hole11_1.setText("")
        self.hole11_1.setObjectName("hole11_1")
        # second column
        self.hole1_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole1_2.setGeometry(QtCore.QRect(200, 15, 16, 17))
        self.hole1_2.setText("")
        self.hole1_2.setObjectName("hole1_2")
        self.hole2_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole2_2.setGeometry(QtCore.QRect(200, 60, 16, 17))
        self.hole2_2.setText("")
        self.hole2_2.setObjectName("hole2_2")
        self.hole3_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole3_2.setGeometry(QtCore.QRect(200, 105, 16, 17))
        self.hole3_2.setText("")
        self.hole3_2.setObjectName("hole3_2")
        self.hole4_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole4_2.setGeometry(QtCore.QRect(200, 150, 16, 17))
        self.hole4_2.setText("")
        self.hole4_2.setObjectName("hole4_2")
        self.hole5_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole5_2.setGeometry(QtCore.QRect(200, 195, 16, 17))
        self.hole5_2.setText("")
        self.hole5_2.setObjectName("hole5_2")
        self.hole6_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole6_2.setGeometry(QtCore.QRect(200, 240, 16, 17))
        self.hole6_2.setText("")
        self.hole6_2.setObjectName("hole6_2")
        self.hole7_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole7_2.setGeometry(QtCore.QRect(200, 285, 16, 17))
        self.hole7_2.setText("")
        self.hole7_2.setObjectName("hole7_2")
        self.hole8_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole8_2.setGeometry(QtCore.QRect(200, 330, 16, 17))
        self.hole8_2.setText("")
        self.hole8_2.setObjectName("hole8_2")
        self.hole9_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole9_2.setGeometry(QtCore.QRect(200, 375, 16, 17))
        self.hole9_2.setText("")
        self.hole9_2.setObjectName("hole9_2")
        self.hole10_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole10_2.setGeometry(QtCore.QRect(200, 420, 16, 17))
        self.hole10_2.setText("")
        self.hole10_2.setObjectName("hole10_2")
        self.hole11_2 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole11_2.setGeometry(QtCore.QRect(200, 465, 16, 17))
        self.hole11_2.setText("")
        self.hole11_2.setObjectName("hole11_2")

        # column 3
        self.hole1_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole1_3.setGeometry(QtCore.QRect(350, 15, 16, 17))
        self.hole1_3.setText("")
        self.hole1_3.setObjectName("hole1_3")
        self.hole2_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole2_3.setGeometry(QtCore.QRect(350, 60, 16, 17))
        self.hole2_3.setText("")
        self.hole2_3.setObjectName("hole2_3")
        self.hole3_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole3_3.setGeometry(QtCore.QRect(350, 105, 16, 17))
        self.hole3_3.setText("")
        self.hole3_3.setObjectName("hole3_3")
        self.hole4_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole4_3.setGeometry(QtCore.QRect(350, 150, 16, 17))
        self.hole4_3.setText("")
        self.hole4_3.setObjectName("hole4_3")
        self.hole5_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole5_3.setGeometry(QtCore.QRect(350, 195, 16, 17))
        self.hole5_3.setText("")
        self.hole5_3.setObjectName("hole5_3")
        self.hole6_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole6_3.setGeometry(QtCore.QRect(350, 240, 16, 17))
        self.hole6_3.setText("")
        self.hole6_3.setObjectName("hole6_3")
        self.hole7_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole7_3.setGeometry(QtCore.QRect(350, 285, 16, 17))
        self.hole7_3.setText("")
        self.hole7_3.setObjectName("hole7_3")
        self.hole8_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole8_3.setGeometry(QtCore.QRect(350, 330, 16, 17))
        self.hole8_3.setText("")
        self.hole8_3.setObjectName("hole8_3")
        self.hole9_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole9_3.setGeometry(QtCore.QRect(350, 375, 16, 17))
        self.hole9_3.setText("")
        self.hole9_3.setObjectName("hole9_3")
        self.hole10_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole10_3.setGeometry(QtCore.QRect(350, 420, 16, 17))
        self.hole10_3.setText("")
        self.hole10_3.setObjectName("hole10_3")
        self.hole11_3 = QtWidgets.QRadioButton(self.centralwidget)
        self.hole11_3.setGeometry(QtCore.QRect(350, 465, 16, 17))
        self.hole11_3.setText("")
        self.hole11_3.setObjectName("hole11_3")

        plantMap.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(plantMap)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 500, 480))
        self.menubar.setObjectName("menubar")
        plantMap.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(plantMap)
        self.statusbar.setObjectName("statusbar")
        plantMap.setStatusBar(self.statusbar)

        self.retranslateUi(plantMap)
        QtCore.QMetaObject.connectSlotsByName(plantMap)

    def retranslateUi(self, plantMap):
        _translate = QtCore.QCoreApplication.translate
        plantMap.setWindowTitle(_translate("plantMap", "MainWindow"))


class plantInfo(object):
    def createNameArray(self):
        name_array1 = []
        name_array2 = []
        name_array3 = []
        for y in range(1, 12):
            name_array1.append("hole" + str(int(y)) + "_1")
            name_array2.append("hole" + str(int(y)) + "_2")
            name_array3.append("hole" + str(int(y)) + "_3")

            name_array = np.array((name_array1, name_array2, name_array3))
        return name_array

    def coordinateConvertor(self, name_folder, column, row, color_num, init_info, height_num, direction_index, distance_num):
        if column is not init_info[0] or row is not init_info[1]:  # new hole
            if color_num == 1 or color_num == 2:  # a new plant
                init_info[0] = column
                init_info[1] = row
                init_info[2] += 1
                if color_num == 2:
                    init_info[3] = 1
        else:  # old plant
            if color_num == 2:  # new boll
                init_info[3] += 1

        if direction_index == 1:
            direction = 'Left'
        elif direction_index == 2:
            direction = 'Right'

        name = getattr(ui, name_folder[column - 1][row - 1])
        if color_num == 0:
            name.setStyleSheet("QRadioButton::indicator" "{""background-color : black""}")
        elif color_num == 1:
            name.setStyleSheet("QRadioButton::indicator" "{""background-color : green""}")
            ui.printWindow.append("Plant " + str(init_info[2]) + "\nHeight: N/A\nDirection: "
                                                                 "N/A\nDirection:N/A\nDistance: N/A\n")
        elif color_num == 2:
            name.setStyleSheet("QRadioButton::indicator" "{""background-color : red""}")
            ui.printWindow.append("Plant " + str(init_info[2]) + "\nBoll " + str(init_info[3]) + "\nHeight: " + str(
                height_num) + "\nDirection: " + direction + "\nDistance: " + str(distance_num) + "\n")

        return init_info


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    plantMap = QtWidgets.QMainWindow()
    ui = Ui_plantMap()
    info = plantInfo()
    ui.setupUi(plantMap)
    plantMap.show()
    name_array = info.createNameArray()
    init_info = [0, 0, 0, 0]  # [row, column, plant number, boll number]
    #  test
    init_info = info.coordinateConvertor(name_array, 1, 1, 2, init_info, 1.3, 1, 2.3)
    print(init_info)
    init_info = info.coordinateConvertor(name_array, 1, 1, 2, init_info, 1, 1, 2)
    print(init_info)
    init_info = info.coordinateConvertor(name_array, 1, 2, 1, init_info, 1, 1, 2)
    init_info = info.coordinateConvertor(name_array, 1, 3, 0, init_info, 1.6, 1, 2)
    init_info = info.coordinateConvertor(name_array, 1, 11, 2, init_info, 1.3, 1, 2)
    init_info = info.coordinateConvertor(name_array, 3, 4, 1, init_info, 1.5, 1, 2)
    init_info = info.coordinateConvertor(name_array, 2, 5, 0, init_info, 1, 1, 2)
    init_info = info.coordinateConvertor(name_array, 2, 4, 2, init_info, 1.7, 1, 2)
    init_info = info.coordinateConvertor(name_array, 3, 4, 2, init_info, 1.8, 1, 2)
    init_info = info.coordinateConvertor(name_array, 3, 1, 2, init_info, 13, 1, 2)
    # end of test
    sys.exit(app.exec_())