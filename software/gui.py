# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gui.ui'
#
# Created by: PyQt5 UI code generator 5.15.11
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(650, 734)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(10, 0, 631, 681))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.groupBox = QtWidgets.QGroupBox(self.gridLayoutWidget_2)
        self.groupBox.setMaximumSize(QtCore.QSize(16777215, 150))
        self.groupBox.setObjectName("groupBox")
        self.gridLayoutWidget_3 = QtWidgets.QWidget(self.groupBox)
        self.gridLayoutWidget_3.setGeometry(QtCore.QRect(10, 30, 572, 91))
        self.gridLayoutWidget_3.setObjectName("gridLayoutWidget_3")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.gridLayoutWidget_3)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.pushButton_4 = QtWidgets.QPushButton(self.gridLayoutWidget_3)
        self.pushButton_4.setObjectName("pushButton_4")
        self.gridLayout_3.addWidget(self.pushButton_4, 0, 2, 1, 1)
        self.pushButton_3 = QtWidgets.QPushButton(self.gridLayoutWidget_3)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout_3.addWidget(self.pushButton_3, 0, 1, 1, 1)
        self.line = QtWidgets.QFrame(self.gridLayoutWidget_3)
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.gridLayout_3.addWidget(self.line, 0, 5, 1, 1)
        self.radioButton = QtWidgets.QRadioButton(self.gridLayoutWidget_3)
        self.radioButton.setObjectName("radioButton")
        self.gridLayout_3.addWidget(self.radioButton, 0, 0, 1, 1)
        self.radioButton_2 = QtWidgets.QRadioButton(self.gridLayoutWidget_3)
        self.radioButton_2.setObjectName("radioButton_2")
        self.gridLayout_3.addWidget(self.radioButton_2, 0, 6, 1, 1)
        self.pushButton_6 = QtWidgets.QPushButton(self.gridLayoutWidget_3)
        self.pushButton_6.setObjectName("pushButton_6")
        self.gridLayout_3.addWidget(self.pushButton_6, 0, 7, 1, 1)
        self.pushButton_5 = QtWidgets.QPushButton(self.gridLayoutWidget_3)
        self.pushButton_5.setObjectName("pushButton_5")
        self.gridLayout_3.addWidget(self.pushButton_5, 0, 8, 1, 1)
        self.label = QtWidgets.QLabel(self.gridLayoutWidget_3)
        self.label.setObjectName("label")
        self.gridLayout_3.addWidget(self.label, 1, 0, 1, 1)
        self.comboBox = QtWidgets.QComboBox(self.gridLayoutWidget_3)
        self.comboBox.setObjectName("comboBox")
        self.gridLayout_3.addWidget(self.comboBox, 1, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.gridLayoutWidget_3)
        self.label_2.setObjectName("label_2")
        self.gridLayout_3.addWidget(self.label_2, 1, 6, 1, 1)
        self.comboBox_2 = QtWidgets.QComboBox(self.gridLayoutWidget_3)
        self.comboBox_2.setObjectName("comboBox_2")
        self.gridLayout_3.addWidget(self.comboBox_2, 1, 7, 1, 1)
        self.gridLayout_2.addWidget(self.groupBox, 0, 0, 1, 1)
        self.groupBox_3 = QtWidgets.QGroupBox(self.gridLayoutWidget_2)
        self.groupBox_3.setMinimumSize(QtCore.QSize(0, 320))
        self.groupBox_3.setObjectName("groupBox_3")
        self.gridLayoutWidget = QtWidgets.QWidget(self.groupBox_3)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(9, 29, 611, 286))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.checkBox_3 = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.checkBox_3.setObjectName("checkBox_3")
        self.gridLayout.addWidget(self.checkBox_3, 3, 0, 1, 1)
        self.spinBox_2 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_2.setObjectName("spinBox_2")
        self.gridLayout.addWidget(self.spinBox_2, 1, 3, 1, 1)
        self.checkBox_7 = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.checkBox_7.setObjectName("checkBox_7")
        self.gridLayout.addWidget(self.checkBox_7, 7, 0, 1, 1)
        self.spinBox_7 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_7.setObjectName("spinBox_7")
        self.gridLayout.addWidget(self.spinBox_7, 6, 3, 1, 1)
        self.pushButton_16 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_16.setObjectName("pushButton_16")
        self.gridLayout.addWidget(self.pushButton_16, 5, 6, 1, 1)
        self.checkBox_5 = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.checkBox_5.setObjectName("checkBox_5")
        self.gridLayout.addWidget(self.checkBox_5, 5, 0, 1, 1)
        self.pushButton_13 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_13.setObjectName("pushButton_13")
        self.gridLayout.addWidget(self.pushButton_13, 2, 6, 1, 1)
        self.pushButton_7 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_7.setObjectName("pushButton_7")
        self.gridLayout.addWidget(self.pushButton_7, 2, 5, 1, 1)
        self.pushButton_12 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_12.setObjectName("pushButton_12")
        self.gridLayout.addWidget(self.pushButton_12, 7, 5, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 0, 0, 1, 1)
        self.spinBox_8 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_8.setObjectName("spinBox_8")
        self.gridLayout.addWidget(self.spinBox_8, 7, 3, 1, 1)
        self.pushButton_14 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_14.setObjectName("pushButton_14")
        self.gridLayout.addWidget(self.pushButton_14, 3, 6, 1, 1)
        self.spinBox_9 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_9.setObjectName("spinBox_9")
        self.gridLayout.addWidget(self.spinBox_9, 2, 4, 1, 1)
        self.pushButton_9 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_9.setObjectName("pushButton_9")
        self.gridLayout.addWidget(self.pushButton_9, 4, 5, 1, 1)
        self.spinBox_5 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_5.setObjectName("spinBox_5")
        self.gridLayout.addWidget(self.spinBox_5, 4, 3, 1, 1)
        self.pushButton_17 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_17.setObjectName("pushButton_17")
        self.gridLayout.addWidget(self.pushButton_17, 6, 6, 1, 1)
        self.checkBox_4 = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.checkBox_4.setObjectName("checkBox_4")
        self.gridLayout.addWidget(self.checkBox_4, 4, 0, 1, 1)
        self.checkBox = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.checkBox.setObjectName("checkBox")
        self.gridLayout.addWidget(self.checkBox, 1, 0, 1, 1)
        self.pushButton_10 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_10.setObjectName("pushButton_10")
        self.gridLayout.addWidget(self.pushButton_10, 5, 5, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_8.setObjectName("label_8")
        self.gridLayout.addWidget(self.label_8, 0, 6, 1, 1)
        self.pushButton = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout.addWidget(self.pushButton, 1, 6, 1, 1)
        self.spinBox_10 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_10.setObjectName("spinBox_10")
        self.gridLayout.addWidget(self.spinBox_10, 3, 4, 1, 1)
        self.spinBox_14 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_14.setObjectName("spinBox_14")
        self.gridLayout.addWidget(self.spinBox_14, 7, 4, 1, 1)
        self.checkBox_2 = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.checkBox_2.setObjectName("checkBox_2")
        self.gridLayout.addWidget(self.checkBox_2, 2, 0, 1, 1)
        self.spinBox_13 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_13.setObjectName("spinBox_13")
        self.gridLayout.addWidget(self.spinBox_13, 6, 4, 1, 1)
        self.spinBox_4 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_4.setObjectName("spinBox_4")
        self.gridLayout.addWidget(self.spinBox_4, 3, 3, 1, 1)
        self.pushButton_8 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_8.setObjectName("pushButton_8")
        self.gridLayout.addWidget(self.pushButton_8, 3, 5, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_7.setObjectName("label_7")
        self.gridLayout.addWidget(self.label_7, 0, 5, 1, 1)
        self.checkBox_6 = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.checkBox_6.setObjectName("checkBox_6")
        self.gridLayout.addWidget(self.checkBox_6, 6, 0, 1, 1)
        self.spinBox_11 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_11.setObjectName("spinBox_11")
        self.gridLayout.addWidget(self.spinBox_11, 4, 4, 1, 1)
        self.pushButton_15 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_15.setObjectName("pushButton_15")
        self.gridLayout.addWidget(self.pushButton_15, 4, 6, 1, 1)
        self.pushButton_18 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_18.setObjectName("pushButton_18")
        self.gridLayout.addWidget(self.pushButton_18, 7, 6, 1, 1)
        self.spinBox_3 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_3.setObjectName("spinBox_3")
        self.gridLayout.addWidget(self.spinBox_3, 2, 3, 1, 1)
        self.spinBox = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox.setObjectName("spinBox")
        self.gridLayout.addWidget(self.spinBox, 1, 4, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_6.setWordWrap(True)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 0, 4, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 1, 5, 1, 1)
        self.spinBox_12 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_12.setObjectName("spinBox_12")
        self.gridLayout.addWidget(self.spinBox_12, 5, 4, 1, 1)
        self.pushButton_11 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_11.setObjectName("pushButton_11")
        self.gridLayout.addWidget(self.pushButton_11, 6, 5, 1, 1)
        self.spinBox_6 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_6.setObjectName("spinBox_6")
        self.gridLayout.addWidget(self.spinBox_6, 5, 3, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_5.setWordWrap(True)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 0, 3, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_4.setWordWrap(True)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 0, 1, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_9.setWordWrap(True)
        self.label_9.setObjectName("label_9")
        self.gridLayout.addWidget(self.label_9, 0, 2, 1, 1)
        self.spinBox_15 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_15.setObjectName("spinBox_15")
        self.gridLayout.addWidget(self.spinBox_15, 1, 2, 1, 1)
        self.spinBox_16 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_16.setObjectName("spinBox_16")
        self.gridLayout.addWidget(self.spinBox_16, 2, 2, 1, 1)
        self.spinBox_17 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_17.setObjectName("spinBox_17")
        self.gridLayout.addWidget(self.spinBox_17, 3, 2, 1, 1)
        self.spinBox_18 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_18.setObjectName("spinBox_18")
        self.gridLayout.addWidget(self.spinBox_18, 4, 2, 1, 1)
        self.spinBox_19 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_19.setObjectName("spinBox_19")
        self.gridLayout.addWidget(self.spinBox_19, 5, 2, 1, 1)
        self.spinBox_20 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_20.setObjectName("spinBox_20")
        self.gridLayout.addWidget(self.spinBox_20, 6, 2, 1, 1)
        self.spinBox_21 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_21.setObjectName("spinBox_21")
        self.gridLayout.addWidget(self.spinBox_21, 7, 2, 1, 1)
        self.spinBox_22 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_22.setObjectName("spinBox_22")
        self.gridLayout.addWidget(self.spinBox_22, 1, 1, 1, 1)
        self.spinBox_23 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_23.setObjectName("spinBox_23")
        self.gridLayout.addWidget(self.spinBox_23, 2, 1, 1, 1)
        self.spinBox_24 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_24.setObjectName("spinBox_24")
        self.gridLayout.addWidget(self.spinBox_24, 3, 1, 1, 1)
        self.spinBox_25 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_25.setObjectName("spinBox_25")
        self.gridLayout.addWidget(self.spinBox_25, 4, 1, 1, 1)
        self.spinBox_26 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_26.setObjectName("spinBox_26")
        self.gridLayout.addWidget(self.spinBox_26, 5, 1, 1, 1)
        self.spinBox_27 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_27.setObjectName("spinBox_27")
        self.gridLayout.addWidget(self.spinBox_27, 6, 1, 1, 1)
        self.spinBox_28 = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.spinBox_28.setObjectName("spinBox_28")
        self.gridLayout.addWidget(self.spinBox_28, 7, 1, 1, 1)
        self.gridLayout_2.addWidget(self.groupBox_3, 1, 0, 1, 1)
        self.groupBox_2 = QtWidgets.QGroupBox(self.gridLayoutWidget_2)
        self.groupBox_2.setMinimumSize(QtCore.QSize(0, 150))
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.groupBox_2)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(9, 29, 611, 161))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setSizeConstraint(QtWidgets.QLayout.SetMaximumSize)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.verticalLayoutWidget)
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.verticalLayout.addWidget(self.plainTextEdit)
        self.gridLayout_2.addWidget(self.groupBox_2, 2, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 650, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox.setTitle(_translate("MainWindow", "Control select"))
        self.pushButton_4.setText(_translate("MainWindow", "Disconnet"))
        self.pushButton_3.setText(_translate("MainWindow", "Connect"))
        self.radioButton.setText(_translate("MainWindow", "USB"))
        self.radioButton_2.setText(_translate("MainWindow", "MQTT"))
        self.pushButton_6.setText(_translate("MainWindow", "Connect"))
        self.pushButton_5.setText(_translate("MainWindow", "Disconnet"))
        self.label.setText(_translate("MainWindow", "Select port"))
        self.label_2.setText(_translate("MainWindow", "Select Dev ID"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Setup parameters"))
        self.checkBox_3.setText(_translate("MainWindow", "Channel 3"))
        self.checkBox_7.setText(_translate("MainWindow", "All"))
        self.pushButton_16.setText(_translate("MainWindow", "Trigger"))
        self.checkBox_5.setText(_translate("MainWindow", "Channel 5"))
        self.pushButton_13.setText(_translate("MainWindow", "Trigger"))
        self.pushButton_7.setText(_translate("MainWindow", "Set"))
        self.pushButton_12.setText(_translate("MainWindow", "Set"))
        self.label_3.setText(_translate("MainWindow", "Channel"))
        self.pushButton_14.setText(_translate("MainWindow", "Trigger"))
        self.pushButton_9.setText(_translate("MainWindow", "Set"))
        self.pushButton_17.setText(_translate("MainWindow", "Trigger"))
        self.checkBox_4.setText(_translate("MainWindow", "Channel 4"))
        self.checkBox.setText(_translate("MainWindow", "Channel 1"))
        self.pushButton_10.setText(_translate("MainWindow", "Set"))
        self.label_8.setText(_translate("MainWindow", "Trigger"))
        self.pushButton.setText(_translate("MainWindow", "Trigger"))
        self.checkBox_2.setText(_translate("MainWindow", "Channel 2"))
        self.pushButton_8.setText(_translate("MainWindow", "Set"))
        self.label_7.setText(_translate("MainWindow", "Set"))
        self.checkBox_6.setText(_translate("MainWindow", "Channel 6"))
        self.pushButton_15.setText(_translate("MainWindow", "Trigger"))
        self.pushButton_18.setText(_translate("MainWindow", "Trigger"))
        self.label_6.setText(_translate("MainWindow", "Amplitude (Step)"))
        self.pushButton_2.setText(_translate("MainWindow", "Set"))
        self.pushButton_11.setText(_translate("MainWindow", "Set"))
        self.label_5.setText(_translate("MainWindow", "duty (%)"))
        self.label_4.setText(_translate("MainWindow", "Frequency (kHz)"))
        self.label_9.setText(_translate("MainWindow", "Duration (us)"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Log"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
