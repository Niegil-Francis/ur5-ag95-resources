# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'stop_resume_exe.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_main_window(object):
    def setupUi(self, main_window):
        main_window.setObjectName("main_window")
        main_window.resize(625, 477)
        main_window.setStyleSheet("background-color: rgb(0, 0, 0);")
        self.centralwidget = QtWidgets.QWidget(main_window)
        self.centralwidget.setObjectName("centralwidget")
        self.start = QtWidgets.QPushButton(self.centralwidget)
        self.start.setGeometry(QtCore.QRect(30, 60, 251, 141))
        font = QtGui.QFont()
        font.setFamily("Tibetan Machine Uni")
        font.setPointSize(26)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.start.setFont(font)
        self.start.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.start.setAutoFillBackground(False)
        self.start.setStyleSheet("background-color: rgb(138, 226, 52);")
        self.start.setObjectName("start")
        self.stop = QtWidgets.QPushButton(self.centralwidget)
        self.stop.setGeometry(QtCore.QRect(340, 60, 251, 141))
        font = QtGui.QFont()
        font.setFamily("Tibetan Machine Uni")
        font.setPointSize(26)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.stop.setFont(font)
        self.stop.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.stop.setAutoFillBackground(False)
        self.stop.setStyleSheet("background-color: rgb(239, 41, 41);")
        self.stop.setObjectName("stop")
        self.resume = QtWidgets.QPushButton(self.centralwidget)
        self.resume.setGeometry(QtCore.QRect(30, 260, 251, 141))
        font = QtGui.QFont()
        font.setFamily("Tibetan Machine Uni")
        font.setPointSize(26)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.resume.setFont(font)
        self.resume.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.resume.setAutoFillBackground(False)
        self.resume.setStyleSheet("background-color: rgb(252, 233, 79);")
        self.resume.setObjectName("resume")
        self.home = QtWidgets.QPushButton(self.centralwidget)
        self.home.setGeometry(QtCore.QRect(340, 260, 251, 141))
        font = QtGui.QFont()
        font.setFamily("Tibetan Machine Uni")
        font.setPointSize(26)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.home.setFont(font)
        self.home.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.home.setAutoFillBackground(False)
        self.home.setStyleSheet("background-color: rgb(114, 159, 207);")
        self.home.setObjectName("home")
        main_window.setCentralWidget(self.centralwidget)

        self.retranslateUi(main_window)
        QtCore.QMetaObject.connectSlotsByName(main_window)

    def retranslateUi(self, main_window):
        _translate = QtCore.QCoreApplication.translate
        main_window.setWindowTitle(_translate("main_window", "UR5 Control | Start-Stop-Resume-Home"))
        self.start.setText(_translate("main_window", "START"))
        self.stop.setText(_translate("main_window", "STOP"))
        self.resume.setText(_translate("main_window", "RESUME"))
        self.home.setText(_translate("main_window", "HOME"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main_window = QtWidgets.QMainWindow()
    ui = Ui_main_window()
    ui.setupUi(main_window)
    main_window.show()
    sys.exit(app.exec_())
