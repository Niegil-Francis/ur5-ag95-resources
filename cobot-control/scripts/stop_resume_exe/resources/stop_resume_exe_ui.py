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
        main_window.resize(399, 247)
        self.centralwidget = QtWidgets.QWidget(main_window)
        self.centralwidget.setObjectName("centralwidget")
        self.main_btn = QtWidgets.QPushButton(self.centralwidget)
        self.main_btn.setGeometry(QtCore.QRect(100, 70, 191, 101))
        self.main_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.main_btn.setObjectName("main_btn")
        main_window.setCentralWidget(self.centralwidget)

        self.retranslateUi(main_window)
        QtCore.QMetaObject.connectSlotsByName(main_window)

    def retranslateUi(self, main_window):
        _translate = QtCore.QCoreApplication.translate
        main_window.setWindowTitle(_translate("main_window", "UR5 Control | Stop-Resume"))
        self.main_btn.setText(_translate("main_window", "RESUME"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main_window = QtWidgets.QMainWindow()
    ui = Ui_main_window()
    ui.setupUi(main_window)
    main_window.show()
    sys.exit(app.exec_())
