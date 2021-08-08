#!/usr/bin/python3

# UI imports
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow

# ROS imports
import rospy
from std_msgs.msg import Bool

# misc imports
import sys
import signal


def signal_handler():
	QApplication.quit()
	sys.exit(0)

# Generated usigng pyuic5 >>>>>>>>>>>>>
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
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

class StopResumeExe(QMainWindow, Ui_main_window):
	"""
	Publishes the resume/stop commands input by the user.
	"""

	def __init__(self, parent=None):
		super().__init__(parent)
		rospy.init_node('stop_resume_exe_gui', anonymous=True)
		self.ur5_resumed_pub = rospy.Publisher('/is_ur5_resumed', Bool, queue_size=1)

		self.is_resumed = Bool()
		self.is_resumed.data = False
		self.ur5_resumed_pub.publish(self.is_resumed)

		self.setupUi(self)
		self.connect_signals()

	def connect_signals(self):
		self.main_btn.clicked.connect(self.main_btn_clicked)

	def main_btn_clicked(self):
		if self.is_resumed.data:
			self.main_btn.setText("RESUME")
			self.is_resumed.data = False
		else:
			self.main_btn.setText("STOP")
			self.is_resumed.data = True
		
		self.ur5_resumed_pub.publish(self.is_resumed)

if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal_handler)
	signal.signal(signal.SIGTERM, signal_handler)

	app = QtWidgets.QApplication(sys.argv)
	win = StopResumeExe()

	# keep on checking for CTRL+C every 100 ms
	timer = QTimer()
	timer.start(100)
	timer.timeout.connect(lambda : None)

	win.show()
	app.exec_()