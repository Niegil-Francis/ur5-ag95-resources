#!/usr/bin/python3

# UI imports
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow

# ROS imports
import rospy
from rospkg import RosPack
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID

# misc imports
import sys
import signal
import argparse
import yaml


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

	def __init__(self, only_sim, parent=None):
		super().__init__(parent)
		rospy.init_node('stop_resume_exe_gui', anonymous=True)

		# initializing variables
		dir_path = RosPack().get_path('cobot-control') + "/scripts/stop_resume_exe/"
		
		# opening YAML file for topics
		with open(dir_path + "topics.yaml", 'r') as stream:
			try:
				self.topics = yaml.safe_load(stream)
			except yaml.YAMLError as exc:
				print(exc)
				print("Exiting because some issues with loading YAML")
				sys.exit(0)

		if only_sim:
			self.traj_cancel_pub = rospy.Publisher(self.topics['sim_traj_cancel'], GoalID, queue_size=1)
		else:
			self.traj_cancel_pub = rospy.Publisher(self.topics['hardware_traj_cancel'], GoalID, queue_size=1)

		self.cancel_traj = GoalID()

		self.is_resumed_pub = rospy.Publisher(self.topics['ur5_resume'], Bool, queue_size=1)
		self.is_resumed = Bool()
		self.is_resumed.data = False
		self.is_resumed_pub.publish(self.is_resumed)

		self.setupUi(self)
		self.connect_signals()

	def connect_signals(self):
		self.main_btn.clicked.connect(self.main_btn_clicked)

	def main_btn_clicked(self):
		if self.is_resumed.data:
			self.main_btn.setText("RESUME")
			self.is_resumed.data = False
			self.traj_cancel_pub.publish(self.cancel_traj)
		else:
			self.main_btn.setText("STOP")
			self.is_resumed.data = True
		
		self.is_resumed_pub.publish(self.is_resumed)

if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal_handler)
	signal.signal(signal.SIGTERM, signal_handler)

	# Create the parser
	my_parser = argparse.ArgumentParser(description="A GUI interface to stop/resume UR5's operation")

	# Add the arguments
	my_parser.add_argument('-s','--onlysim',
						action='store_true',
						help='Put this flag to use only simulation')

	# Execute the parse_args() method
	args, _ = my_parser.parse_known_args()

	app = QtWidgets.QApplication(sys.argv)
	win = StopResumeExe(only_sim=args.onlysim)

	# keep on checking for CTRL+C every 100 ms
	timer = QTimer()
	timer.start(100)
	timer.timeout.connect(lambda : None)

	win.show()
	app.exec_()