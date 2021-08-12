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

		self.is_started_pub = rospy.Publisher(self.topics['ur5_start'], Bool, queue_size=1)
		self.is_started = Bool()
		self.is_started.data = False
		self.is_started_pub.publish(self.is_started)


		self.is_stopped_pub = rospy.Publisher(self.topics['ur5_stop'], Bool, queue_size=1)
		self.is_stopped = Bool()
		self.is_stopped.data = False
		self.is_stopped_pub.publish(self.is_stopped)

		self.is_resumed_pub = rospy.Publisher(self.topics['ur5_resume'], Bool, queue_size=1)
		self.is_resumed = Bool()
		self.is_resumed.data = False
		self.is_resumed_pub.publish(self.is_started)

		self.is_homed_pub = rospy.Publisher(self.topics['ur5_home'], Bool, queue_size=1)
		self.is_homed = Bool()
		self.is_homed.data = False
		self.is_homed_pub.publish(self.is_homed)

		self.setupUi(self)
		self.connect_signals()

	def connect_signals(self):
		self.start.clicked.connect(self.start_btn_clicked)
		self.stop.clicked.connect(self.stop_btn_clicked)
		self.resume.clicked.connect(self.resume_btn_clicked)
		self.home.clicked.connect(self.home_btn_clicked)

	def start_btn_clicked(self):
		if self.is_homed.data:
			self.is_started.data = True
			self.is_homed.data= False
		else:
			print("Please HOME the manipulator")
		self.is_started_pub.publish(self.is_started)
		self.is_stopped_pub.publish(self.is_stopped)
		self.is_resumed_pub.publish(self.is_resumed)
		self.is_homed_pub.publish(self.is_homed)
	
		
	def stop_btn_clicked(self):
		if self.is_started.data or self.is_resumed.data:
			self.is_resumed.data = False
			self.is_started.data = False
			self.is_stopped.data = True
		else:
			print("Please START operation of the manipulator")
		self.is_started_pub.publish(self.is_started)
		self.is_stopped_pub.publish(self.is_stopped)
		self.is_resumed_pub.publish(self.is_resumed)
		self.is_homed_pub.publish(self.is_homed)
		self.traj_cancel_pub.publish(self.cancel_traj)
			
	def resume_btn_clicked(self):
		if self.is_stopped.data:
			self.is_resumed.data = True
			self.is_stopped.data = False
		else:
			print("Operation needs to be STOPPED to resume. Press START to begin a new trajectory execution")
		self.is_started_pub.publish(self.is_started)
		self.is_stopped_pub.publish(self.is_stopped)
		self.is_resumed_pub.publish(self.is_resumed)
		self.is_homed_pub.publish(self.is_homed)

	def home_btn_clicked(self):
		self.is_homed.data = True
		self.is_stopped.data = False
		self.is_started_pub.publish(self.is_started)
		self.is_stopped_pub.publish(self.is_stopped)
		self.is_resumed_pub.publish(self.is_resumed)
		self.is_homed_pub.publish(self.is_homed)



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