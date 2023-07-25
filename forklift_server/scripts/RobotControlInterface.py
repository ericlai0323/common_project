import sys
import rospy
from PyQt5.QtWidgets import (
    QComboBox, QGridLayout, QAction, QHBoxLayout, QApplication, QMainWindow,
    QVBoxLayout, QWidget, QLabel, QPushButton
)
from PyQt5.QtGui import QFont  # Corrected import statement
from PyQt5 import QtCore

import math
from nav_msgs.msg import Odometry
import tf2_ros
import tf
from apriltag_ros.msg import AprilTagDetectionArray

class RobotControl(QMainWindow):
    def __init__(self):
        super().__init__()

        self.wheelodom_position_x = 0.0
        self.wheelodom_position_y = 0.0
        self.wheelodom_orientation_z = 0.0
        self.wheelodom_orientation_w = 0.0

        self.marker_2d_distance = 0.0
        self.marker_2d_offset = 0.0
        self.marker_2d_theta = 0.0

        self.initROS()
        self.initUI()

    def initROS(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        sub_wheelodom = rospy.Subscriber('/wheel_odom', Odometry, self.callback_wheelodom)
        self.sub_info_marker = rospy.Subscriber('/tag_detections_up', AprilTagDetectionArray, self.callback_GetMarker_up, queue_size = 1)
        self.sub_info_marker = rospy.Subscriber('/tag_detections_down', AprilTagDetectionArray, self.callback_GetMarker_down, queue_size = 1)

    def initUI(self):
        self.setWindowTitle('Robot Contorl Interface')
        self.setGeometry(100, 100, 1280, 720)

        menubar = self.menuBar()
        mode_menu = menubar.addMenu('Mode')

        auto_mode_action = QAction('Auto Mode', self)
        auto_mode_action.triggered.connect(self.switchToAutoMode)
        mode_menu.addAction(auto_mode_action)

        manual_mode_action = QAction('Manual Mode', self)
        manual_mode_action.triggered.connect(self.switchToManualMode)
        mode_menu.addAction(manual_mode_action)

        self.main_layout = QHBoxLayout()

        robot_info_widget = self.createRobotInfoWidget()
        self.main_layout.addWidget(robot_info_widget)

        self.control_widget = self.createAutoControlWidget()
        self.main_layout.addWidget(self.control_widget)

        main_widget = QWidget()
        main_widget.setLayout(self.main_layout)

        self.setCentralWidget(main_widget)

        self.show()

    def createAutoControlWidget(self):
        auto_control_widget = QWidget()
        layout = QVBoxLayout()

        button_autolaunch = QPushButton('Auto Launch')
        layout.addWidget(button_autolaunch)

        button_autolaunch.clicked.connect(self.button_autolaunchClicked)

        auto_control_widget.setLayout(layout)
        return auto_control_widget
    
    def createManualControlWidget(self):
        manual_control_widget = QWidget()
        layout = QVBoxLayout()

        layout_navigation = QGridLayout()

        label_navigation = QLabel('Navigation')
        label_navigation.setFont(QFont("Ubuntu Medium", 14, QFont.Bold))
        button_A_VS = QPushButton('Visual Servoing Point A')
        button_B_VS = QPushButton('Visual Servoing Point B')

        layout_visualservoing = QGridLayout()

        label_visualservoing = QLabel('Visual Servoing')
        label_visualservoing.setFont(QFont("Ubuntu Medium", 14, QFont.Bold))
        button_apriltagon = QPushButton('AprilTag ON')
        button_apriltagoff = QPushButton('AprilTag OFF')

        button_approachapriltag = QPushButton('Approach AprilTag')
        button_raisepallet = QPushButton('Raise Pallet')
        button_droppallet = QPushButton('Drop Pallet')
        
        layout_navigation.addWidget(label_navigation,0,0)
        layout_navigation.addWidget(button_A_VS, 1, 0)
        layout_navigation.addWidget(button_B_VS, 1, 1)

        layout_visualservoing.addWidget(label_visualservoing, 0, 0)
        layout_visualservoing.addWidget(button_apriltagon, 1, 0)
        layout_visualservoing.addWidget(button_apriltagoff, 1, 1)
        layout_visualservoing.addWidget(button_approachapriltag, 2,0)
        layout_visualservoing.addWidget(button_raisepallet, 3, 0)
        layout_visualservoing.addWidget(button_droppallet, 3, 1)

        layout_navigation.setAlignment(QtCore.Qt.AlignTop)
        layout_visualservoing.setAlignment(QtCore.Qt.AlignTop)
        layout.addLayout(layout_navigation)
        layout.addLayout(layout_visualservoing)

        button_approachapriltag.clicked.connect(self.button_approachapriltagClicked)
        button_raisepallet.clicked.connect(self.button_raisepalletClicked)
        button_droppallet.clicked.connect(self.button_droppalletClicked)

        manual_control_widget.setLayout(layout)
        return manual_control_widget
    
    def createRobotInfoWidget(self):
        robot_info_widget = QWidget()
        layout = QVBoxLayout()

        # info_label = QLabel('INFO')
        robot_status_label = QLabel('Robot Status')
        robot_status_label.setFont(QFont("Ubuntu Medium", 14, QFont.Bold))
        self.robot_status_onoff = QLabel('ON/OFF: ')
        self.robot_status_mode_label = QLabel('Mode: ')


        apriltag_status_label = QLabel('AprilTag Status')
        apriltag_status_label.setFont(QFont("Ubuntu Medium", 14, QFont.Bold))
        self.apriltag_status_onoff_label = QLabel('ON/OFF: ')
        self.apriltag_distance_label = QLabel('Distance: ')
        self.apriltag_offset_label = QLabel('Offset: ')
        self.apriltag_angle_label = QLabel('Angle: ')

        robot_position_label = QLabel('Robot Position')
        robot_position_label.setFont(QFont("Ubuntu Medium", 14, QFont.Bold))
        self.robot_position_x_label = QLabel('X: ')
        self.robot_position_y_label = QLabel('Y: ')
        self.robot_orientation_z_label = QLabel('Z: ')
        self.robot_orientation_w_label = QLabel('W: ')

        robot_odometry_label = QLabel('Robot Odometry')
        robot_odometry_label.setFont(QFont("Ubuntu Medium", 14, QFont.Bold))
        self.odometry_position_x_label = QLabel('X: ')
        self.odometry_position_y_label = QLabel('Y: ')
        self.odometry_orientation_z_label = QLabel('Z: ')
        self.odometry_orientation_w_label = QLabel('W: ')

        robot_status_layout = QVBoxLayout()
        robot_status_layout.addWidget(robot_status_label)
        robot_status_layout.addWidget(self.robot_status_onoff)
        robot_status_layout.addWidget(self.robot_status_mode_label)
        
        apriltag_status_layout = QVBoxLayout()
        apriltag_status_layout.addWidget(apriltag_status_label)
        apriltag_status_layout.addWidget(self.apriltag_status_onoff_label)
        apriltag_status_layout.addWidget(self.apriltag_distance_label)
        apriltag_status_layout.addWidget(self.apriltag_offset_label)
        apriltag_status_layout.addWidget(self.apriltag_angle_label)

        robot_position_layout = QVBoxLayout()  # Add parentheses here
        robot_position_layout.addWidget(robot_position_label)
        robot_position_layout.addWidget(self.robot_position_x_label)
        robot_position_layout.addWidget(self.robot_position_y_label)
        robot_position_layout.addWidget(self.robot_orientation_z_label)
        robot_position_layout.addWidget(self.robot_orientation_w_label)

        robot_odometry_layout = QVBoxLayout()
        robot_odometry_layout.addWidget(robot_odometry_label)
        robot_odometry_layout.addWidget(self.odometry_position_x_label)
        robot_odometry_layout.addWidget(self.odometry_position_y_label)
        robot_odometry_layout.addWidget(self.odometry_orientation_z_label)
        robot_odometry_layout.addWidget(self.odometry_orientation_w_label)


        # 將文字對齊到上方
        # info_label.setAlignment(QtCore.Qt.AlignTop)
        # robot_status_label.setAlignment(QtCore.Qt.AlignTop)
        # apriltag_status_label.setAlignment(QtCore.Qt.AlignTop)
        # robot_position_label.setAlignment(QtCore.Qt.AlignTop)
        # robot_odometry_label.setAlignment(QtCore.Qt.AlignTop)

        # 將按鈕設置為對齊上方
        # robot_labels_layout = QVBoxLayout()
        # robot_labels_layout.addWidget(robot_status_label)
        # robot_labels_layout.addWidget(apriltag_status_label)
        # robot_labels_layout.addWidget(robot_position_label)
        # robot_labels_layout.addWidget(robot_odometry_label)

        # layout.addWidget(info_label)
        layout.addLayout(robot_status_layout,0)
        layout.addLayout(apriltag_status_layout)
        layout.addLayout(robot_position_layout)
        layout.addLayout(robot_odometry_layout)

        robot_info_widget.setLayout(layout)
        return robot_info_widget
    
    def callback_GetMarker_up(self, msg):
        try:
            # print("up tag")
            marker_msg = msg.detections[0].pose.pose.pose
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            # theta = self.ekf_theta.update(theta)
            self.marker_2d_distance = -marker_msg.position.z
            self.marker_2d_offset = marker_msg.position.x
            self.marker_2d_theta = -theta

        except:
            pass

    def callback_GetMarker_down(self, msg):
        try:
            # print("down tag")
            marker_msg = msg.detections[0].pose.pose.pose
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            # theta = self.ekf_theta.update(theta)
            self.marker_2d_distance = -marker_msg.position.z
            self.marker_2d_offset = marker_msg.position.x
            self.marker_2d_theta = -theta
        except:
            pass

    def callback_wheelodom(self, msg):
        self.wheelodom_position_x = msg.pose.pose.position.x
        self.wheelodom_position_y = msg.pose.pose.position.y
        self.wheelodom_orientation_z = msg.pose.pose.orientation.z
        self.wheelodom_orientation_w = msg.pose.pose.orientation.w

    def updateRobotOdometryLabel(self):
        self.odometry_position_x_label.setText(f'X: {self.wheelodom_position_x:.2f}')
        self.odometry_position_y_label.setText(f'Y: {self.wheelodom_position_y:.2f}')
        self.odometry_orientation_z_label.setText(f'Z: {self.wheelodom_orientation_z:.2f}')
        self.odometry_orientation_w_label.setText(f'W: {self.wheelodom_orientation_w:.2f}')

    def updateAprilTagLabel(self):
        self.apriltag_distance_label.setText(f'Distance: {self.marker_2d_distance:.2f}')
        self.apriltag_offset_label.setText(f'Offset: {self.marker_2d_offset:.2f}')
        self.apriltag_angle_label.setText(f'Angle: {math.degrees(self.marker_2d_theta):.2f}')

    def updateRobotPositionLabel(self):
        try:
            trans = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))

            self.robot_position_x_label.setText(f'X: {trans.transform.translation.x:.2f}')
            self.robot_position_y_label.setText(f'Y: {trans.transform.translation.y:.2f}')

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def updateLabels(self):
        self.updateRobotPositionLabel()
        self.updateRobotOdometryLabel()
        self.updateAprilTagLabel()
        # Add other label updates if needed.



    def switchToAutoMode(self):
        self.main_layout.removeWidget(self.control_widget)
        self.control_widget.deleteLater()
        self.control_widget = self.createAutoControlWidget()
        self.main_layout.addWidget(self.control_widget)

    def switchToManualMode(self):
        self.main_layout.removeWidget(self.control_widget)
        self.control_widget.deleteLater()
        self.control_widget = self.createManualControlWidget()
        self.main_layout.addWidget(self.control_widget)

    def button_approachapriltagClicked(self):
        print("Approach AprilTag")

    def button_raisepalletClicked(self):
        print("Raise Pallet")

    def button_droppalletClicked(self):
        print("Drop Pallet")

    def button_autolaunchClicked(self):
        print("Auto Launch")

if __name__ == '__main__':
    # 初始化ROS節點
    rospy.init_node('robot_control_app', anonymous=True)

    # 創建QT應用程序
    app = QApplication(sys.argv)

    # 創建主界面
    main_window = RobotControl()

    # Create a timer to update the labels periodically
    timer = QtCore.QTimer()
    timer.timeout.connect(main_window.updateLabels)
    timer.start(2000)  # Update every 1000 milliseconds (1 second)


    # rospy.spin()
    # 運行QT應用程序
    sys.exit(app.exec_())
