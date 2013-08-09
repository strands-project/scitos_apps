import rospy

from python_qt_binding.QtGui import QLabel
from rqt_robot_dashboard.widgets import BatteryDashWidget


class ScitosMileage(QLabel):
    def __init__(self):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        super(ScitosMileage, self).__init__('Scitos Mileage')


    def set_mileage_from_msg(self, msg):
        """
        Sets mileage based on msg

        :param msg: message containing the mileage,  of the Scitos
        :type msg: std_msgs::Float32
        """
        self.setText("%d m"%int(msg.data))

    def set_stale(self):
        self.setText("No mileage data!")
