import rospy

from python_qt_binding.QtCore import QSize
from rqt_robot_dashboard.widgets import BatteryDashWidget


class ScitosBattery(BatteryDashWidget):
    def __init__(self):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        super(ScitosBattery, self).__init__('Scitos Battery')

        self._power_consumption = 0.0
        self._pct = 0
        self._time_remaining = rospy.rostime.Duration(0)
        self._ac_present = 0
        self._plugged_in = False

        self.setFixedSize(self._icons[1].actualSize(QSize(50, 30)))

        self.update_perc(0)

    def set_power_state(self, msg):
        """
        Sets button state based on msg

        :param msg: message containing the power state of the Scitos
        :type msg: scitos_msgs.BatteryState
        """
        last_pct = self._pct
        last_plugged_in = self._plugged_in

        self._pct = msg.lifePercent
        self._plugged_in = msg.powerSupplyPresent

        if (last_pct != self._pct or last_plugged_in != self._plugged_in):
            drain_str = "not charging"
            if (self._plugged_in):
                drain_str = "charging"
            self.setToolTip("Battery: %.2f%% (%s)" % (self._pct, drain_str))

            self.set_charging( self._plugged_in )

            self.update_perc(msg.lifePercent)

    def set_stale(self):
        self._plugged_in = False
        self._pct = 0

        self.setToolTip("Battery: Stale")
