import roslib
roslib.load_manifest('scitos_dashboard')
import rospy

from scitos_msgs.msg import MotorStatus, BatteryState
from scitos_msgs.srv import EmergencyStop, ResetMotorStop, EnableMotors
from std_msgs.msg import Float32

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.monitor_dash_widget import MonitorDashWidget
from rqt_robot_dashboard.console_dash_widget import ConsoleDashWidget

from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QMessageBox

from scitos_drive import ScitosDrive
from scitos_battery import ScitosBattery
from scitos_mileage import ScitosMileage

from threading import Timer

class ScitosDashboard(Dashboard):
    def setup(self, context):
        self.name = 'Scitos Dashboard'
        self.max_icon_size = QSize(50, 30)
        self.message = None

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

        self._raw_byte = None
        self.digital_outs = [0, 0, 0]

        self._console = ConsoleDashWidget(self.context, minimal=False)
        self._monitor = MonitorDashWidget(self.context)
        self._mileage = ScitosMileage()

        self._drive = ScitosDrive(self.reset_motorstop_cb, self.freerun_cb, self.enable_motorstop_cb)

        self._batteries = ScitosBattery()
        self._freerun_state = True
        
        self._motor_state_sub = rospy.Subscriber('/motor_status', MotorStatus, self.motor_status_callback)
        self._battery_state_sub = rospy.Subscriber('/battery_state', BatteryState, self.battery_callback)
        self._mileage_sub = rospy.Subscriber('/mileage', Float32, self.mileage_callback)
        self._motor_stale_timer = Timer(0, self._drive.set_stale)
        self._motor_stale_timer.start()
        self._battery_stale_timer = Timer(0, self._batteries.set_stale)
        self._battery_stale_timer.start()
        self._mileage_stale_timer = Timer(0, self._mileage.set_stale)
        self._mileage_stale_timer.start()
        

    def get_widgets(self):
        return [[self._monitor, self._console], [self._drive],[self._batteries], [self._mileage]]

    def motor_status_callback(self, msg):
        self._motor_stale_timer.cancel()
        self._motor_stale_timer = Timer(1, self._drive.set_stale)
        self._motor_stale_timer.start()
        self._motorstatus_message = msg
        self._last_motorstatus_message_time = rospy.get_time()

        if msg.emergency_button_pressed:
            self._drive.set_hardstop()
        elif msg.normal:
            self._drive.set_ok()
        elif msg.free_run:
            self._freerun_state=msg.free_run
            self._drive.set_free_run(msg.motor_stopped)
        elif msg.motor_stopped:
            self._drive.set_stopped()

    def battery_callback(self,msg):
        self._battery_stale_timer.cancel()
        self._battery_stale_timer = Timer(3, self._batteries.set_stale)
        self._battery_stale_timer.start()
        self._batteries.set_power_state(msg)

        
    def mileage_callback(self, msg):
        self._mileage_stale_timer.cancel()
        self._mileage_stale_timer = Timer(1, self._mileage.set_stale)
        self._mileage_stale_timer.start()

        self._mileage.set_mileage_from_msg(msg)
        
    def reset_motorstop_cb(self):
        reset_srv = rospy.ServiceProxy("/reset_motorstop", ResetMotorStop)
        try:
            reset_srv()
        except rospy.ServiceException, e:
            QMessageBox.critical(self._drive, "Error", "Failed to halt the motors: service call failed with error: %s" % (e))

    def freerun_cb(self):
        self._freerun_state = not self._freerun_state
        freerun_srv = rospy.ServiceProxy("/enable_motors", EnableMotors)
        try:
            freerun_srv(not self._freerun_state)
        except rospy.ServiceException, e:
            QMessageBox.critical(self._drive, "Error", "Failed to enter freerun: service call failed with error: %s" % (e))

    def enable_motorstop_cb(self):
        stop_srv = rospy.ServiceProxy("/emergency_stop", EmergencyStop)
        try:
            stop_srv()
        except rospy.ServiceException, e:
            QMessageBox.critical(self._drive, "Error", "Failed to emergency stop: service call failed with error: %s" % (e))

    
    def shutdown_dashboard(self):
        self._motor_state_sub.unregister()
        self._battery_state_sub.unregister()
        
    def save_settings(self, plugin_settings, instance_settings):
        self._console.save_settings(plugin_settings, instance_settings)
        self._monitor.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._console.restore_settings(plugin_settings, instance_settings)
        self._monitor.restore_settings(plugin_settings, instance_settings)
