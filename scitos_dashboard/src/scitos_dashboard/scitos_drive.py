from python_qt_binding.QtCore import QSize

from rqt_robot_dashboard.widgets import MenuDashWidget


class ScitosDrive(MenuDashWidget):
    def __init__(self, reset_motorstop_cb, freerun_cb, enable_motorstop_cb):
        ok_icon = ['bg-green.svg', 'ic-motors.svg']
        free_run_icon = ['bg-yellow.svg', 'ic-motors.svg']
        free_run_icon_halted = ['bg-yellow.svg', 'ic-motors.svg', 'ol-warn-badge.svg']
        err_icon = ['bg-red.svg', 'ic-motors.svg', 'ol-err-badge.svg']
        stale_icon = ['bg-grey.svg', 'ic-motors.svg', 'ol-stale-badge.svg']
        hardstop_icon = ['bg-red.svg', 'ic-hardstop.svg', 'ol-stale-badge.svg']

        icons = [ok_icon, free_run_icon, free_run_icon_halted, err_icon, stale_icon, hardstop_icon]
        super(ScitosDrive, self).__init__('Drive', icons, icon_paths=[['scitos_dashboard', 'images']])
        self.update_state(5)

        self.add_action('Reset Motors Stop', reset_motorstop_cb)
        self.add_action('Toggle Free Run', freerun_cb)
        self.add_action('Halt', enable_motorstop_cb)
        self.setToolTip('Drive System')

        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))

    def set_ok(self):
        self.update_state(0)

    def set_free_run(self, halted):
        if halted:
            self.update_state(2)
        else:
            self.update_state(1)

    def set_stopped(self):
        self.update_state(3)

    def set_stale(self):
        self.update_state(4)

    def set_hardstop(self):
        self.update_state(5)
        
