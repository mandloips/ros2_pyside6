# gui.py
from PySide6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from PySide6.QtCore import QTimer, Qt
from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.figure import Figure

#class for canvas
class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)
        self.axes.set_ylabel('Linear Velocity (m/s)')

#class for gui window
class AppWindow(QWidget):
    def __init__(self, ros_gui_node, shutdown_event):
        super().__init__()

        self.shutdown_event = shutdown_event

        self.ros_gui_node = ros_gui_node
        self.setWindowTitle("GUI for turtlesim")
        self.setGeometry(100, 100, 800, 600)

        self.layout = QVBoxLayout(self)

        self.status = "x : " + str(self.ros_gui_node.x)

        self.button_status = "stopped"

        self.status_label = QLabel(str(self.ros_gui_node.x), self)
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.layout.addWidget(self.status_label)

        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        self.layout.addWidget(self.canvas)

        n_data = 50
        self.xdata = list(range(n_data))
        self.ydata = [0 for i in range(n_data)]

        self._plot_ref = None
        self.canvas.axes.set_ylim(0, 3)
        self.show()

        self.update_button = QPushButton("Start", self)
        self.update_button.clicked.connect(self.start_action)
        self.layout.addWidget(self.update_button)

        self.plot_button = QPushButton("Stop", self)
        self.plot_button.clicked.connect(self.stop_action)
        self.layout.addWidget(self.plot_button)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_value_plot)
        self.timer.start(100)

    #start button action
    def start_action(self):
        self.button_status = "started"
        self.ros_gui_node.start_ros_timer()

    #stop button action
    def stop_action(self):
        self.button_status = "stopped"
        self.ros_gui_node.stop_ros_timer()

    #update value and live plot
    def update_value_plot(self):
        self.button_label = "button status : " + str(self.button_status) + "\n"
        self.x_label = "x position : " + str(self.ros_gui_node.x) + "\n"
        self.y_label = "y position : " + str(self.ros_gui_node.y) + "\n"
        self.theta_lable = "theta : " + str(self.ros_gui_node.theta) + "\n"
        self.lv_label = "linear velocity : " + str(self.ros_gui_node.linear_velocity) + "\n"
        self.av_label = "angular velocity : " + str(self.ros_gui_node.angular_velocity) + "\n"
        self.updated_status = self.button_label + self.x_label + self.y_label + self.theta_lable + self.lv_label + self.av_label
        self.status_label.setText(self.updated_status)

        self.ydata = self.ydata[1:] + [self.ros_gui_node.linear_velocity]
        if self._plot_ref is None:
            plot_refs = self.canvas.axes.plot(self.xdata, self.ydata, 'r')
            self._plot_ref = plot_refs[0]
        else:
            self._plot_ref.set_ydata(self.ydata)
        self.canvas.draw()
        
    #shutdown event upon closing the GUI window
    def closeEvent(self, event):
        self.shutdown_event.set()
        event.accept()

