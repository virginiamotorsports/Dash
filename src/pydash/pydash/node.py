from xmlrpc.client import Boolean
import rclpy
import sys
import rclpy.qos
from threading import Thread
from rclpy.node import Node

from std_msgs.msg import Float32,Float64,Int32MultiArray,Int32,String,Bool
from dash_msgs.msg import DashReport

from pydash.node import RPMGauge
from PyQt5.QtWidgets import QApplication, QWidget

class GuiNode(Node):
    def __init__(self, gui):
        super().__init__("qt_gui_node")
        self.gui = gui

        sub_list = [("dash_report", DashReport)]

        for topic,topic_type in sub_list:
            self.create_subscription(topic_type, topic, lambda msg,topic=topic: self.gui.receive_msg(topic,msg), qos_profile=qos)

def spin_node(gui):
    node = GuiNode(gui)
    timeout = 1.0 # [s]
    while node.gui.running:
        rclpy.spin_once(node,timeout_sec=timeout)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    gui = RPMGauge()
    gui.show()
    sys.exit(app.exec_())

    spinner_thread = Thread(target=spin_node,args=(gui,))
    spinner_thread.setDaemon(1)
    spinner_thread.start()
    # gui.run()

if __name__ == '__main__':
    main()
