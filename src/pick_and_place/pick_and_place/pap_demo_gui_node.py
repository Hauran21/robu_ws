import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from PyQt5 import QtCore, QtGui, QtWidgets

class PAPDemo(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        # Publisher für Pumpe 
        self._pub_vacuum_grip_ok = self.create_publisher(Bool, 
                                                         "vacuum/grip_ok", 
                                                         10)
        
        # Publisher für Lichtschranke
        self._pub_light_barrier_detected = self.create_publisher(Bool, 
                                                                 "light_barrier/detected", 
                                                                 10)
        
        # Service für das Fließband
        self._srv_conveyor_set_running = self.create_service(SetBool, 
                                                               "conveyor/set_running", 
                                                               self._srv_conveyor_set_running_cb)

    def set_vacuum_pump(self, value=True):
        msg = Bool()
        msg.data = value
        self._pub_vacuum_grip_ok.publish(msg)
        
    def set_light_barrier_detected(self, value=True):
        msg = Bool()
        msg.data = value
        self._pub_light_barrier_detected.publish(msg)

    def _srv_conveyor_set_running_cb(self, req:SetBool.Request, resp:SetBool.Response) -> SetBool.Response:
        if req.data: #Starte Maching will den Motor einschalten
            self.get_logger().info("Motor wurde aktiviert")
        else: 
            self.get_logger().info("Motor wurde ausgeschalten")
        
        resp.success = True
        return resp
    
    def destroy_node(self):
        return super().destroy_node()


class PAPGUI(QtWidgets.QWidget):
    """
    Qt-GUI
    """

    def __init__(self, node: PAPDemo):
        super().__init__()
        self.node = node

        self.setWindowTitle("Pick and Place Demo")
        self.resize(400, 100)

        self.btn_light_barrier_detected = QtWidgets.QPushButton("Lichtschranke unterbrechen")
        self.btn_vacuum = QtWidgets.QPushButton("Objekt ansaugen")
        self.btn_quit = QtWidgets.QPushButton("Beenden")

        layout_button = QtWidgets.QVBoxLayout()
        
        layout_button.addWidget(self.btn_light_barrier_detected)
        layout_button.addWidget(self.btn_vacuum)    
        layout_button.addWidget(self.btn_quit)

        self.setLayout(layout_button)
        
        self.btn_light_barrier_detected.setCheckable(True)
        self.btn_vacuum.setCheckable(True)

        self.btn_light_barrier_detected.clicked.connect(self.on_light_barrier_detected_clicked)
        self.btn_vacuum.clicked.connect(self.on_vacuum_clicked)
        self.btn_quit.clicked.connect(self.close)

    def on_light_barrier_detected_clicked(self):
        if self.btn_light_barrier_detected.isChecked():
            self.node.set_light_barrier_detected(True)
            self.btn_light_barrier_detected.setText("Lichtschranke freigeben")
        else:
            self.node.set_light_barrier_detected(False)
            self.btn_light_barrier_detected.setText("Lichtschranke unterbrechen")
            
    def on_vacuum_clicked(self):
        if self.btn_vacuum.isChecked():
            self.node.set_vacuum_pump(True)
            self.btn_vacuum.setText("Objekt loslassen")
        else:
            self.node.set_vacuum_pump(False)
            self.btn_vacuum.setText("Objekt ansaugen")
        
        #PAPDemo.set_vacuum_pump(self,True)
        
        """Dummy-Button gedrückt."""
        # self.node.get_logger().info("Dummy-Button gedrückt!")
        # msgbox = QtWidgets.QMessageBox(self)
        # msgbox.setWindowTitle("Dummy")
        # msgbox.setText("Du hast mich geklickt!")
        # msgbox.show()

    def closeEvent(self, event):
        if rclpy.ok():
            self.node.get_logger().info("GUI wird geschlossen, ROS wird heruntergefahren...")
            rclpy.shutdown()
        event.accept()


def main():
    node = None
    executor = None
    spin_thread = None

    try:
        rclpy.init()

        node = PAPDemo("PAPDemo")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Thread-Funktion für ROS-Spin
        def ros_spin():
            try:
                executor.spin()
            finally:
                executor.shutdown()
                node.destroy_node()

        # Hintergrund-Thread starten
        spin_thread = threading.Thread(target=ros_spin, daemon=True)
        spin_thread.start()

        # Qt-App im Hauptthread laufen lassen
        app = QtWidgets.QApplication(sys.argv)
        window = PAPGUI(node)
        window.show()
        exit_code = app.exec_()

        # Wenn Qt beendet → sicherstellen, dass ROS auch aus ist
        if rclpy.ok():
            rclpy.shutdown()

        # Auf Thread warten (kurz)
        if spin_thread.is_alive():
            spin_thread.join(timeout=1.0)

        sys.exit(exit_code)

    except KeyboardInterrupt:
        print("STRG+C gedrückt")
        if rclpy.ok():
            rclpy.shutdown()

    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()