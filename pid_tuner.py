import sys
import serial
import serial.tools.list_ports
import threading
import time
from collections import deque
import pyqtgraph as pg
import json
import os
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QComboBox,
    QVBoxLayout,
    QHBoxLayout,
    QSlider,
    QDoubleSpinBox,
    QGroupBox,
    QFormLayout
)
from PyQt5.QtCore import Qt, QTimer

class PIDTuner(QWidget):

    def __init__(self):

        super().__init__()

        self.serial_port = None
        self.running = False

        self.target_rpm = 0
        self.left_rpm = 0
        self.right_rpm = 0

        self.left_pwm = 0
        self.right_pwm = 0
        self.time_ms = 0

        self.left_count = 0
        self.right_count = 0

        self.time_data = deque(maxlen=300)

        self.target_data = deque(maxlen=300)

        self.left_rpm_data = deque(maxlen=300)

        self.right_rpm_data = deque(maxlen=300)

       
        self.initUI()


    def initUI(self):
        self.setWindowTitle("DC Motor PID Tuner")

        self.resize(1300,900)


        # COM Port Selection

        self.port_label = QLabel("COM Port")

        self.port_combo = QComboBox()

        self.refreshPorts()


        self.connect_button = QPushButton("Connect")

        self.connect_button.clicked.connect(self.connectArduino)


        self.status_label = QLabel("Disconnected")

        self.reset_button = QPushButton("Reset Graph")

        self.reset_button.clicked.connect(self.resetGraph)  
        # Top Layout

        top_layout = QHBoxLayout()

        top_layout.addWidget(self.port_label)

        top_layout.addWidget(self.port_combo)

        top_layout.addWidget(self.connect_button)
        top_layout.addWidget(self.reset_button)
        top_layout.addWidget(self.status_label)


         # Main Layout
 
        main_layout = QVBoxLayout()

        main_layout.addLayout(top_layout)

        # Main area
        content_layout = QHBoxLayout()

        # Left panel (controls)
        left_layout = QVBoxLayout()

        # Right panel (graphs)
        right_layout = QVBoxLayout()

        self.createPIDControls(left_layout)
        self.createRPMGraph(right_layout)

        content_layout.addLayout(left_layout, 1)
        content_layout.addLayout(right_layout, 3)

        main_layout.addLayout(content_layout)

        self.setLayout(main_layout)
        self.loadSettings() 
        self.timer = QTimer()

        self.timer.timeout.connect(self.updateDisplay)

        self.timer.start(50)

    def refreshPorts(self):

        self.port_combo.clear()

        ports = serial.tools.list_ports.comports()

        for port in ports:

            self.port_combo.addItem(port.device)

    def createPIDControls(self, layout):

         # LEFT MOTOR
 
        layout.addWidget(QLabel("LEFT MOTOR"))

        self.left_kp_label = QLabel("Left Kp : 0.80")
        self.left_kp_spin = QDoubleSpinBox()

        self.left_kp_spin.setDecimals(4)

        self.left_kp_spin.setRange(0.0000,10.0000)

        self.left_kp_spin.setSingleStep(0.0001)

        self.left_kp_spin.setValue(0.8000)

        self.left_kp_spin.editingFinished.connect(self.sendLeftKp)
        self.left_kp_spin.editingFinished.connect(self.saveSettings)

        layout.addWidget(self.left_kp_label)
        layout.addWidget(self.left_kp_spin)


        self.left_ki_label = QLabel("Left Ki : 0.00")
        self.left_ki_spin = QDoubleSpinBox()

        self.left_ki_spin.setDecimals(4)

        self.left_ki_spin.setRange(0.0000,5.0000)

        self.left_ki_spin.setSingleStep(0.0001)

        self.left_ki_spin.setValue(0.0000)

        self.left_ki_spin.editingFinished.connect(self.sendLeftKi)
        self.left_ki_spin.editingFinished.connect(self.saveSettings)

        layout.addWidget(self.left_ki_label)
        layout.addWidget(self.left_ki_spin)


        self.left_kd_label = QLabel("Left Kd : 0.00")
        self.left_kd_spin = QDoubleSpinBox()

        self.left_kd_spin.setDecimals(4)

        self.left_kd_spin.setRange(0.0000,5.0000)

        self.left_kd_spin.setSingleStep(0.0001)

        self.left_kd_spin.setValue(0.0)

        self.left_kd_spin.editingFinished.connect(self.sendLeftKd)
        self.left_kd_spin.editingFinished.connect(self.saveSettings)

        layout.addWidget(self.left_kd_label)
        layout.addWidget(self.left_kd_spin)


         # RIGHT MOTOR
 
        layout.addWidget(QLabel("RIGHT MOTOR"))

        self.right_kp_label = QLabel("Right Kp : 0.80")
        self.right_kp_spin = QDoubleSpinBox()

        self.right_kp_spin.setDecimals(4)

        self.right_kp_spin.setRange(0.0000,10.0000)

        self.right_kp_spin.setSingleStep(0.0001)

        self.right_kp_spin.setValue(0.8000)

        self.right_kp_spin.editingFinished.connect(self.sendRightKp)
        self.right_kp_spin.editingFinished.connect(self.saveSettings)
        layout.addWidget(self.right_kp_label)
        layout.addWidget(self.right_kp_spin)


        self.right_ki_label = QLabel("Right Ki : 0.00")
        self.right_ki_spin = QDoubleSpinBox()

        self.right_ki_spin.setDecimals(4)

        self.right_ki_spin.setRange(0.0000,5.0000)

        self.right_ki_spin.setSingleStep(0.0001)

        self.right_ki_spin.setValue(0.0)

        self.right_ki_spin.editingFinished.connect(self.sendRightKi)
        self.right_ki_spin.editingFinished.connect(self.saveSettings)

        layout.addWidget(self.right_ki_label)
        layout.addWidget(self.right_ki_spin)


        self.right_kd_label = QLabel("Right Kd : 0.00")
        self.right_kd_spin = QDoubleSpinBox()

        self.right_kd_spin.setDecimals(4)

        self.right_kd_spin.setRange(0.0000,5.0000)

        self.right_kd_spin.setSingleStep(0.0001)

        self.right_kd_spin.setValue(0.0)

        self.right_kd_spin.editingFinished.connect(self.sendRightKd)
        self.right_kd_spin.editingFinished.connect(self.saveSettings)

        layout.addWidget(self.right_kd_label)
        layout.addWidget(self.right_kd_spin)


         # Target RPM
 
        self.rpm_label = QLabel("Target RPM : 0")

        self.rpm_slider = QSlider(Qt.Horizontal)
        self.rpm_slider.setRange(0,300)

        self.rpm_slider.valueChanged.connect(self.sendRPM)
        self.rpm_slider.sliderReleased.connect(self.saveSettings)

        layout.addWidget(self.rpm_label)
        layout.addWidget(self.rpm_slider)


         # Live Values
 
        self.targetValue = QLabel("Target RPM : 0")
        self.leftRPMValue = QLabel("Left RPM : 0")
        self.rightRPMValue = QLabel("Right RPM : 0")
        self.leftPWMValue = QLabel("Left PWM : 0")
        self.rightPWMValue = QLabel("Right PWM : 0")

        layout.addWidget(self.targetValue)
        layout.addWidget(self.leftRPMValue)
        layout.addWidget(self.rightRPMValue)
        layout.addWidget(self.leftPWMValue)
        layout.addWidget(self.rightPWMValue)

    def createRPMGraph(self, layout):

        self.rpm_plot = pg.PlotWidget()

        self.rpm_plot.setTitle("Motor RPM")

        self.rpm_plot.setLabel("left","RPM")

        self.rpm_plot.setLabel("bottom","Samples")

        self.rpm_plot.showGrid(x=True,y=True)

        self.rpm_plot.addLegend()
        self.rpm_plot.enableAutoRange(False)

        self.rpm_plot.setXRange(0,300)

        self.rpm_plot.setYRange(0,100)

        self.target_curve = self.rpm_plot.plot(
            pen=pg.mkPen('b',width=2),
            name="Target"
        )

        self.left_curve = self.rpm_plot.plot(
            pen=pg.mkPen('r',width=2),
            name="Left"
        )

        self.right_curve = self.rpm_plot.plot(
            pen=pg.mkPen('g',width=2),
            name="Right"
        )

        layout.addWidget(self.rpm_plot)

    # def connectArduino(self):

    #     try:

    #         self.serial_port = serial.Serial(
    #             self.port_combo.currentText(),
    #             115200,
    #             timeout=0.1
    #         )
    #         time.sleep(2)
    #         self.running = True

    #         threading.Thread(
    #             target=self.readSerial,
    #             daemon=True
    #         ).start()

    #         self.status_label.setText("Connected")
    #         self.connect_button.setText("Disconnect")
    #         self.sendLeftKp()
    #         self.sendLeftKi()
    #         self.sendLeftKd()

    #         self.sendRightKp()
    #         self.sendRightKi()
    #         self.sendRightKd()

    #         self.sendRPM()
    #     except Exception as e:

    #         self.status_label.setText(str(e))
    
    def connectArduino(self):

         # Disconnect
         if self.serial_port is not None:

            self.running = False

            time.sleep(0.2)      # Give the read thread time to exit

            self.serial_port.close()

            self.serial_port = None

            self.status_label.setText("Disconnected")

            self.connect_button.setText("Connect")

            return

         # Connect
         try:

            self.serial_port = serial.Serial(
                self.port_combo.currentText(),
                115200,
                timeout=0.1
            )

            time.sleep(2)

            self.running = True

            threading.Thread(
                target=self.readSerial,
                daemon=True
            ).start()

            self.status_label.setText("Connected")

            self.connect_button.setText("Disconnect")

            # Send all current settings to Arduino
            self.sendLeftKp()
            self.sendLeftKi()
            self.sendLeftKd()

            self.sendRightKp()
            self.sendRightKi()
            self.sendRightKd()

            self.sendRPM()

        except Exception as e:

            self.status_label.setText(str(e))

    def readSerial(self):

        while self.running:

            try:

                if self.serial_port.in_waiting:

                    line = self.serial_port.readline().decode(errors="ignore").strip()

                    print("Received :", line)
                    # Ignore empty lines
                    if not line:
                        continue

                    # Status messages
                    if line.startswith("#"):
                        print(line)
                        continue

                    # Telemetry
                    if line.startswith("DATA"):

                        parts = line.split(",")

                        if len(parts) != 9:
                            continue

                        self.time_ms = int(parts[1])

                        self.target_rpm = float(parts[2])
                        self.left_rpm = float(parts[3])
                        self.right_rpm = float(parts[4])

                        self.left_pwm = int(parts[5])
                        self.right_pwm = int(parts[6])

                        self.left_count = int(parts[7])
                        self.right_count = int(parts[8])
                        self.time_data.append(self.time_ms)

                        self.target_data.append(self.target_rpm)

                        self.left_rpm_data.append(self.left_rpm)

                        self.right_rpm_data.append(self.right_rpm)

            except Exception as e:

                if self.running:
                    print(e)

                break
            
    def sendLeftKp(self):

        # value = self.left_kp_slider.value() / 100.0
        value = self.left_kp_spin.value()   
        self.left_kp_label.setText(f"Left Kp : {value:.4f}")
        self.resetGraph()   
        if self.serial_port:
            self.serial_port.write(f"LP,{value:.4f}\n".encode())
            self.serial_port.flush()


    def sendLeftKi(self):

        # value = self.left_ki_slider.value() / 100.0
        value = self.left_ki_spin.value()   

        self.left_ki_label.setText(f"Left Ki : {value:.4f}")
        self.resetGraph()   

        if self.serial_port:
            self.serial_port.write(f"LI,{value:.4f}\n".encode())
            self.serial_port.flush()


    def sendLeftKd(self):

        # value = self.left_kd_slider.value() / 100.0
        value = self.left_kd_spin.value()   

        self.left_kd_label.setText(f"Left Kd : {value:.4f}")
        self.resetGraph()   

        if self.serial_port:
            self.serial_port.write(f"LD,{value:.4f}\n".encode())
            self.serial_port.flush()


    def sendRightKp(self):

        # value = self.right_kp_slider.value() / 100.0
        value = self.right_kp_spin.value()   

        self.right_kp_label.setText(f"Right Kp : {value:.4f}")
        self.resetGraph()   

        if self.serial_port:
            self.serial_port.write(f"RP,{value:.4f}\n".encode())
            self.serial_port.flush()


    def sendRightKi(self):

        # value = self.right_ki_slider.value() / 100.0
        value = self.right_ki_spin.value()   

        self.right_ki_label.setText(f"Right Ki : {value:.4f}")
        self.resetGraph()   

        if self.serial_port:
            self.serial_port.write(f"RI,{value:.4f}\n".encode())
            self.serial_port.flush()


    def sendRightKd(self):

        # value = self.right_kd_slider.value() / 100.0
        value = self.right_kd_spin.value()   

        self.right_kd_label.setText(f"Right Kd : {value:.4f}")
        self.resetGraph()   

        if self.serial_port:
            self.serial_port.write(f"RD,{value:.4f}\n".encode())
            self.serial_port.flush()


    def sendRPM(self):

        value = self.rpm_slider.value()

        self.rpm_label.setText(f"Target RPM : {value}")
        self.resetGraph()   

        if self.serial_port:
            self.serial_port.write(f"R,{value}\n".encode())
            self.serial_port.flush()

    def updateDisplay(self):

        self.targetValue.setText(
            f"Target RPM : {self.target_rpm:.1f}"
        )

        self.leftRPMValue.setText(
            f"Left RPM : {self.left_rpm:.1f}"
        )

        self.rightRPMValue.setText(
            f"Right RPM : {self.right_rpm:.1f}"
        )

        self.leftPWMValue.setText(
            f"Left PWM : {self.left_pwm}"
        )

        self.rightPWMValue.setText(
            f"Right PWM : {self.right_pwm}"
        )
        # x = list(range(len(self.target_data)))
        x = range(len(self.target_data))
        self.target_curve.setData(
            x,
            list(self.target_data)
        )

        self.left_curve.setData(
            x,
            list(self.left_rpm_data)
        )

        self.right_curve.setData(
            x,
            list(self.right_rpm_data)
)
        print("Number of points:", len(self.target_data))

    def resetGraph(self):

        self.time_data.clear()

        self.target_data.clear()

        self.left_rpm_data.clear()

        self.right_rpm_data.clear()

        self.target_curve.clear()

        self.left_curve.clear()

        self.right_curve.clear()

        print("===== Graph Reset =====")

    def saveSettings(self):

        settings = {
            "left_kp": self.left_kp_spin.value(),
            "left_ki": self.left_ki_spin.value(),
            "left_kd": self.left_kd_spin.value(),

            "right_kp": self.right_kp_spin.value(),
            "right_ki": self.right_ki_spin.value(),
            "right_kd": self.right_kd_spin.value(),

            "target_rpm": self.rpm_slider.value()
        }

        with open("pid_settings.json", "w") as f:
            json.dump(settings, f, indent=4)

    def loadSettings(self):

        if not os.path.exists("pid_settings.json"):
            return

        with open("pid_settings.json", "r") as f:
            settings = json.load(f)

        # Load values into widgets
        self.left_kp_spin.setValue(settings.get("left_kp", 0.8))
        self.left_ki_spin.setValue(settings.get("left_ki", 0.0))
        self.left_kd_spin.setValue(settings.get("left_kd", 0.0))

        self.right_kp_spin.setValue(settings.get("right_kp", 0.8))
        self.right_ki_spin.setValue(settings.get("right_ki", 0.0))
        self.right_kd_spin.setValue(settings.get("right_kd", 0.0))

        self.rpm_slider.setValue(settings.get("target_rpm", 0))

        self.left_kp_label.setText(f"Left Kp : {self.left_kp_spin.value():.4f}")
        self.left_ki_label.setText(f"Left Ki : {self.left_ki_spin.value():.4f}")
        self.left_kd_label.setText(f"Left Kd : {self.left_kd_spin.value():.4f}")

        self.right_kp_label.setText(f"Right Kp : {self.right_kp_spin.value():.4f}")
        self.right_ki_label.setText(f"Right Ki : {self.right_ki_spin.value():.4f}")
        self.right_kd_label.setText(f"Right Kd : {self.right_kd_spin.value():.4f}")

        self.rpm_label.setText(f"Target RPM : {self.rpm_slider.value()}")

        self.sendLeftKp()
        self.sendLeftKi()
        self.sendLeftKd()

        self.sendRightKp()
        self.sendRightKi()
        self.sendRightKd()

        self.sendRPM()

if __name__ == "__main__":

    app = QApplication(sys.argv)

    window = PIDTuner()

    window.show()

    sys.exit(app.exec_())