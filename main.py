import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QThread, pyqtSignal, QObject, QTimer
from PyQt5 import QtGui
from gui import Ui_MainWindow
import time, csv
from threading import Thread
import serial.tools.list_ports
import pyqtgraph as pg

from esp import ESP
from trajector_planning import Trajector

class SerialReceiverThread(QThread):
    # Tín hiệu để gửi dữ liệu nhận được từ luồng đến giao diện chính
    data_received = pyqtSignal(str)

    def __init__(self, serial_com):
        super().__init__()
        self.serial_com = serial_com  # Cổng serial mà ta đã kết nối
        self.is_running = True

    def run(self):
        while self.is_running:
            if self.serial_com and self.serial_com.in_waiting > 0:
                raw_data = self.serial_com.readline().strip()
                decoded_data = raw_data.decode("utf-8", errors="replace")
                print(decoded_data)
                if decoded_data:
                    self.data_received.emit(decoded_data)  # Phát tín hiệu khi có dữ liệu mới
    
    def stop(self):
        self.is_running = False
        self.wait()  # Đợi luồng dừng hẳn

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.main_win = QMainWindow()
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self.main_win)
        self.esp = ESP()
        self.TP = Trajector()

        # Đồ thị PyQtGraph 1
        self.uic.graphicsView.setBackground("w")
        self.curve = self.uic.graphicsView.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_line2 = self.uic.graphicsView.plot(pen=pg.mkPen(color='b', width=2))
        self.uic.graphicsView.setLabel('left', 'Angle (degree)')
        self.uic.graphicsView.setLabel('bottom', 'Time (ms)')
        legend = self.uic.graphicsView.addLegend()
        legend.addItem(self.curve, "Tín hiệu đặt")
        legend.addItem(self.curve_line2, "Tín hiệu thực tế")

        # Đồ thị PyQtGraph 2
        self.uic.graphicsView_2.setBackground("w")
        self.curve_2 = self.uic.graphicsView_2.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_2_line2 = self.uic.graphicsView_2.plot(pen=pg.mkPen(color='b', width=2))
        self.uic.graphicsView_2.setLabel('left', 'Force (Nm)')
        self.uic.graphicsView_2.setLabel('bottom', 'Time (ms)')
        legend_2 = self.uic.graphicsView_2.addLegend()
        legend_2.addItem(self.curve_2, "Tín hiệu đặt")
        legend_2.addItem(self.curve_2_line2, "Tín hiệu thực tế")

        # Đồ thị PyQtGraph 3
        self.uic.graphicsView_3.setBackground("w")
        self.curve_3 = self.uic.graphicsView_3.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_3_line2 = self.uic.graphicsView_3.plot(pen=pg.mkPen(color='b', width=2))
        self.uic.graphicsView_3.setLabel('left', 'Angle (degree)')
        self.uic.graphicsView_3.setLabel('bottom', 'Time (ms)')
        legend_3 = self.uic.graphicsView_3.addLegend()
        legend_3.addItem(self.curve_3, "Tín hiệu đặt")
        legend_3.addItem(self.curve_3_line2, "Tín hiệu thực tế")

        # Đồ thị PyQtGraph 4
        self.uic.graphicsView_8.setBackground("w")
        self.curve_4 = self.uic.graphicsView_8.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_4_line2 = self.uic.graphicsView_8.plot(pen=pg.mkPen(color='b', width=2))
        self.uic.graphicsView_8.setLabel('left', 'Force (Nm)')
        self.uic.graphicsView_8.setLabel('bottom', 'Time (ms)')
        legend_4 = self.uic.graphicsView_8.addLegend()
        legend_4.addItem(self.curve_4, "Tín hiệu đặt")
        legend_4.addItem(self.curve_4_line2, "Tín hiệu thực tế")

        # Đồ thị 5
        self.uic.graphicsView_4.setBackground("w")
        self.curve_5 = self.uic.graphicsView_4.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_5_line2 = self.uic.graphicsView_4.plot(pen=pg.mkPen(color='b', width=2))
        self.uic.graphicsView_4.setLabel('left', 'Distance (m)')
        self.uic.graphicsView_4.setLabel('bottom', 'Time (ms)')
        legend_5 = self.uic.graphicsView_4.addLegend()
        legend_5.addItem(self.curve_5, "Tín hiệu đặt")
        legend_5.addItem(self.curve_5_line2, "Tín hiệu thực tế")

        # Đồ thị 6
        self.uic.graphicsView_5.setBackground("w")
        self.curve_6 = self.uic.graphicsView_5.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_6_line2 = self.uic.graphicsView_5.plot(pen=pg.mkPen(color='b', width=2))
        self.uic.graphicsView_5.setLabel('left', 'Distance (m)')
        self.uic.graphicsView_5.setLabel('bottom', 'Time (ms)')
        legend_6 = self.uic.graphicsView_5.addLegend()
        legend_6.addItem(self.curve_6, "Tín hiệu đặt")
        legend_6.addItem(self.curve_6_line2, "Tín hiệu thực tế")

        # Đồ thị 7
        self.uic.graphicsView_6.setBackground("w")
        self.curve_7 = self.uic.graphicsView_6.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_7_line2 = self.uic.graphicsView_6.plot(pen=pg.mkPen(color='b', width=2))
        self.uic.graphicsView_6.setLabel('left', 'Distance (m)')
        self.uic.graphicsView_6.setLabel('bottom', 'Time (ms)')
        legend_7 = self.uic.graphicsView_6.addLegend()
        legend_7.addItem(self.curve_7, "Tín hiệu đặt")
        legend_7.addItem(self.curve_7_line2, "Tín hiệu thực tế")

        # Đồ thị 8
        self.uic.graphicsView_7.setBackground("w")
        self.curve_8 = self.uic.graphicsView_7.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_8_line2 = self.uic.graphicsView_7.plot(pen=pg.mkPen(color='b', width=2))
        self.uic.graphicsView_7.setLabel('left', 'Distance (m)')
        self.uic.graphicsView_7.setLabel('bottom', 'Time (ms)')
        legend_8 = self.uic.graphicsView_7.addLegend()
        legend_8.addItem(self.curve_8, "Tín hiệu đặt")
        legend_8.addItem(self.curve_8_line2, "Tín hiệu thực tế")

        # Đồ thị 9
        self.uic.graphicsView_12.setBackground("w")
        self.curve_9 = self.uic.graphicsView_12.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_9_line2 = self.uic.graphicsView_12.plot(pen=pg.mkPen(color='b', width=2))
        self.uic.graphicsView_12.setLabel('left', 'Value')
        self.uic.graphicsView_12.setLabel('bottom', 'Time (ms)')
        legend_9 = self.uic.graphicsView_12.addLegend()
        legend_9.addItem(self.curve_9, "Tín hiệu đặt")
        legend_9.addItem(self.curve_9_line2, "Tín hiệu thực tế")

        # Đồ thị 10
        self.uic.graphicsView_13.setBackground("w")
        self.curve_10 = self.uic.graphicsView_13.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_10_line2 = self.uic.graphicsView_13.plot(pen=pg.mkPen(color='b', width=2))
        self.uic.graphicsView_13.setLabel('left', 'Value')
        self.uic.graphicsView_13.setLabel('bottom', 'Time (ms)')
        legend_10 = self.uic.graphicsView_13.addLegend()
        legend_10.addItem(self.curve_10, "Tín hiệu đặt")
        legend_10.addItem(self.curve_10_line2, "Tín hiệu thực tế")
        
        self.COM_PORT = ""
        self.BAUD_RATE = 115200
        self.PORT_LIST, self.DRIVER_LIST = self.esp.get_com_port()
        self.serialCom = None
        self.data_graph = []
        self.data_graph_line2 = []
        self.data_graph_2 = []
        self.data_graph_2_line2 = []
        self.data_graph_3 = []
        self.data_graph_3_line2 = []
        self.data_graph_4 = []
        self.data_graph_4_line2 = []
        self.data_graph_5 = []
        self.data_graph_5_line2 = []
        self.data_graph_6 = []
        self.data_graph_6_line2 = []
        self.data_graph_7 = []
        self.data_graph_7_line2 = []
        self.data_graph_8 = []
        self.data_graph_8_line2 = []
        self.data_graph_9 = []
        self.data_graph_9_line2 = []

        self.time_graph = []
        self.time_graph_2 = []
        self.time_graph_3 = []
        self.time_graph_4 = []
        self.time_graph_5 = []
        self.time_graph_6 = []
        self.time_graph_7 = []
        self.time_graph_8 = []
        self.time_graph_9 = []
        self.mcu_process_time = 0

        self.angle_1 = 0
        self.angle_2 = 0

        self.position_1 = 0
        self.torque_1 = 0
        self.speed_1 = 0
        self.position_set_1 = 0
        self.temp_1 = 0
        self.torque_pid_1 = 0
        self.position_1 = 0
        self.torque_2 = 0
        self.speed_2 = 0
        self.position_set_2 = 0
        self.temp_2 = 0
        self.torque_pid_2 = 0

        self.csv_data = []

        # Threading
        self.is_running = False

        # Quy hoạch quỹ đạo
        self.point2point_lst = self.TP.point2point_operate()
        self.triangle_lst = self.TP.traingle_operate()

        self.uic.comboBox.addItems(self.PORT_LIST)
        self.uic.comboBox_2.addItems(["4800", "9600", "14400", "19200", "28800", "38400", "57600", "115200"])
        self.uic.comboBox_2.setCurrentText(str(self.BAUD_RATE))

        self.trajectory_lst = ["Point to point", "Triangle"]
        self.uic.comboBox_3.addItems(self.trajectory_lst)
        self.uic.comboBox_3.setCurrentText(self.trajectory_lst[0])

        self.uic.pushButton_6.clicked.connect(self.btn_refresh)
        self.uic.pushButton_3.clicked.connect(self.btn_connect)
        self.uic.pushButton_4.clicked.connect(self.btn_disconnect)
        self.uic.pushButton_7.clicked.connect(self.btn_send_pid_params)
        self.uic.pushButton.clicked.connect(self.btn_send_serial_monitor)
        self.uic.pushButton_2.clicked.connect(self.btn_clear_serial_monitor)
        self.uic.pushButton_8.clicked.connect(self.trajector_planning_start)
        self.uic.pushButton_9.clicked.connect(self.trajector_planning_stop)

        self.uic.actionOpen.triggered.connect(self.open_serial_monitor)
        self.uic.actionClose.triggered.connect(self.close_serial_monitor)
        self.uic.actionClear_Graph_1.triggered.connect(self.clear_graph_1)
        self.uic.actionClear_Graph_2.triggered.connect(self.clear_graph_2)
        self.uic.actionClear_Graph_3.triggered.connect(self.clear_graph_3)
        self.uic.actionClear_All.triggered.connect(self.clear_all)
        self.uic.actionExit.triggered.connect(self.exit_application)
        self.uic.actionGraph_3.triggered.connect(self.hide_graph_3)
        self.uic.actionUnhide_All.triggered.connect(self.unhide_all)
        self.uic.actionSave.triggered.connect(self.csv_save)
        self.uic.actionSave_as.triggered.connect(self.csv_save)
        self.uic.actionController.triggered.connect(self.show_page)
        self.uic.actionIndentification.triggered.connect(self.show_page_2)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)

        self.uic.stackedWidget.setCurrentWidget(self.uic.page)

########################################## Page 1 #################################################
    def trajector_planning_start(self):
        self.uic.comboBox_3.setStyleSheet("QComboBox"
                                     "{"
                                     "background-color: lightgreen;"
                                     "}")
        self.is_running = True
        try:
            self.thread1 = Thread(target=self.trajector_planning_operate)
            self.thread1.daemon = True
            self.thread1.start()
        except:
            print("Trajectory planning thread error")
            self.serial_monitor("Trajectory planning thread error")

    def trajector_planning_stop(self):
        self.uic.comboBox_3.setStyleSheet("QComboBox"
                                     "{"
                                     "background-color: light gray;"
                                     "}")
        self.is_running = False
        if self.thread1 is not None:
            self.thread1.join()  # Wait for the thread to finish
        
    def trajector_planning_operate(self):
        while self.is_running:
            kp_1 = self.uic.lineEdit_2.text()
            ki_1 = self.uic.lineEdit_3.text()
            kd_1 = self.uic.lineEdit_4.text()

            kp_2 = self.uic.lineEdit_5.text()
            ki_2 = self.uic.lineEdit_6.text()
            kd_2 = self.uic.lineEdit_7.text()

            for i in range(len(self.point2point_lst)):
                if self.uic.comboBox_3.currentText() == self.trajectory_lst[0]:
                    self.angle_2 = self.point2point_lst[i][1]*10*(-1)
                    self.angle_1 = self.point2point_lst[i][0]*10
                elif self.uic.comboBox_3.currentText() == self.trajectory_lst[1]:
                    self.angle_2 = self.triangle_lst[i][1]*10*(-1)
                    self.angle_1 = self.triangle_lst[i][0]*10

                pid = kp_1+","+ki_1+","+kd_1+","+str(self.angle_1)+","+kp_2+","+ki_2+","+kd_2+","+str(self.angle_2)

                if self.serialCom is not None:
                    try:
                        self.serialCom.write(pid.encode())
                        time.sleep(0.02)
                        #print("Data trajectory send:", pid)
                        self.serial_monitor("Data trajectory send:" + pid)
                        #time.sleep(0.5)
                    except:
                        print("Failed to send data!")
                        self.serial_monitor("Failed to send data!")
    
    def update_plot(self):
        if len(self.data_graph) > 100:
            self.data_graph.pop(0)
            self.data_graph_line2.pop(0)
            self.time_graph.pop(0)
            #print(self.data_graph)
        self.curve.setData(self.time_graph, self.data_graph)
        self.curve_line2.setData(self.time_graph, self.data_graph_line2)

        if len(self.data_graph_2) > 100:
            self.data_graph_2.pop(0)
            self.data_graph_2_line2.pop(0)
            self.time_graph_2.pop(0)
        self.curve_2.setData(self.time_graph_2, self.data_graph_2)
        self.curve_2_line2.setData(self.time_graph_2, self.data_graph_2_line2)

        if len(self.data_graph_3) > 100:
            self.data_graph_3.pop(0)
            self.data_graph_3_line2.pop(0)
            self.time_graph_3.pop(0)
        self.curve_3.setData(self.time_graph_3, self.data_graph_3)
        self.curve_3_line2.setData(self.time_graph_3, self.data_graph_3_line2)

        if len(self.data_graph_4) > 100:
            self.data_graph_4.pop(0)
            self.data_graph_4_line2.pop(0)
            self.time_graph_4.pop(0)
        self.curve_4.setData(self.time_graph_4, self.data_graph_4)
        self.curve_4_line2.setData(self.time_graph_4, self.data_graph_4_line2)

        self.mcu_params_display()

    def btn_send_pid_params(self):
        kp_1 = self.uic.lineEdit_2.text()
        ki_1 = self.uic.lineEdit_3.text()
        kd_1 = self.uic.lineEdit_4.text()
        self.angle_1 = int(self.uic.lineEdit_8.text())*10

        kp_2 = self.uic.lineEdit_5.text()
        ki_2 = self.uic.lineEdit_6.text()
        kd_2 = self.uic.lineEdit_7.text()
        self.angle_2 = int(self.uic.lineEdit_9.text())*10*(-1)

        pid = kp_1+","+ki_1+","+kd_1+","+str(self.angle_1)+","+kp_2+","+ki_2+","+kd_2+","+str(self.angle_2)

        if self.serialCom is not None:
            try:
                self.serialCom.write(pid.encode())
                print("Data send:", pid)
                self.serial_monitor(pid)
                #time.sleep(0.5)
            except:
                print("Failed to send data!")
                self.serial_monitor("Failed to send data!")

    def btn_send_serial_monitor(self):
        data = self.uic.lineEdit.text()
        #print(data)
        if self.serialCom is not None:
            try:
                self.serialCom.write(data.encode())
                print("Data send:", data)
                self.serial_monitor(data)
                self.uic.lineEdit.clear()
                #time.sleep(0.5)
            except:
                print("Failed to send data!")
                self.serial_monitor("Failed to send data!")

    def btn_clear_serial_monitor(self):
        self.uic.textEdit.clear()

    def btn_refresh(self):
        self.PORT_LIST, self.DRIVER_LIST = self.esp.get_com_port()
        self.uic.comboBox.clear()
        self.uic.comboBox.addItems(self.PORT_LIST)
        self.serial_monitor("Refreshed serial port")
    
    def btn_connect(self):
        self.COM_PORT = self.uic.comboBox.currentText()
        self.BAUD_RATE = int(self.uic.comboBox_2.currentText())
        if self.COM_PORT:
            self.uic.label_13.setText(self.DRIVER_LIST[self.PORT_LIST.index(self.COM_PORT)])
            try:
                self.serialCom = serial.Serial(self.COM_PORT, self.BAUD_RATE, timeout=1)
                print("Initialized serial port")
                self.serial_monitor("Initialized serial port")

                # Khởi tạo và chạy luồng nhận dữ liệu
                self.receiver_thread = SerialReceiverThread(self.serialCom)
                self.receiver_thread.data_received.connect(self.handle_data_received)
                self.receiver_thread.start()  # Bắt đầu nhận dữ liệu

                # QTimer
                self.clear_all()
                self.timer.start(50)  # Cập nhật đồ thị mỗi 50ms
            except:
                print("Failed to initialize serial port")
                self.serial_monitor("Failed to initialize serial port")
                self.serialCom = None
        else:
            print("No serial port selected.")
            self.serial_monitor("No serial port selected.")
    
    def btn_disconnect(self):
        if self.receiver_thread:
            self.receiver_thread.stop()  # Dừng luồng nhận dữ liệu
            self.receiver_thread = None

        if hasattr(self, 'serialCom') and self.serialCom.is_open:
            self.serialCom.close()
            print("Serial port closed.")
            self.serial_monitor("Serial port closed.")
        self.timer.stop()
        self.mcu_process_time = 0
        self.uic.label_13.setText("N/A")

    def handle_data_received(self, data):
        #print(data)
        self.serial_monitor(data) # Phải mở lại khi debug xong
        self.csv_data.append(data)

        #try:
        values = data.split('/') 
        #print(values, len(values))

        if len(values) == 12:
            self.position_1 = float(values[0])/182
            self.torque_1 = int(values[1])
            self.speed_1 = int(values[2])
            self.torque_pid_1 = int(values[3])
            self.position_set_1 = float(values[4])/10
            self.temp_1 = int(values[5])
            self.position_2 = float(values[6])/182
            self.torque_2 = int(values[7])
            self.speed_2 = int(values[8])
            self.torque_pid_2 = int(values[9])
            self.position_set_2 = float(values[10])/10
            self.temp_2 = int(values[11])

            self.data_graph.append(self.position_set_1)
            self.data_graph_line2.append(self.position_1)
            self.data_graph_2.append(self.torque_pid_1)
            self.data_graph_2_line2.append(self.torque_1)
            self.data_graph_3.append(self.position_set_2)
            self.data_graph_3_line2.append(self.position_2)
            self.data_graph_4.append(self.torque_pid_2)
            self.data_graph_4_line2.append(self.torque_2)

            self.mcu_process_time += 0.02 # Thời gian delay trên vi điều khiển (ms)
            self.time_graph.append(self.mcu_process_time)
            self.time_graph_2.append(self.mcu_process_time)
            self.time_graph_3.append(self.mcu_process_time)
            self.time_graph_4.append(self.mcu_process_time)
            #print(values)
        else:
            print("Error: Invalid data format")
            self.serial_monitor("Error: Invalid data format")
        #except:
        #    print("Error parsing data")
        #    self.serial_monitor("Error parsing data")
###################################################################################################

########################################## Page 2 #################################################
    def mcu_params_display(self):
        self.uic.lineEdit_10.setText(str(round(self.position_1, 0)))
        self.uic.lineEdit_11.setText(str(self.torque_1))
        self.uic.lineEdit_12.setText(str(self.speed_1))
        self.uic.lineEdit_13.setText(str(self.temp_1))
        self.uic.lineEdit_14.setText(str(round(self.position_2, 0)))
        self.uic.lineEdit_15.setText(str(self.torque_2))
        self.uic.lineEdit_16.setText(str(self.speed_2))
        self.uic.lineEdit_17.setText(str(self.temp_2))
###################################################################################################

######################################### Menu Bar ################################################
    def show_page(self):
        self.uic.stackedWidget.setCurrentWidget(self.uic.page)
    
    def show_page_2(self):
        self.uic.stackedWidget.setCurrentWidget(self.uic.page_2)

    def csv_save(self):
        file_name = time.strftime("data_%Y%m%d_%H%M%S.csv")
        try:
            with open(file_name, mode='w', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                for row in self.csv_data:
                    writer.writerow(row.split(','))
            error_message = f"Data saved to {file_name} successfully."
            print(error_message)
            self.serial_monitor(error_message)
        except Exception as e:
            error_message = f"Error saving to CSV: {e}"
            self.serial_monitor(error_message)

    def hide_graph_3(self):
        self.uic.label_10.setVisible(False)
        self.uic.graphicsView_3.setVisible(False)

    def unhide_all(self):
        self.uic.label_10.setVisible(True)
        self.uic.graphicsView_3.setVisible(True)
    
    def clear_graph_1(self):
        self.curve.clear()
        self.curve_line2.clear()
        self.data_graph = []
        self.data_graph_line2 = []
        self.time_graph = []

    def clear_graph_2(self):
        self.curve_2.clear()
        self.data_graph_2 = []
        self.time_graph_2 = []

    def clear_graph_3(self):
        self.curve_3.clear()
        self.data_graph_3 = []
        self.time_graph_3 = []

    def clear_all(self):
        self.curve.clear()
        self.curve_line2.clear()
        self.curve_2.clear()
        self.curve_3.clear()

        self.data_graph = []
        self.data_graph_line2 = []
        self.data_graph_2 = []
        self.data_graph_3 = []
        self.time_graph = []
        self.time_graph_2 = []
        self.time_graph_3 = []

    def serial_monitor(self, text):
        display_text = str(time.strftime("%H:%M:%S", time.localtime())) + " -> " + text
        self.uic.textEdit.append(display_text)
    
    def open_serial_monitor(self):
        self.uic.groupBox.setVisible(True)

    def close_serial_monitor(self):
        self.uic.groupBox.setVisible(False)

    def show(self):
        self.main_win.showMaximized()

    def exit_application(self):
        QApplication.quit()
###################################################################################################

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())