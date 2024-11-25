import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QComboBox
from PyQt5.QtCore import QTimer
import pyqtgraph as pg


class RealTimePlot(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Time Plot with PySerial")
        self.setGeometry(100, 100, 800, 600)

        # Cấu trúc giao diện
        self.widget = QWidget()
        self.layout = QVBoxLayout(self.widget)
        self.setCentralWidget(self.widget)

        # Danh sách các cổng serial
        self.port_selector = QComboBox()
        self.list_ports()
        self.layout.addWidget(self.port_selector)

        # Nút bắt đầu
        self.start_button = QPushButton("Start Plotting")
        self.start_button.clicked.connect(self.start_plotting)
        self.layout.addWidget(self.start_button)

        # Đồ thị PyQtGraph
        self.graph = pg.PlotWidget()
        self.layout.addWidget(self.graph)
        self.graph.setBackground('w')
        self.curve = self.graph.plot(pen=pg.mkPen(color='b', width=2))
        self.graph.setLabel('left', 'Value')
        self.graph.setLabel('bottom', 'Time')

        # Dữ liệu để vẽ
        self.data = []
        self.time = []

        # Serial và Timer
        self.serial_connection = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)

    def list_ports(self):
        """Liệt kê các cổng serial có sẵn"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_selector.addItem(port.device)

    def start_plotting(self):
        """Kết nối với serial và bắt đầu nhận dữ liệu"""
        port = self.port_selector.currentText()
        try:
            self.serial_connection = serial.Serial(port, baudrate=9600, timeout=1)
            self.timer.start(50)  # Lấy dữ liệu mỗi 50ms
            self.start_button.setEnabled(False)
        except Exception as e:
            print(f"Không thể kết nối với {port}: {e}")

    def update_plot(self):
        """Nhận dữ liệu từ serial và vẽ đồ thị"""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                line = self.serial_connection.readline().strip()
                decoded_data = line.decode("utf-8", errors="replace")
                if line:  # Kiểm tra xem có dữ liệu không
                    value = float(decoded_data)  # Chuyển dữ liệu thành số thực
                    print(value)
                    self.data.append(value)
                    if len(self.data) > 100:  # Chỉ lưu 100 điểm dữ liệu gần nhất
                        self.data.pop(0)

                    self.time = list(range(len(self.data)))
                    self.curve.setData(self.time, self.data)  # Cập nhật đồ thị
            except Exception as e:
                print(f"Lỗi đọc dữ liệu: {e}")
        else:
            self.timer.stop()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RealTimePlot()
    window.show()
    sys.exit(app.exec_())
