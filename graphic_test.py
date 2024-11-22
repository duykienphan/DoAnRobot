import sys
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtGui import QPixmap, QPainter
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Matplotlib into QWidget")
        self.setGeometry(100, 100, 800, 600)
        
        # Tạo widget chính và layout
        widget = QWidget(self)
        layout = QVBoxLayout(widget)

        # Tạo một QWidget để chứa đồ thị
        self.canvas_widget = QWidget(self)
        layout.addWidget(self.canvas_widget)

        # Vẽ đồ thị và chèn vào QWidget
        self.plot_data()

        # Thiết lập widget chính
        self.setCentralWidget(widget)

    def plot_data(self):
        # Dữ liệu mẫu
        x = np.linspace(0, 10, 100)
        y = np.sin(x)

        # Tạo figure và plot
        fig, ax = plt.subplots(figsize=(5, 4))
        ax.plot(x, y, label='y = sin(x)')
        ax.set_title('Đồ thị y = sin(x)')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.legend()

        # Tạo một FigureCanvas để vẽ đồ thị vào buffer
        canvas = FigureCanvas(fig)
        canvas.draw()

        # Chuyển đồ thị vào QWidget
        layout = QVBoxLayout(self.canvas_widget)
        layout.addWidget(canvas)

        self.canvas_widget.setLayout(layout)
        self.canvas_widget.setFixedSize(600, 400)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
