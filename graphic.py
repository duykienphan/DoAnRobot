from matplotlib import pyplot as plt
import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

class CHART(FigureCanvasQTAgg):
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        super().__init__(self.fig)

        plt.close
        plt.ion
        self.loop()

    def loop(self):
        xpoints = np.array([1, 2, 6, 8])
        ypoints = np.array([3, 8, 1, 10])

        self.ax.plot(xpoints, ypoints)