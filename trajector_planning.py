"""
Quy hoạch quỹ đạo 2 điểm xy. Trong mô hình thực tế xy = xz
"""

import numpy as np
import math

class Trajector:
    def __init__(self):
        pass

    def point2point_formular(self, t, t_start, t_end, x_start, y_start, vx_start, vy_start, x_end, y_end, vx_end, vy_end):        
        # Ma trận hệ số cho đa thức bậc ba
        A = np.array([[1, t_start, t_start**2, t_start**3],
                     [0, 1, 2*t_start, 3*(t_start**2)],
                     [1, t_end, t_end**2, t_end**3],
                     [0, 1, 2*t_end, 3*(t_end**2)]])
        
        # Giải hệ số x
        MX = np.array([x_start, vx_start, x_end, vx_end])
        xx = np.matmul(np.linalg.inv(A), MX.T)

        # Giải hệ số y
        MY = np.array([y_start, vy_start, y_end, vy_end])
        yy = np.matmul(np.linalg.inv(A), MY.T)

        # Tính toán vị trí x và y tại thời điểm t
        x = xx[0] + xx[1]*t + xx[2]*(t**2) + xx[3]*(t**3)
        y = yy[0] + yy[1]*t + yy[2]*(t**2) + yy[3]*(t**3)

        return x, y
    
    def point2point_operate(self):
        ta = 0
        xa = 13
        ya = 40
        vxa = 0
        vya = 0

        tb = 3
        xb = -13
        yb = 40
        vxb = 0
        vyb = 0

        tc = 6
        xc = 13
        yc = 40
        vxc = 0
        vyc = 0

        t = 0.02

        loop_duration = tc

        loop_time = t % loop_duration # Thời gian lặp

        if loop_time < ta:
            # Nếu thời gian trước điểm A, robot đứng tại A
            x = xa
            y = ya
        elif loop_time <= tb:
            # Đoạn A đến B: Tính vị trí sử dụng phương trình bậc ba
            x, y = self.point2point_formular(loop_time, ta, tb, xa, ya, vxa, vya, xb, yb, vxb, vyb)
        elif loop_time <= tc:
            x, y = self.point2point_formular(loop_time, tb, tc, xb, yb, vxb, vyb, xc, yc, vxc, vyc)

        x, y = round(x, 2), round(y, 2)
        return x, y

if __name__ == "__main__":
    TP = Trajector()
    x, y = TP.point2point_operate()
    print(x, y)
