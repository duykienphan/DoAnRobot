"""
Quy hoạch quỹ đạo 2 điểm xy. Trong mô hình thực tế xy = xz
"""

import numpy as np
import math
from inverse_kinematic import Inverse_Kinematic

class Trajector:
    def __init__(self):
        self.IK = Inverse_Kinematic()

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
    
    def point2point_operate(self, t_time=6, cycle=0.02): # t_time: Thời gian chạy
        ta = 0
        xa = 20
        ya = 40
        vxa = 0
        vya = 0

        tb = 3
        xb = -20
        yb = 40
        vxb = 0
        vyb = 0

        tc = 6
        xc = 20
        yc = 40
        vxc = 0
        vyc = 0

        t = 0
        loop_duration = tc
        loop_time = t % loop_duration # Thời gian lặp. Thgian nhap = 20s
        list_coordinate = []

        while t <= t_time:
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
            t += cycle
            loop_time = t % loop_duration
            angle_x, angle_y = self.IK.IK_2DOF(x, 0, y, 1)
            list_coordinate.append((self.IK.rad2deg(angle_x), self.IK.rad2deg(angle_y)))
        return list_coordinate

    def traingle_operate(self, t_time=6, cycle=0.02):
        ta = 0
        xa = 22
        ya = 20
        vxa = 0
        vya = 0

        tb = 3
        xb = 0
        yb = 42
        vxb = 0
        vyb = 0

        tc = 6
        xc = -26
        yc = 23
        vxc = 0
        vyc = 0

        td = 9
        xd = 22
        yd = 20
        vxd = 0
        vyd = 0

        t = 0
        loop_duration = tc
        loop_time = t % loop_duration # Thời gian lặp. Thgian nhap = 20s
        list_coordinate = []

        while t <= t_time:
            if loop_time < ta:
                # Nếu thời gian trước điểm A, robot đứng tại A
                x = xa
                y = ya
            elif loop_time <= tb:
                # Đoạn A đến B: Tính vị trí sử dụng phương trình bậc ba
                x, y = self.point2point_formular(loop_time, ta, tb, xa, ya, vxa, vya, xb, yb, vxb, vyb)
            elif loop_time <= tc:
                x, y = self.point2point_formular(loop_time, tb, tc, xb, yb, vxb, vyb, xc, yc, vxc, vyc)
            elif loop_time <= td:
                x, y = self.point2point_formular(loop_time, tc, td, xc, yc, vxc, vyc, xd, yd, vxd, vyd)
            else:
                x = xd
                y = yd

            x, y = round(x, 2), round(y, 2)
            t += cycle
            loop_time = t % loop_duration
            angle_x, angle_y = self.IK.IK_2DOF(x, 0, y, 2)
            angle_x, angle_y = self.IK.rad2deg(angle_x), self.IK.rad2deg(angle_y)
            list_coordinate.append((angle_x, angle_y))

        while (angle_x != list_coordinate[0][0]) or (angle_y != list_coordinate[0][1]):
            if angle_x < list_coordinate[0][0]:
                angle_x += 1
            elif angle_x > list_coordinate[0][0]:
                angle_x -= 1

            if angle_y < list_coordinate[0][1]:
                angle_y += 1
            elif angle_y > list_coordinate[0][1]:
                angle_y -= 1
            #print(angle_x, angle_y, (angle_x != list_coordinate[0][0]) and (angle_y != list_coordinate[0][1]))
            list_coordinate.append((angle_x, angle_y))
        return list_coordinate
    
if __name__ == "__main__":
    TP = Trajector()
    lst = TP.traingle_operate()
    print(len(lst))
    """
    for i in range(len(lst)):
        print(lst[i])
"""
