import math
import numpy as np

class RLS:
    def __init__(self):
        self.error = 10
        self.pre_speed_1 = 0
        self.pre_speed_2 = 0
        self.pre_pos_1 = 0
        self.pre_pos_2 = 0
        self.R_p = np.ones((8, 8)) * self.error
        self.theta_p = np.zeros((8, 1))    
        self.count = 0

    def identification(self, torque1, torque2, theta1, theta2): # theta = position, dtheta = speed
        t1 = torque1
        t2 = torque2

        dtheta1 = self.speed_1_calc(theta1)
        dtheta2 = self.speed_2_calc(theta2)

        ddtheta1 = self.acceleration_1_calc(dtheta1)
        ddtheta2 = self.acceleration_2_calc(dtheta2)

        a11 = ddtheta1
        a12 = ddtheta1 + ddtheta2
        a13 = (2 * ddtheta1 + ddtheta2) * math.cos(theta2) - (2 * dtheta1 * dtheta2 + (dtheta2**2)) * math.sin(theta2)
        a14 = -(2 * ddtheta1 + ddtheta2) * math.sin(theta2) - (2 * dtheta1 * dtheta2 + (dtheta2**2)) * math.cos(theta2)
        a15 = math.cos(theta1+theta2)
        a16 = -(math.sin(theta1+theta2))
        a17 = math.cos(theta1)
        a18 = -(math.sin(theta1))

        a21 = 0
        a22 = ddtheta1 + ddtheta2
        a23 = ddtheta1 * math.cos(theta2) + (dtheta1**2) * math.sin(theta2)
        a24 = -ddtheta1 * math.sin(theta2) + (dtheta1**2) * math.cos(theta2)
        a25 = math.cos(theta1+theta2)
        a26 = -(math.sin(theta1+theta2))
        a27 = 0
        a28 = 0

        lamda = 0.98
        y_k = np.array([t1, t2])
        phi = np.array([[a11, a12, a13, a14, a15, a16, a17, a18],
                        [a21, a22, a23, a24, a25, a26, a27, a28]])
        
        R_current = lamda*self.R_p + np.matmul(phi.T, phi)
        e_k = y_k - np.matmul(phi, self.theta_p)
        
        R_inv = np.linalg.inv(R_current)
        theta = self.theta_p + np.matmul(np.matmul(R_inv, phi.T), e_k)
        y_h = np.matmul(phi, theta)

        self.R_p = R_current
        self.theta_p = theta

        return y_h, R_current, theta

    def acceleration_1_calc(self, speed):
        acceleration = (speed - self.pre_speed_1) / (0.02)
        self.pre_speed_1 = speed
        return round(acceleration, 2)
    
    def acceleration_2_calc(self, speed):
        acceleration = (speed - self.pre_speed_2) / (0.02)
        self.pre_speed_2 = speed
        return round(acceleration, 2)

    def speed_1_calc(self, pos):
        speed = (pos - self.pre_pos_1) / (0.02)
        self.pre_pos_1 = pos
        return round(speed, 2)
    
    def speed_2_calc(self, pos):
        speed = (pos - self.pre_pos_2) / (0.02)
        self.pre_pos_2 = pos
        return round(speed, 2)

if __name__ == "__main__":
    rls = RLS()
    x, y, z = rls.identification(0.0393, -0.02141, 1.499, 0.5983) # angle(radian), speed (dps), accleration (m/s^2)
    print(x)
    print()
    print(y)
    print()
    print(z)
    