import math

class Inverse_Kinematic:
    def __init__(self):
        pass

    def IK_2DOF(self, Px, Py, Pz, solution):
        l0 = 6.8 #cm
        l1 = 20.1
        l2 = 15.2
        d0 = 1.9
        d1 = -4.7
        Py = 0 - d0 - d1

        # Theta 2
        c2 = (Px**2+(Pz-l0)**2-(l2**2+l1**2))/(2*l1*l2)
        c2 = min(1, max(c2, -1))
        s2_1 = math.sqrt(1-c2**2)
        s2_2 = -(math.sqrt(1-c2**2))

        theta2_1 = math.atan2(s2_1, c2)
        theta2_2 = math.atan2(s2_2, c2)

        # Theta 1
        c1_1 = (Px*(l2*math.cos(theta2_1)+l1)+(Pz-l0)*l2*math.sin(theta2_1))/(((l2*math.cos(theta2_1)+l1)**2)+(l2*math.sin(theta2_1))**2)
        c1_2 = (Px*(l2*math.cos(theta2_2)+l1)+(Pz-l0)*l2*math.sin(theta2_2))/(((l2*math.cos(theta2_2)+l1)**2)+(l2*math.sin(theta2_2))**2)

        s1_1 = (-Px*l2*math.sin(theta2_1)+(Pz-l0)*(l2*math.cos(theta2_1)+l1))/((l2*math.cos(theta2_1)+l1)**2+(l2*math.sin(theta2_1))**2)
        s1_2 = (-Px*l2*math.sin(theta2_2)+(Pz-l0)*(l2*math.cos(theta2_2)+l1))/((l2*math.cos(theta2_2)+l1)**2+(l2*math.sin(theta2_2))**2)

        theta1_1 = math.atan2(s1_1, c1_1) 
        theta1_2 = math.atan2(s1_2, c1_2)

        if solution == 1:
            theta1 = theta1_1
            theta2 = theta2_1
        else:
            theta1 = theta1_2
            theta2 = theta2_2

        return theta1, theta2
    
    def rad2deg(self, rad):
        deg = rad*(180/math.pi)
        deg = round(deg, 0)
        if deg == -0.0:
            deg = 0.0
        return int(deg)
    
    def deg2rad(self, deg):
        rad = deg * (math.pi/180)
        rad = round(rad, 4)
        if rad == -0.0000:
            rad = 0.0
        return float(rad)
    
if __name__ == "__main__":
    IK = Inverse_Kinematic()

    theta1, theta2 = IK.IK_2DOF(-35.3, 2.8, 6.8, 1)
    print(IK.rad2deg(theta1), IK.rad2deg(theta2))
