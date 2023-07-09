import numpy as np
from numpy import cos, sin, sqrt, pi
from numpy import arctan2 as atan2
from numpy import arcsin as asin
from numpy import arccos as acos

class IKSlover:
    def __init__(self) -> None:
        pass
    
    def angles_to_pi(self, angles):
        for i in range(len(angles)):
            while angles[i] > pi:
                angles[i] -= 2*pi
            while angles[i] < -pi:
                angles[i] += 2*pi

    def angle_to_pi(self, angle):
        while angle > pi:
            angle -= 2*pi
        while angle < -pi:
            angle += 2*pi
        return angle
    
    def euler_angles_to_rotation_matrix(self, theta):
        R_x = [[1, 0, 0],
               [0, cos(theta[0]), -sin(theta[0])],
               [0, sin(theta[0]), cos(theta[0])]
               ]
        R_y = [[cos(theta[1]), 0, sin(theta[1])],
               [0, 1, 0],
               [-sin(theta[1]), 0, cos(theta[1])]
               ]
        R_z = [[cos(theta[2]), -sin(theta[2]), 0],
               [sin(theta[2]), cos(theta[2]), 0],
               [0, 0, 1]
               ]
        R = np.dot(R_x, np.dot(R_y, R_z))
        return R
    
    # input:x, y, z, roll, pitch, yaw, return:transformation matrix
    def state_to_matrix(self, state):
        R = self.euler_angles_to_rotation_matrix(state[3:])
        T = np.array([[state[0]], [state[1]], [state[2]]])
        M = np.hstack((R, T))
        M = np.vstack((M, [0, 0, 0, 1]))
        return M

    def solve(self, state):
        T = self.state_to_matrix(state)
        Len = 1
        m = -0.085*T[0][2] - T[0][3]
        n = -0.085*T[1][2] - T[1][3]
        t1 = np.array([-atan2(0.3,-sqrt(m**2 + n**2 - 0.3**2))+ atan2(m,-n), 
                       -atan2(0.3,sqrt(m**2 + n**2 - 0.3**2))+ atan2(m,-n)], dtype=np.float64) 
        Len = Len * 2
        t6_1 = np.array([asin(-T[1][2]*sin(x)-T[0][2]*cos(x)) for x in t1], dtype=np.float64)
        t6_2 = np.array([pi-asin(-T[1][2]*sin(x)-T[0][2]*cos(x)) for x in t1], dtype=np.float64)
        t6 = np.hstack((t6_1, t6_2))
        m = np.array([T[0][1]*cos(x) + T[1][1]*sin(x) for x in t1], dtype=np.float64)
        n = np.array([T[0][0]*cos(x) + T[1][0]*sin(x) for x in t1], dtype=np.float64)
        t7_1 = np.array([atan2(m[i], -n[i])-pi  for i in range(Len)], dtype=np.float64)
        t7_2 = np.array([atan2(m[i], -n[i]) for i in range(Len)], dtype=np.float64)
        t7 = np.hstack((t7_1, t7_2))
        Len = Len * 2 # 4

        m = np.array([-0.085 + 0.085*T[2][2] + T[2][3] - 0.1*T[2][1]*cos(x) - 0.1*T[2][0]*sin(x) for x in t7], dtype=np.float64)
        n = np.array([-0.1 + sin(t1[i%2])*(0.085*T[0][2]+T[0][3]-0.1*T[0][1]*cos(t7[i])-0.1*T[0][0]*sin(t7[i])) + cos(t1[i%2])*(-0.085*T[1][2]-T[1][3]+0.1*T[1][1]*cos(t7[i])+0.1*T[1][0]*sin(t7[i]))for i in range(Len)], dtype=np.float64)
        t4_1 = np.array([acos((0.32-m[i]**2-n[i]**2)/0.32) for i in range(Len)], dtype=np.float64)
        t4_2 = np.array([-acos((0.32-m[i]**2-n[i]**2)/0.32) for i in range(Len)], dtype=np.float64)
        t4 = np.hstack((t4_1, t4_2))
        Len = Len * 2 # 8

        t3 = np.array([atan2((-1+cos(t4[i]))*n[i%(Len//2)] + sin(t4[i])*m[i%(Len//2)], (1-cos(t4[i]))*m[i%(Len//2)] + sin(t4[i])*n[i%(Len//2)]) for i in range(Len)], dtype=np.float64)
        m = np.array([-T[2][1]*cos(x) + T[2][0]*sin(x) for x in t7], dtype=np.float64)
        n = np.array([T[2][2]*cos(t6[i]) + sin(t6[i])*(T[2][0]*cos(t7[i])-T[2][1]*sin(t7[i])) for i in range(4)], dtype=np.float64)
        t5 = np.array([t3[i] + t4[i] - atan2(m[i%(Len//2)], n[i%(Len//2)]) for i in range(Len)], dtype=np.float64)
        # t2 all 0
        t2 = np.zeros(Len, dtype=np.float64)

        self.angles_to_pi(t1)
        self.angles_to_pi(t2)
        self.angles_to_pi(t3)
        self.angles_to_pi(t4)
        self.angles_to_pi(t5)
        self.angles_to_pi(t6)
        self.angles_to_pi(t7)


        # rst = np.array([[t1[i%2], t2] for i in range(Len)]).T
        rst = np.array([[t1[i%2], t2[i], t3[i], t4[i], t5[i], t6[i%4], t7[i%4]] for i in range(Len)])

        return rst
    
if __name__ == "__main__":
    ik = IKSlover()
    state = np.array([0.6, 0.0, 0.0, 0, 0, 0], dtype=np.float64)
    # state = np.array([0.0, 0.0, -0.6, 0, 0, 0], dtype=np.float64)
    # state = np.array([-0.4, 0.135, -0.1, -np.pi/2, 0, 0], dtype=np.float64)
    rst = ik.solve(state)
    print("*"*20)
    print(rst)