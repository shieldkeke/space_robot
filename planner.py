import numpy as np

# quintic curve planning with zero or none-zero start and end velocity and zero acc, return k_array
def quintic_curve_planning(start_pos, end_pos, time, start_vel = [0]*10, end_vel = [0]*10):
    time_matrix = np.matrix([
        [         0,           0,             0,          0,        0,   1],
        [   time**5,     time**4,       time**3,    time**2,     time,   1],
        [         0,           0,             0,          0,        1,   0],
        [ 5*time**4,   4*time**3,     3*time**2,     2*time,        1,   0],
        [         0,           0,             0,          2,        0,   0],
        [20*time**3,  12*time**2,        6*time,          2,        0,   0]

    ])
    inv_time_matrix = np.linalg.inv(time_matrix)
    k_array = []
    for i in range(len(start_pos)):
        s = start_pos[i]
        e = end_pos[i]
        if abs(s-e)>np.pi:
            if s<0:
                s = s + np.pi*2
            else:
                e = e + np.pi*2

        X = np.matrix([s, e, start_vel[i], end_vel[i], 0, 0]).T
        k = np.dot(inv_time_matrix,X)
        k_array.append(k)
    return k_array

# input time, return joint position
def quintic_curve_excute(k_array, time):
    time_vector = np.matrix([time**5,     time**4,       time**3,    time**2,     time,   1]).T
    joint_positions = []
    for i in range(len(k_array)):
        joint_position = np.dot(k_array[i].T,time_vector)
        joint_positions.append(joint_position[0,0])
    return np.array(joint_positions)

def linear_interpolation(start_pos, end_pos, num):
    pos = []
    for i in range(num):
        pos.append([start_pos[j] + (end_pos[j] - start_pos[j]) / num * i for j in range(len(start_pos))] )
    return pos