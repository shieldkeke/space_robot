# Before running this example, make sure you have installed the following package.
# pip install coppeliasim-zmqremoteapi-client numpy
# You can find more information about ZeroMQ remote API 
# in the file path <Coppeliasim_install_path>\programming\zmqRemoteApi
# or on https://github.com/CoppeliaRobotics/zmqRemoteApi
#
# You can get more API about coppleliasim on https://coppeliarobotics.com/helpFiles/en/apiFunctions.htm

import time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from planner import quintic_curve_excute, quintic_curve_planning, linear_interpolation
from ik_space_robot import IKSlover

# for joint order
LEFT = 0
RIGHT = 1

# for task type
CARTESIAN_SPACE = 0 # linear interpolation In Cartesian space , then IK
JOINT_SPACE = 1 # IK with start and end ,then polynomial interpolation in joint space

# ik solver
IK = IKSlover()

# for simulation
client = RemoteAPIClient()
sim = client.getObject('sim')
TIME_STEP = 0.05

# Get object handle
l_joint = []
r_joint = []
mid_joint = None
for i in range(1, 4):
    l_joint.append(sim.getObject(f'./L_Joint{i}'))
    r_joint.append(sim.getObject(f'./R_Joint{i}'))
mid_joint = sim.getObject('./Joint4')

l_base = sim.getObject('./L_Base')
r_base = sim.getObject('./R_Base')
l_link = []
r_link = []
for i in range(1, 4):
    l_link.append(sim.getObject(f'./L_Link{i}'))
    r_link.append(sim.getObject(f'./R_Link{i}'))
space_station = sim.getObject('./SpaceStation')

# determine the base of robot and return the order of joints
def get_robot(configuration=LEFT):
    joints = []
    links = []
    base = None

    if configuration == LEFT:
        # get order
        joints = [l_joint[0], l_joint[1], l_joint[2], mid_joint, r_joint[2], r_joint[1], r_joint[0]]
        links = [l_base, l_link[0], l_link[1], l_link[2], r_link[2], r_link[1], r_link[0], r_base]
        base = l_base
        
    elif configuration == RIGHT:
        joints = [r_joint[0], r_joint[1], r_joint[2], mid_joint, l_joint[2], l_joint[1], l_joint[0]]
        links = [r_base, r_link[0], r_link[1], r_link[2], l_link[2], l_link[1], l_link[0], l_base]
        base = r_base
    
    else:
        print("error! wrong configuration type!")

    # change parent
    sim.setObjectParent(base, space_station, True)
    for i in range(len(links)-1):
        sim.setObjectParent(joints[i], links[i], True)
        sim.setObjectParent(links[i+1], joints[i], True)

    return joints


# input task table: [[k or states on a line, traj_time, message, type], ...] 
# return joint angles
last_task = None

def excute_task_table(t, table):
    global last_task
    global robot
    global ORDER
    sum_time = 0
    for i, task in enumerate(table):
        if t < sum_time + task[1]:
            if last_task != i:
                ORDER = task[4]
                robot = get_robot(ORDER)
                print(task[2])
                last_task = i
            if task[3] == JOINT_SPACE:
                return quintic_curve_excute(task[0], t-sum_time) # IK with start and end ,then polynomial interpolation in joint space
            elif task[3] == CARTESIAN_SPACE:
                return task[0][int( (t-sum_time) / task[1] * (len(task[0])-1) )] # linear interpolation In Cartesian space , then IK
            else: 
                print("error! wrong task type!")
                return None
        sum_time = sum_time + task[1]
    # print("no more task")
    return None
# state: [x, y, z, alpha, beta, gamma], is End-effector's state
# input: start state & table: [[state , traj_time, message, type], ...]
# output: task table
def make_task_table(start, table):
    task_table = []
    for i, task in enumerate(table):
        if i == 0:
            start_state = start
        else:
            start_state = table[i-1][0]
        end_state = task[0]
        traj_time = task[1]
        message = task[2]
        type = task[3]
        orientation = task[4]

        if type == JOINT_SPACE:
            start_joint = IK.solve(start_state)[0]
            end_joint = IK.solve(end_state)[0]
            if orientation == RIGHT:
                start_joint = start_joint[::-1] 
                end_joint = end_joint[::-1] 

            print(start_state)
            print(end_state)
            print(start_joint)
            print(end_joint)
            print(orientation)
            print("*"*20)
            if orientation == LEFT: 
                task_table.append([quintic_curve_planning(start_joint, 2*start_joint+end_joint, traj_time), traj_time, message, type, orientation])
            else:
                task_table.append([quintic_curve_planning(start_joint, end_joint, traj_time), traj_time, message, type, orientation])
        elif type == CARTESIAN_SPACE:
            # start and end ori must the same
            states = linear_interpolation(start_state, end_state, int(traj_time/TIME_STEP))
            angles = [IK.solve(state)[0] for state in states]
            if orientation == RIGHT:
                angles = [angle[::-1] for angle in angles]
            task_table.append([angles, traj_time, message, type, orientation])
        else:
            print("error! wrong task type!")
            return None
        
    return task_table


if __name__ == '__main__':

    TOLTAL_TIME = 12

    print('Program started')

    # When simulation is not running, ZMQ message handling could be a bit
    # slow, since the idle loop runs at 8 Hz by default. So let's make
    # sure that the idle loop runs at full speed for this program:
    defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
    sim.setInt32Param(sim.intparam_idle_fps, 0)

    # Run a simulation in stepping mode:
    client.setStepping(True)
    sim.startSimulation()

    # ORDER = RIGHT
    ORDER = LEFT
    robot = get_robot(ORDER)

    # make task table
    start = [-0.3, 0.0, 0.0, 0, 0, 0]
    states = [[[0.6, 0.0, 0.0, 0, 0, 0], 3, "start to A", JOINT_SPACE, RIGHT],
            #   [[-0.3, 0.0, 0.0, 0, 0, 0], 3, "A to B", JOINT_SPACE, LEFT]]
              [[-0.4, 0.135, -0.135, -np.pi/2, 0, 0], 3, "A to B", JOINT_SPACE, LEFT]]
    # start = [0.6, 0.0, 0.0, 0, 0, 0]
    # states = [[[-0.4, 0.135, -0.135, -np.pi/2, 0, 0], 3, "A to B", JOINT_SPACE, LEFT]]
    task_table = make_task_table(start, states)

    # for graph
    dataPos = []
    dataVel = []
    dataAcc = []
    last_v = [0]*7
    first_flag = True
    # graphPos = sim.getObject('./DataPos')
    # graphVel = sim.getObject('./DataVel')
    # graphAcc = sim.getObject('./DataAcc')
    # color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1]]
    # for i in range(7):
    #     dataPos.append(sim.addGraphStream(graphPos, 'Joint'+str(i+1), 'deg', 0, color[i]))
    #     dataVel.append(sim.addGraphStream(graphVel, 'Joint'+str(i+1), 'deg/s', 0, color[i]))
    #     dataAcc.append(sim.addGraphStream(graphAcc, 'Joint'+str(i+1), 'deg/s2', 0, color[i]))

    # run simulation
    while (t := sim.getSimulationTime()) < TOLTAL_TIME:
        
        # do some stuff...

        angles = excute_task_table(t, task_table)
        if angles is not None:
            # print(robot)
            # print(angles)
            for i in range(len(robot)):
                sim.setJointPosition(robot[i], angles[i])

        # message output
        message = f'Simulation time: {t:.2f} s'
        # print(message)
        sim.addLog(sim.verbosity_scriptinfos, message)

        # for graph
        for i in range(len(robot)):
            if first_flag:
                vel = sim.getJointVelocity(robot[i])
                last_v[i] = vel
                first_flag = False
                break
            pos = sim.getJointPosition(robot[i])
            # if i == 6:
            #     if pos < -100/180*np.pi:
            #         pos += 2*np.pi
            vel = sim.getJointVelocity(robot[i])
            acc = (vel - last_v[i])/sim.getSimulationTimeStep()
            last_v[i] = vel
            # sim.setGraphStreamValue(graphPos,dataPos[i], pos*180/np.pi)
            # sim.setGraphStreamValue(graphVel,dataVel[i], vel*180/np.pi)
            # sim.setGraphStreamValue(graphAcc,dataAcc[i], acc*180/np.pi)

        # triggers next simulation step
        client.step()  
        time.sleep(0.01)

    # Stop simulation
    sim.stopSimulation()

    # Restore the original idle loop frequency:
    sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

    print('Program ended')
