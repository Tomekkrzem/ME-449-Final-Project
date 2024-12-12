from src import FeedbackControl, Velocities, NextState, TrajectoryGenerator
from config import conf_t, conf
from numpy import cos, sin
import numpy as np
import modern_robotics as mr
import pandas as pd

Mse = conf()[0]
M0e = conf()[1]
Tb0 = conf()[2]
B_list = conf()[3]
F = conf()[4]
r_conf = conf()[5]
dt = conf()[6]

J_lim = None

Kp = 0
Ki = 0
err_sum = [np.array([0,0,0,0,0,0])]

def main():
    r_tj = TrajectoryGenerator(*conf_t())

    # Vector of Trajectories
    state = r_conf
    traj = [[*r_conf.tolist(),0]]

    for i in range(len(r_tj)-1):

        # Desired Frame
        Xd = np.array([[r_tj[i][0] ,r_tj[i][1] ,r_tj[i][2] ,r_tj[i][9]],
                       [r_tj[i][3] ,r_tj[i][4] ,r_tj[i][5] ,r_tj[i][10]],
                       [r_tj[i][6] ,r_tj[i][7] ,r_tj[i][8] ,r_tj[i][11]],
                       [          0,          0,          0,          1]])

        # Next Desired Frame
        Xd_next = np.array([[r_tj[i+1][0],r_tj[i+1][1],r_tj[i+1][2],r_tj[i+1][9]],
                           [r_tj[i+1][3],r_tj[i+1][4],r_tj[i+1][5],r_tj[i+1][10]],
                           [r_tj[i+1][6],r_tj[i+1][7],r_tj[i+1][8],r_tj[i+1][11]],
                           [           0,           0,            0,           1]])

        # Gripper State
        g_state = r_tj[i][-1]

        phi, x, y = state[0:3]

        Tsb = np.array([[cos(phi), -sin(phi),    0,      x],
                        [sin(phi),  cos(phi),    0,      y],
                        [       0,         0,    1, 0.0963],
                        [       0,         0,    0,      1]])

        # End Effector Frame in Arm Base Frame
        T0e = mr.FKinBody(M0e,B_list,state[3:8])

        # Current End Effector Position
        X = np.matmul(Tsb,np.matmul(Tb0,T0e))

        # Commanded End Effector Twist
        FC = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt,err_sum[i])
        V = FC[0]
        err_sum.append(FC[1])

        # Wheel and Joint Velocities
        rates = Velocities(state[0:8],B_list,V,M0e,Tb0,F,J_lim,dt,15)

        # Current State Updates to Next State
        state = NextState(state, rates, dt, 15)

        # Complete Robot Trajectory
        traj.append([*state, g_state])

    # Code Used to Generate CSV Files
    df = pd.DataFrame(traj)

    file_name = 'testf'

    # CHANGE FILE PATH TO GENERATE CSV PROPERLY
    file_path = 'C:/Users/tomek/Downloads/School Work/ME 449/'

    df.to_csv(str(file_path) + str(file_name) + '.csv', index=False, header=False)

    print(f"File {file_name} was Generated")


if __name__ == "__main__":
    main()