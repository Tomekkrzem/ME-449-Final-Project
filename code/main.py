from code.src import FeedbackControl, Velocities, NextState, TrajectoryGenerator
from code.config import (conf_t_best, conf_t_overshoot, conf_t_newTask,
                         conf_best, conf_overshoot, conf_newTask)
from numpy import cos, sin
import numpy as np
import modern_robotics as mr
import pandas as pd
import matplotlib.pyplot as plt

def main(Mse,M0e,Tb0,B_list,F,r_conf,dt,Kp,Ki,err_arr,v_lim,file_name,traj_config):
    # Starting Error Integral
    err_sum = np.array([0, 0, 0, 0, 0, 0], dtype='float64')

    r_tj = TrajectoryGenerator(*traj_config)
    # Vector of Trajectories
    state = r_conf
    traj = [[*r_conf.tolist(),0]]

    # Loop Through all Trajectories
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
        g_state = r_tj[i+1][-1]

        # Chassis Configurations
        phi, x, y = state[0:3]

        # Updating Base Frame
        Tsb = np.array([[cos(phi), -sin(phi),    0,      x],
                        [sin(phi),  cos(phi),    0,      y],
                        [       0,         0,    1, 0.0963],
                        [       0,         0,    0,      1]])

        # End Effector Frame in Arm Base Frame
        T0e = mr.FKinBody(M0e,B_list,state[3:8])

        # Current End Effector Position
        X = np.matmul(Tsb,np.matmul(Tb0,T0e))

        # Commanded End Effector Twist
        FC = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt,err_sum)
        V = FC[0]
        err_arr.append(FC[1])

        # Wheel and Joint Velocities
        rates = Velocities(state,B_list,V,M0e,Tb0,F)

        # Current State Updates to Next State
        state = NextState(state, rates, dt, v_lim)

        # Complete Robot Trajectory
        traj.append([*state, g_state])

    # Code Used to Generate CSV Files
    df = pd.DataFrame(traj)
    df2 = pd.DataFrame(err_arr)

    # CHANGE FILE PATH TO GENERATE CSV PROPERLY
    file_path = f'C:/Users/tomek/PycharmProjects/ME 449 Final Project/results/{file_name}/'
    df.to_csv(str(file_path) + str(file_name) + '.csv', index=False, header=False)
    df2.to_csv(str(file_path) + 'X_err_' + str(file_name) + '.csv', index=False, header=False)

    print(f"File {file_name} was Generated")
    print(f"File X_err_{file_name} was Generated")

    fig, ax = plt.subplots()
    ax.plot(np.linspace(0, len(err_arr)/100, len(err_arr)),err_arr)
    ax.set_title(f"{file_name.capitalize()} Arm Joint Errors")
    ax.set_ylabel('Error')
    ax.set_xlabel('Time (s)')
    ax.legend([r'$X_{Err1}$',r'$X_{Err2}$',r'$X_{Err3}$',r'$X_{Err4}$',r'$X_{Err5}$',r'$X_{Err6}$'])
    plt.savefig(f'{file_path}{file_name}.pdf')

if __name__ == "__main__":
    # Running Solutions with Configuration from Config File
    main(*conf_best(),'best',conf_t_best())
    main(*conf_overshoot(), 'overshoot', conf_t_overshoot())
    main(*conf_newTask(), 'newTask', conf_t_newTask())
    plt.show()