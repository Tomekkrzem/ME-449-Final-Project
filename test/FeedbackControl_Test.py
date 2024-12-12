from src.FeedbackControl import FeedbackControl, Velocities
import numpy as np
import modern_robotics as mr
import pandas as pd

def main():
    X = np.array([[0.170,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.170,0.570],[0,0,0,1]])
    Xd = np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
    Xd_next = np.array([[0,0,1,0.6], [0,1,0,0], [-1,0,0,0.3], [0,0,0,1]])

    X_err_sum = np.array([0,0,0,0,0,0])

    Kp = 0
    Ki = 0
    dt = 0.01

    V = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt,X_err_sum)[0]

    B_list = np.array([[0,0,0,0,0],
                       [0,-1,-1,-1,0],
                       [1,0,0,0,1],
                       [0,-0.5076,-0.3526,-0.2176,0],
                       [0.033,0,0,0,0],
                       [0,0,0,0,0]])

    robot_conf = np.array([0,0,0,0,0,0.2,-1.6,0])

    M0e = np.array([[1,0,0,0.033],
                    [0,1,0,0],
                    [0,0,1,0.6546],
                    [0,0,0,1]])

    Tb0 = np.array([[1,0,0,0.1662],
                    [0,1,0,0],
                    [0,0,1,0.0026],
                    [0,0,0,1]])

    l = 0.235
    w = 0.15
    r = 0.0475

    F = (r / 4) * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                            [1, 1, 1, 1],
                            [-1, 1, -1, 1]])

    uO = Velocities(robot_conf,B_list,V,M0e,Tb0,F).tolist()

    print(list(map(lambda x: round(x, 1), uO)))

if __name__ == "__main__":
    main()