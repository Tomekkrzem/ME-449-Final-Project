import modern_robotics as mr
import numpy as np
from tqdm import tqdm

def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k):

    # Parameters:
    # Tse_init = Initial Configuration of End Effector in Space Frame
    # Tsc_init = Initial Configuration of Cube in Space Frame
    # Tsc_fin = Final Configuration of Cube in Space Frame
    # Tce_grasp = Position of End Effector Relative to the Cube During Grasping
    # Tce_standoff = Position of End Effector Relative to the Cube at Stand Off Position

    # Output:
    # full_traj = N x 13 Matrix Representing End Effector Position
    #
    #           = [r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,g]
    #

    # Time Step
    dt = 0.01

    # End Effector Stand Off Position In Space
    Tse_standoff1 = np.matmul(Tsc_initial,Tce_standoff)
    Tse_standoff2 = np.matmul(Tsc_final, Tce_standoff)

    # End Effector Grasp Position In Space
    Tse_grasp1 = np.matmul(Tsc_initial,Tce_grasp)
    Tse_grasp2 = np.matmul(Tsc_final, Tce_grasp)

    # Empty Trajectory
    full_traj = []

    # Time Configurations for Trajectories
    T1 = 2          # Time Config for First Long Motion
    TL = 1          # Time Config for Linear Standoff Motion
    TG = 0.63       # Time Config for Grasping Motion
    T2 = 5          # Time Config for Second Long Motion

    # Time Scaling Method
    m = 3

    # Trajectory Sequence List
    traj_seq = [[Tse_initial,Tse_standoff1,T1,(T1*k)/dt],
                [Tse_standoff1, Tse_grasp1,TL,(TL*k)/dt],
                [Tse_grasp1, Tse_grasp1,TG,(TG*k)/dt],
                [Tse_grasp1, Tse_standoff1,TL,(TL*k)/dt],
                [Tse_standoff1, Tse_standoff2,T2,(T2*k)/dt],
                [Tse_standoff2, Tse_grasp2,TL,(TL*k)/dt],
                [Tse_grasp2, Tse_grasp2,TG,(TG*k)/dt],
                [Tse_grasp2, Tse_standoff2,TL,(TL*k)/dt]]

    # Processing Each Trajectory Sequence and Adding them to Full Trajectory
    for i in range(len(traj_seq)):
        s = mr.CartesianTrajectory(*traj_seq[i],m)
        if i < 2 or i > 6: g = 0
        else: g = 1
        for out in tqdm(s):
            out = out.tolist()
            r1_3 = out[0][:3]
            r2_3 = out[1][:3]
            r3_3 = out[2][:3]
            full_traj.append([*r1_3 ,*r2_3 ,*r3_3 ,out[0][3] ,out[1][3] ,out[2][3] ,g])

    return full_traj

