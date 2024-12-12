import numpy as np
from .init_state import conf_best, conf_overshoot, conf_newTask

# Trajectory Configuration for Best
def conf_t_best():
    # Initial Cube Configuration
    Tsc_i = np.array([[1, 0, 0, 1],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

    # Final Cube Configuration
    Tsc_f = np.array([[0, 1, 0, 0],
                      [-1, 0, 0, -1],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

    # End Effector Home Configuration in Space Frame
    Mse = conf_best()[0]

    # End Effector Angle
    ee_ang = np.sin(np.pi / 4)

    # Configuration of End Effector When Grasping
    T_grasp = np.array([[-ee_ang, 0, ee_ang, 0],
                        [0, 1, 0, 0],
                        [-ee_ang, 0, -ee_ang, 0],
                        [0, 0, 0, 1]])

    # Configuration of End Effector at Standoff
    T_standoff = np.array([[-ee_ang, 0, ee_ang, 0],
                           [0, 1, 0, 0],
                           [-ee_ang, 0, -ee_ang, 0.3],
                           [0, 0, 0, 1]])

    # Simulation Controller Refresh Rate
    k = 1

    return [Mse, Tsc_i, Tsc_f, T_grasp, T_standoff,k]

# Trajectory Configuration for Overshoot
def conf_t_overshoot():
    # Initial Cube Configuration
    Tsc_i = np.array([[1, 0, 0, 1],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

    # Final Cube Configuration
    Tsc_f = np.array([[0, 1, 0, 0],
                      [-1, 0, 0, -1],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

    # End Effector Home Configuration in Space Frame
    Mse = conf_overshoot()[0]

    # End Effector Angle
    ee_ang = np.sin(np.pi / 4)

    # Configuration of End Effector When Grasping
    T_grasp = np.array([[-ee_ang, 0, ee_ang, 0],
                        [0, 1, 0, 0],
                        [-ee_ang, 0, -ee_ang, 0],
                        [0, 0, 0, 1]])

    # Configuration of End Effector at Standoff
    T_standoff = np.array([[-ee_ang, 0, ee_ang, 0],
                           [0, 1, 0, 0],
                           [-ee_ang, 0, -ee_ang, 0.3],
                           [0, 0, 0, 1]])

    # Simulation Controller Refresh Rate
    k = 1

    return [Mse, Tsc_i, Tsc_f, T_grasp, T_standoff,k]

# Trajectory Configuration for New Task
def conf_t_newTask():
    # NEW Initial Cube Configuration
    Tsc_i = np.array([[1, 0, 0, 1],
                      [0, 1, 0, 2],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

    # NEW Final Cube Configuration
    Tsc_f = np.array([[0, 1, 0, -2],
                      [-1, 0, 0, -0.25],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

    # End Effector Home Configuration in Space Frame
    Mse = conf_newTask()[0]

    # End Effector Angle
    ee_ang = np.sin(np.pi / 4)

    # Configuration of End Effector When Grasping
    T_grasp = np.array([[-ee_ang, 0, ee_ang, 0],
                        [0, 1, 0, 0],
                        [-ee_ang, 0, -ee_ang, 0],
                        [0, 0, 0, 1]])

    # Configuration of End Effector at Standoff
    T_standoff = np.array([[-ee_ang, 0, ee_ang, 0],
                           [0, 1, 0, 0],
                           [-ee_ang, 0, -ee_ang, 0.3],
                           [0, 0, 0, 1]])

    # Simulation Controller Refresh Rate
    k = 1

    return [Mse, Tsc_i, Tsc_f, T_grasp, T_standoff,k]
