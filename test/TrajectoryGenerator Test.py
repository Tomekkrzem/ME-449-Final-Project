from src.TrajectoryGenerator import TrajectoryGenerator
import numpy as np
import pandas as pd

Tsb = np.array([[1,0,0,0],
                [0,1,0,0],
                [0,0,1,0.0963],
                [0,0,0,1]])

Tb0 = np.array([[1,0,0,0.1662],
                [0,1,0,0],
                [0,0,1,0.0026],
                [0,0,0,1]])

M0e = np.array([[1,0,0,0.033],
                [0,1,0,0],
                [0,0,1,0.6546],
                [0,0,0,1]])

Mse =  np.matmul(Tsb,np.matmul(Tb0,M0e))

# Given Cube Test
Tsc_i = np.array([[1,0,0,1],
                  [0,1,0,0],
                  [0,0,1,0.025],
                  [0,0,0,1]])

Tsc_f = np.array([[0,1,0,0],
                  [-1,0,0,-1],
                  [0,0,1,0.025],
                  [0,0,0,1]])

ee_ang = np.sin(np.pi/4)

T_grasp = np.array([[-ee_ang,0,ee_ang,0],
                    [0,1,0,0],
                    [-ee_ang,0,-ee_ang,0],
                    [0,0,0,1]])

T_standoff = np.array([[-ee_ang,0,ee_ang,0],
                       [0,1,0,0],
                       [-ee_ang,0,-ee_ang,0.15],
                       [0,0,0,1]])

t = TrajectoryGenerator(Mse,Tsc_i,Tsc_f,T_grasp,T_standoff,10)

# Code Used to Generate CSV Files
df = pd.DataFrame(t)
file_name = 'test'

# CHANGE FILE PATH TO GENERATE CSV PROPERLY
file_path = 'C:/Users/tomek/Downloads/School Work/ME 449/'

df.to_csv(str(file_path) + str(file_name) + '.csv', index=False, header=False)