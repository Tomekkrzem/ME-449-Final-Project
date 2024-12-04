from src.NextState import NextState
import numpy as np
import pandas as pd

init_conf = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
const_control1 = np.array([-10,10,-10,10,0,0,0,0,0])
dt = 0.01
time = 10
sim_time = time / dt
g_state = 0

state = [init_conf]
trajectory = [[*state[0].tolist(),g_state]]

i = 1

for i in range(int(sim_time)-1):

    state.append(NextState(state[i-1],const_control1,dt,5))
    trajectory.append([*state[i], g_state])
    i = i + 1

# Code Used to Generate CSV Files
df = pd.DataFrame(trajectory)
file_name = 'milestone1'

# CHANGE FILE PATH TO GENERATE CSV PROPERLY
file_path = 'C:/Users/tomek/Downloads/School Work/ME 449/'

df.to_csv(str(file_path) + str(file_name) + '.csv', index=False, header=False)