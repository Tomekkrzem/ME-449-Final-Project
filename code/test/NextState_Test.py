from code.src.NextState import NextState
import numpy as np
import pandas as pd
from numpy import cos, sin

def main():
    # Initial Configuration
    init_conf = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
    # Constant Wheel Speeds and Joint Speeds (Spin in Place)
    const_control1 = np.array([-10,10,10,-10,0,0,0,0,0])
    # Time Step
    dt = 0.01
    # Time to Run for in Seconds
    time = 1
    # Overall Run Time of Simulation
    sim_time = time / dt
    # Gripper State
    g_state = 0

    # List of Configurations
    state = [init_conf]
    # List of Configurations Including Gripper State
    trajectory = [[*state[0].tolist(),g_state]]
    # Iteration Number
    i = 1

    # Loop Until Runtime is Done
    for i in range(int(sim_time)-1):
        # Update Configuration Lists
        phi, x, y = state[i-1][0:3]
        Tsb = np.array([[cos(phi), -sin(phi), 0, x],
                        [sin(phi), cos(phi), 0, y],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])

        state.append(NextState(state[i-1],const_control1,dt,10))
        trajectory.append([*state[i], g_state])
        i = i + 1

    # Code Used to Generate CSV Files
    df = pd.DataFrame(trajectory)
    file_name = 'milestone1'

    # CHANGE FILE PATH TO GENERATE CSV PROPERLY
    file_path = 'C:/Users/tomek/Downloads/School Work/ME 449/'

    df.to_csv(str(file_path) + str(file_name) + '.csv', index=False, header=False)

    print(f"File {file_name} was Generated")

if __name__ == "__main__":
    main()