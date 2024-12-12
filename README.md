# ME449 Robotic Manipulation

## Project Overview
This is the final project for ME449 Robotic Manipulation in which the goal was to generate trajectories for a KUKA Youbot to follow and move a cube in a CoppeliaSim V-REP Scene called "Scene6_youBot_cube.ttt". The project can be found [here](https://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone_2024).

The general procedure of the program is as follows:
1. Generate Trajectories given cube configurations and starting end effector configuration.
2. Iterate through reference trajectories.
3. Compute Feedforward Control using the kinematic task-space feedforward plus feedback control law:
   {\displaystyle {\mathcal {V}}(t)=[{\text{Ad}}_{X^{-1}X_{d}}]{\mathcal {V}}_{d}(t)+K_{p}X_{\text{err}}(t)+K_{i}\int _{0}^{t}X_{\text{err}}({\text{t}})d{\text{t}}.}
4. Compute the Next State configurations of the youBot.
5. Generate all required files.

## Directory Overview
```
/code/
  /config/
    __init__.py: Config Constructor
    init_state.py: All Initial State Configurations for best, overshoot, and newTask
    init_traj.py: All Trajectory Configurations for best, overshoot, and newTask
  /src/
    __init__.py: Src Constructor
    Nextstate.py: Milestone 1
    TrajectoryGenerator.py: Milestone 2
    FeedbackControl.py: Milestone 3
  /test/
    __init__.py: Test Constructor
    Nextstate_Test.py: Test for Milestone 1
    TrajectoryGenerator_Test.py: Test for Milestone 2
    FeedbackControl_Test.py: Test for Milestone 3
  __init__.py: Main Constructor
  main.py: Full Program
/result/
  /best/
    *Files pertaining to outputs for best
  /overshoot/
    *Files pertaining to outputs for overshoot
  /newTask/
    *Files pertaining to outputs for newTask
```

## Milestone Overview

### Milestone 1
The focus of milestone 1 was generating the next state of the youBot given a current configuration, current wheel and joint velocities, timestep, and velocity limits.

### Milestone 2
The focus of milestone 2 was generating reference trajectories given initial end effector position, initial and final cube configurations, end effector configuration when grasping, end effector stand-off configuration, and a simulation frequency.

### Milestone 3
The focus of milestone 3 was computing the feedforward controls which were then fed into milestone 1. This requires the current end effector configuration, desired configuration, next desired configuration, proportional gain, integral gain, time step, and the error integral value.

## Result (best)
Utilizes optimized Feedforward-PI controller to move a cube from Initial to Final Configuration (Cube Config is Default to CoppeliaSim Scene 6).

### Configurations:
- **Kp** = 30
- **Ki** = 0.2

### Timestep:
- **dt** = 0.01

### Velocity Limits:
- **v_lim** = 15

### Robot Configuration:
```
R_config = [0.4, -0.4, -0.2, 0, 0, 1, -1.8, 0, 0, 0, 0, 0]
```

### Standard Initial Cube Configuration:
```
Tsc_i = np.array([[1, 0, 0, 1],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])
```

### Standard Final Cube Configuration:
```
Tsc_f = np.array([[0, 1, 0, 0],
                  [-1, 0, 0, -1],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])
```

### Best X_err Plot:
![image](https://github.com/user-attachments/assets/52d6d1bb-834c-4af8-8ea3-c8565de7979f)

### Simulation:
![best](https://github.com/user-attachments/assets/375bb12c-9320-4ce3-a0c7-703afa4a6e2e)

## Result (overshoot)
Utilizes un-optimized Feedforward-PI controller to move a cube from Initial to Final Configuration with overshoot (Cube Config is Default to CoppeliaSim Scene 6).

### Configurations:
- **Kp** = 3
- **Ki** = 3.01

### Timestep:
- **dt** = 0.01

### Velocity Limits:
- **v_lim** = 12

### Robot Configuration:
```
R_config = [0.1, -0.5, -0.5, 0, 1, 0, -1.2, 0, 0, 0, 0, 0]
```

### Standard Initial Cube Configuration:
```
Tsc_i = np.array([[1, 0, 0, 1],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])
```

### Standard Final Cube Configuration:
```
Tsc_f = np.array([[0, 1, 0, 0],
                  [-1, 0, 0, -1],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])
```

### Overshoot X_err Plot:
![image](https://github.com/user-attachments/assets/4aedcab9-07bd-4c53-aadd-72e288df5f47)

### Simulation:
![overshoot](https://github.com/user-attachments/assets/8914799a-4386-45b4-a5c5-6e349f128e0d)

## Result (newTask)
Utilizes Optimized Feedforward-PI controller to move a cube from Initial to Final Configuration (Cube Config is Custom).

### Configurations:
- **Kp** = 3
- **Ki** = 0.1

### Timestep:
- **dt** = 0.01

### Velocity Limits:
- **v_lim** = 25

### Robot Configuration:
```
R_config = [1.5, 0.8, 0.2, 0, 0, 0.3, -0.7, 0, 0, 0, 0, 0]
```

### Custom Initial Cube Configuration:
```
Tsc_i = np.array([[1, 0, 0, 1],
                  [0, 1, 0, 2],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])
```

### Custom Final Cube Configuration:
```
Tsc_f = np.array([[ 0, 1, 0, -2],
                  [-1, 0, 0, -0.25],
                  [ 0, 0, 1,  0.025],
                  [ 0, 0, 0,  1]])
```

### New Task X_err Plot:
![image](https://github.com/user-attachments/assets/adb53d28-1251-4860-8190-33ba54ed1cf3)

### Simulation:
![newTask](https://github.com/user-attachments/assets/539ad327-f9bb-4308-892d-910dbca9c71c)
