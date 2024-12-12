from code.src.TrajectoryGenerator import TrajectoryGenerator
from code.config import conf_t_best
import pandas as pd

def main():
    # Complete Trajectory Generated Using TrajectoryGenerator
    t = TrajectoryGenerator(*conf_t_best())

    # Code Used to Generate CSV Files
    df = pd.DataFrame(t)
    file_name = 'milestone2'

    # CHANGE FILE PATH TO GENERATE CSV PROPERLY
    file_path = 'C:/Users/tomek/Downloads/School Work/ME 449/Krzeminski_Tomasz_milestone2/'

    df.to_csv(str(file_path) + str(file_name) + '.csv', index=False, header=False)

if __name__ == "__main__":
    main()