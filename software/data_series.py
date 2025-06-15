import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import glob


THRESHOLD= 4.0
EXPERIMENT = "peg_in_hole_raph"
data_paths = glob.glob(f".//software//output//{EXPERIMENT}//*.csv")
timing_data = []

def get_timing_data(path):
    df = pd.read_csv(path)
    targets = df[['Target_x','Target_y']].drop_duplicates().to_numpy()
    start_index = len(df[(df['Target_x'] == targets[0][0]) & (df['Target_y'] == targets[0][1])])
    block = df[['Block_x','Block_y']]
    block = block[start_index:].to_numpy()
    target_data = []
    output = []
    for target in targets:
        target_data.append(df[(df['Target_x'] == target[0]) & (df['Target_y'] == target[1])])

    for target in target_data:
        start_time = target.iloc[0]['Timestamp']
        distance = np.linalg.norm(target[['Block_x','Block_y']].to_numpy()-target[['Target_x','Target_y']].to_numpy(),axis=1)
        completed_index = np.argmax(distance<THRESHOLD)
        end_time = target.iloc[completed_index]['Timestamp']
        output.append(((end_time-start_time),np.min(distance)))

    return output


for path in data_paths:
    data = get_timing_data(path)
    timing_data.append((data[-1][0],get_timing_data(path)[-1][1]))

plt.plot(timing_data)
plt.show()