import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import glob


THRESHOLD= 4.0
EXPERIMENT = "orientation_david"
data_paths = glob.glob(f".//software//output//{EXPERIMENT}//*.csv") + glob.glob(f".//software//output//{EXPERIMENT}_large//*.csv")
orientation_data = []

def get_accuracy_data(path):
    df = pd.read_csv(path)
    targets = df[['Target_orientation']].drop_duplicates().to_numpy()
    start_index = len(df[ (df['Target_orientation'] == targets[0][0]) ])
    block = df[['Block_orientation']]
    block = block[start_index:].to_numpy()
    target_data = []
    for target in targets:
        target_data.append(df[(df['Target_orientation'] == target[0])])

    target = target_data[-1]
    target_orientation = targets[-1]
    distance = np.linalg.norm(target[['Block_orientation']].to_numpy()-target[['Target_orientation']].to_numpy(),axis=1)
    return(target_orientation[0],np.min(distance))



for path in data_paths:
    data = get_accuracy_data(path)
    orientation_data.append((data[0],data[1]))

orientation_data = np.array(orientation_data)
orientation_data = orientation_data.swapaxes(0,1)
plt.scatter(orientation_data[0],orientation_data[1])
plt.show()