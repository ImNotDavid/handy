import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
from scipy import interpolate

df = pd.read_csv('software/output/data_4.csv')
targets = df[['Target_x','Target_y']].drop_duplicates().to_numpy()
start_index = len(df[(df['Target_x'] == targets[0][0]) & (df['Target_y'] == targets[0][1])])
block = df[['Block_x','Block_y']]
block = block[start_index:].to_numpy()
target_data = []

for target in targets:
    target_data.append(df[(df['Target_x'] == target[0]) & (df['Target_y'] == target[1])])

for target in target_data:
    distance = np.linalg.norm(target[['Block_x','Block_y']].to_numpy()-target[['Target_x','Target_y']].to_numpy(),axis=1)
    print(np.min(distance))
#block = block.reshape((block.shape[1],block.shape[0]))
mask = np.concatenate([[True], np.any(block[1:] != block[:-1], axis=1)])

# Apply the mask
filtered = block[mask]
x, y = zip(*filtered)
x = np.r_[x, x[0]]
y = np.r_[y, y[0]]
f, u = interpolate.splprep([x, y], s=0, per=True)
xint, yint = interpolate.splev(np.linspace(0, 1, 3000), f)
plt.scatter(targets[:,0],-targets[:,1],c='r')
plt.plot(xint, -yint)
plt.show()

