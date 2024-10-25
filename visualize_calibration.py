import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


# Load the CSV file
df = pd.read_csv('rhcalib_output.csv', header=None)

# Extract x, y, z coordinates
x = df[0]
y = df[1]
z = df[2]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

colors = plt.cm.jet(np.linspace(0, 1, len(df)//20 + 1))

# Plot every 20th row with a different color
for i in range(0, len(df), 20):
    ax.scatter(x[i], y[i], z[i], c=[colors[i//20]], marker='o')


# Set labels
ax.set_xlabel('X Error')
ax.set_ylabel('Y Error')
ax.set_zlabel('Z Error')

# Show the plot
plt.show()