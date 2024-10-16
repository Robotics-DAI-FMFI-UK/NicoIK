import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

calib_pose = ['right low tscreen','right tscreen edge1','right tscreen edge2','right tscreen edge3','tscreen place1',
            'touchscreen place1','middle low tscreen','topleftfront body','left eye','nose','right eye','right head','top head']

calib_poser = ['leftlow tscreen','left tcreen e1','left tscreen e2','bottom tscreen e1','bottom tscreen e2 ',
            'middlelow tscreen','bot. tscreen edge 3', 'topright body ',
            'bottomright body', 'leftinner hole','rightinner hole ','mark sign middle']


# Load the CSV file
df = pd.read_csv('lhcalib_output.csv', header=None)

# Extract x, y, z coordinates
x = df[0]
y = df[1]
z = df[2]

# Create a figure for the boxplots
fig, axs = plt.subplots(3, 1, figsize=(10, 15))

# Plot boxplots for every 20 values
for i in range(0, len(df), 20):
    end = i + 20 if i + 20 < len(df) else len(df)
    axs[0].boxplot(x[i:end], positions=[i//20], widths=0.6)
    axs[1].boxplot(y[i:end], positions=[i//20], widths=0.6)
    axs[2].boxplot(z[i:end], positions=[i//20], widths=0.6)

# Set y-ticks from calib_poser
num_boxes = len(range(0, len(df), 20))
axs[0].set_xticks(range(num_boxes))
axs[0].set_xticklabels(calib_poser[:num_boxes])

axs[1].set_xticks(range(num_boxes))
axs[1].set_xticklabels(calib_poser[:num_boxes])

axs[2].set_xticks(range(num_boxes))
axs[2].set_xticklabels(calib_poser[:num_boxes])

# Set y-axis limits
axs[0].set_ylim(-0.037, -0.015)
axs[1].set_ylim(-0.025, 0.025)
axs[2].set_ylim(-0.0125, 0.00)


# Set labels
axs[0].set_title('X axis error')
axs[0].set_ylabel('X Error in meters')

axs[1].set_title('Y axis error')
axs[1].set_ylabel('Y Error in meters')

axs[2].set_title('Z axis error')
axs[2].set_xlabel('Position')
axs[2].set_ylabel('Z Error in meters')

# Show the plot
plt.tight_layout()
plt.show()