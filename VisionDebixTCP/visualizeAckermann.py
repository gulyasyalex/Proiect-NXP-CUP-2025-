import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# Create a 20x20 grid filled with 0s
grid = np.zeros((20, 20), dtype=int)

# Add a vertical line of 1s on column 5
grid[3:17, 5] = 1

# Add a vertical line of 2s on column 15 
grid[3:17, 15] = 2

# Plot the grid
fig, ax = plt.subplots(figsize=(8, 8))

# Define a custom colormap: 0 = light gray, 1 = dim gray, 2 = black
cmap = ListedColormap(['black', 'white', 'white'])

# Show the grid and flip the Y-axis
im = ax.imshow(grid, cmap=cmap, vmin=0, vmax=2, origin='lower')

# Display the number in each cell
for y in range(20):
    for x in range(20):
        val = grid[y, x]
        color = 'gray' if val == 0 else 'black'
        ax.text(x, y, str(val), ha='center', va='center', color=color)

# Configure axis ticks (no grid lines)
ax.set_xticks(np.arange(0,20,2))
ax.set_yticks(np.arange(0,20,2))
ax.tick_params(which='both', bottom=True, left=True, labelbottom=True, labelleft=True)

plt.show()
