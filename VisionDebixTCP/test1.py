import matplotlib.pyplot as plt
import numpy as np

# Dummy coordinates for left and right lane boundaries
right_line = np.array([[200, 100], [200, 200], [200, 300]])

# Dummy finish line candidate
finish_line2 = np.array([[160, 150], [190, 150]])  # Horizontal line

# Midpoint of the finish line
finish_mid = (finish_line2[0] + finish_line2[1]) / 2

# Nearest left and right segments (simply using first two points from each boundary for illustration)
right_segment = np.array([right_line[1], right_line[2]])

# Vectors
finish_vec = finish_line2[1] - finish_line2[0]
right_vec = right_segment[1] - right_segment[0]

# Plotting
fig, ax = plt.subplots()
ax.plot(right_line[:, 0], right_line[:, 1], 'b-o', label='Right Boundary')
ax.plot(finish_line2[:, 0], finish_line2[:, 1], 'r-', label='Finish Line2', linewidth=2)
ax.plot(finish_mid[0], finish_mid[1], 'ko', label='Finish Line Midpoint')

# Draw vectors for clarity
def draw_vector(origin, vec, color, label):
    ax.arrow(origin[0], origin[1], vec[0], vec[1], head_width=5, head_length=5, fc=color, ec=color, label=label)

draw_vector(finish_line2[0], finish_vec, 'r', 'Finish Vec')
draw_vector(right_segment[0], right_vec, 'b', 'Right Vec')

ax.set_aspect('equal')
ax.set_xlim(80, 220)
ax.set_ylim(80, 320)
ax.invert_yaxis()
ax.legend()
ax.set_title("Finish Line Detection Vectors and Geometry")
plt.show()
