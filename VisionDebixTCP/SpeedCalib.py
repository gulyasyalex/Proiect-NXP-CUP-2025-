# Re-import required libraries after environment reset
import numpy as np
import matplotlib.pyplot as plt

# Updated data including the (91 -> 0) constraint
servo_values_updated = np.array([
    90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100,
    101, 102, 103, 104, 105, 106, 107, 108,
    109, 110, 111, 112, 113, 114, 115,
    116, 117, 118, 119, 120
])

speeds_updated = np.array([
     0.00,  3.0,  6.0, 10.0, 20.0, 44.0, 59.0, 70.0, 83.0, 95.0, 107.0,
   120.0, 143.0, 155.0, 167.0, 180.0, 190.0, 200.0, 218.0,
   240.78, 264.83, 278.31, 288.97, 300.48, 311.17, 320.04,
   346.12, 350.28, 352.82, 361.57, 361.27
])

# Perform new quadratic fit
coeffs_updated = np.polyfit(servo_values_updated, speeds_updated, 2)

# Generate updated equation
updated_equation = f"Speed = {coeffs_updated[0]:.4f} * value^2 + {coeffs_updated[1]:.4f} * value + {coeffs_updated[2]:.4f}"

# Optional: plot updated fit
plt.scatter(servo_values_updated, speeds_updated, color='red', label='Data Points (Updated)')
x_fit_updated = np.linspace(min(servo_values_updated), max(servo_values_updated), 200)
y_fit_updated = np.polyval(coeffs_updated, x_fit_updated)
plt.plot(x_fit_updated, y_fit_updated, label='Updated Quadratic Fit')
plt.xlabel('Servo Write Value')
plt.ylabel('Speed (cm/s)')
plt.legend()
plt.title('Updated Servo Value vs Speed Quadratic Fit')
plt.grid(True)
plt.show()

print(updated_equation)
