import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import curve_fit
from scipy.interpolate import splrep, splev

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Your list of poses
poses = [
    np.array([0.78726253,  0.34454266, -0.71004344]),
    np.array([0.76453513,  0.34536993, -0.70425916]),
    np.array([0.72300046,  0.35044268, -0.70193804]),
    np.array([0.7027234 ,  0.35031719, -0.6948665]),
    np.array([0.68187972,  0.35343439, -0.69521656]),
    np.array([0.66169145,  0.35416544, -0.69135061]),
]

# Extract x, y, z values for each pose
x, y, z = zip(*poses)

# # Ensure x values are in ascending order
# sorted_indices = np.argsort(x)
# x = np.array(x)[sorted_indices]
# y = np.array(y)[sorted_indices]
# z = np.array(z)[sorted_indices]

# # Remove duplicate x values
# unique_indices = np.unique(x, return_index=True)[1]
# x = x[unique_indices]
# y = y[unique_indices]
# z = z[unique_indices]

# # Use cubic spline interpolation for each dimension
# try:
#     tck_x = splrep(x, z, s=0)
#     tck_y = splrep(x, y, s=0)
#     # Evaluate the spline over a finer range
#     x_fine = np.linspace(min(x), max(x), 100)
#     y_fine = splev(x_fine, tck_y)
#     z_fine = splev(x_fine, tck_x)

#     # Plot the data points
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.scatter(x, y, z, c='blue', marker='o', label='Data')

#     # Plot the fitted spline
#     ax.plot(x_fine, y_fine, z_fine, color='red', label='Fitted Spline')

#     # Set axis labels
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')

#     # Set plot title
#     ax.set_title('Fitted 3D Spline to Data')

#     # Show the plot
#     plt.show()

# except ValueError as e:
#     print(f"Spline interpolation failed: {e}")

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import curve_fit

# Your list of poses


# Define a 3D curve function
def curve_function(t, a, b, c, d, e, f):
    x = a * np.cos(b * t)
    y = c * np.sin(d * t)
    z = e * t + f
    return np.column_stack((x, y, z)).flatten()

# Parametrize the curve with t (you can choose a suitable range)
t = np.linspace(0, 5, len(x))

# Perform the fit using curve_fit
params, covariance = curve_fit(curve_function, t, np.column_stack((x, y, z)).flatten())

# Extract the fitted parameters
a, b, c, d, e, f = params

# Print the fitted parameters
print(f"Fitted Parameters: a = {a}, b = {b}, c = {c}, d = {d}, e = {e}, f = {f}")

# Evaluate the curve over a finer range
t_fine = np.linspace(min(t), max(t), 100)
curve_fine = np.array([curve_function(ti, a, b, c, d, e, f) for ti in t_fine])

# Plot the data points
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, c='blue', marker='o', label='Data')

# Plot the fitted curve
ax.plot(curve_fine[:, 0], curve_fine[:, 1], curve_fine[:, 2], color='red', label='Fitted Curve')

# Set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set plot title
ax.set_title('Fitted 3D Curve to Data')

# Show the plot
plt.show()

