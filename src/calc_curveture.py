import numpy as np
import matplotlib.pyplot as plt

# Given parameters
frequency = 0.127  # Hz
amplitude = 1.5  # m

# Angular frequency (omega)
omega = 2 * np.pi * frequency

# Define time range for plotting
t = np.linspace(0, 10, 1000)

# Define the sinusoidal wave
y = amplitude * np.sin(omega * t)

# First and second derivatives (dy/dx and d2y/dx2)
y_prime = amplitude * omega * np.cos(omega * t)
y_double_prime = -amplitude * omega**2 * np.sin(omega * t)

# Curvature formula: kappa = |y''| / (1 + (y')^2)^(3/2)
curvature = np.abs(y_double_prime) / (1 + y_prime**2)**(3/2)

# Plot the curvature
plt.figure(figsize=(10, 6))
plt.plot(t, curvature, label="Curvature")
plt.title("Curvature of the Sine Wave (Frequency = 0.254 Hz, Amplitude = 1.5 m)")
plt.xlabel("Time (s)")
plt.ylabel("Curvature (1/m)")
plt.grid(True)
plt.legend()
plt.show()

# Return the maximum curvature
max_curvature = np.max(curvature)
max_curvature
