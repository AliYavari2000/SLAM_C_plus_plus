import numpy as np
import matplotlib.pyplot as plt

# Load robot error log
i_r, err_x2, err_y2, err_theta = np.loadtxt("robot_error.txt", unpack=True)

# Load landmark error log
i_l, err_lx2, err_ly2 = np.loadtxt("landmark_error.txt", unpack=True)

# --- Plot Robot Error ---
plt.figure(figsize=(10, 6))
plt.plot(i_r, err_x2, label='Error X²')
plt.plot(i_r, err_y2, label='Error Y²')
plt.plot(i_r, err_theta, label='Error Θ')
plt.title("Robot State Error Over Time")
plt.xlabel("Iteration")
plt.ylabel("Squared Error")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Plot Landmark Error ---
plt.figure(figsize=(10, 6))
plt.plot(i_l, err_lx2, label='Landmark Error X²')
plt.plot(i_l, err_ly2, label='Landmark Error Y²')
plt.title("Landmark Position Error Over Time")
plt.xlabel("Iteration")
plt.ylabel("Squared Error")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
