import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load simulation data
try:
    df = pd.read_csv('simulation_results.csv')
except FileNotFoundError:
    print("Error: simulation_results.csv not found. Please run the C++ simulation first.")
    exit(1)

t = df['t'].values

# State extractions
x11 = df[['x11_1', 'x11_2']].values
x12 = df[['x12_1', 'x12_2']].values
x21 = df[['x21_1', 'x21_2']].values
x22 = df[['x22_1', 'x22_2']].values
x31 = df[['x31_1', 'x31_2']].values
x32 = df[['x32_1', 'x32_2']].values
x41 = df[['x41_1', 'x41_2']].values
x42 = df[['x42_1', 'x42_2']].values

# Recreate Leader state (x01)
A = 0.2
w = 0.4
x01_1 = A * np.cos(w * np.pi * t)
x01_2 = A * np.sin(w * np.pi * t)

# ---------- PLOTTING ----------

# 1. Plot x01 and agent position states (x_i1)
fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Dimension 1 (p=1)
ax1.plot(t, x01_1, 'b-', label='x01')
ax1.plot(t, x11[:, 0], 'r-', label='x11')
ax1.plot(t, x21[:, 0], 'g-', label='x21')
ax1.plot(t, x31[:, 0], 'k-', label='x31')
ax1.plot(t, x41[:, 0], '-', label='x41')
ax1.set_title("Trajectories, p=1")
ax1.legend()
ax1.grid(True)

# Dimension 2 (p=2)
ax2.plot(t, x01_2, 'b-', label='x01')
ax2.plot(t, x11[:, 1], 'r-', label='x11')
ax2.plot(t, x21[:, 1], 'g-', label='x21')
ax2.plot(t, x31[:, 1], 'k-', label='x31')
ax2.plot(t, x41[:, 1], '-', label='x41')
ax2.set_title("Trajectories, p=2")
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.savefig('trajectories.png')
print("Generated trajectories.png")

# 2. Tracking errors (delta) and performance bounds
p_bound = (0.8 - 0.01) * np.exp(-t) + 0.01

fig2, (ax3, ax4) = plt.subplots(2, 1, figsize=(10, 8))

# Error agent 1
delta11_1 = x11[:, 0] - x01_1
delta11_2 = x11[:, 1] - x01_2

ax3.plot(t, delta11_1, 'b-')
ax3.plot(t, p_bound, 'r--')
ax3.plot(t, -p_bound, 'r--')
ax3.set_title("Tracking Error \u03b41,1, p=1")
ax3.grid(True)

ax4.plot(t, delta11_2, 'b-')
ax4.plot(t, p_bound, 'r--')
ax4.plot(t, -p_bound, 'r--')
ax4.set_title("Tracking Error \u03b41,1, p=2")
ax4.grid(True)

plt.tight_layout()
plt.savefig('tracking_error_agent1.png')
print("Generated tracking_error_agent1.png")

# 3. Tracking error - Agent 2
delta21_1 = x21[:, 0] - x01_1
delta21_2 = x21[:, 1] - x01_2

fig3, (ax5, ax6) = plt.subplots(2, 1, figsize=(10, 8))

ax5.plot(t, delta21_1, 'b-')
ax5.plot(t, p_bound, 'r--')
ax5.plot(t, -p_bound, 'r--')
ax5.set_title("Tracking Error \u03b42,1, p=1")
ax5.grid(True)

ax6.plot(t, delta21_2, 'b-')
ax6.plot(t, p_bound, 'r--')
ax6.plot(t, -p_bound, 'r--')
ax6.set_title("Tracking Error \u03b42,1, p=2")
ax6.grid(True)

plt.tight_layout()
plt.savefig('tracking_error_agent2.png')
print("Generated tracking_error_agent2.png")

# 4. Tracking error - Agent 3
delta31_1 = x31[:, 0] - x01_1
delta31_2 = x31[:, 1] - x01_2

fig4, (ax7, ax8) = plt.subplots(2, 1, figsize=(10, 8))

ax7.plot(t, delta31_1, 'b-')
ax7.plot(t, p_bound, 'r--')
ax7.plot(t, -p_bound, 'r--')
ax7.set_title("Tracking Error \u03b43,1, p=1")
ax7.grid(True)

ax8.plot(t, delta31_2, 'b-')
ax8.plot(t, p_bound, 'r--')
ax8.plot(t, -p_bound, 'r--')
ax8.set_title("Tracking Error \u03b43,1, p=2")
ax8.grid(True)

plt.tight_layout()
plt.savefig('tracking_error_agent3.png')
print("Generated tracking_error_agent3.png")

# 5. Tracking error - Agent 4
delta41_1 = x41[:, 0] - x01_1
delta41_2 = x41[:, 1] - x01_2

fig5, (ax9, ax10) = plt.subplots(2, 1, figsize=(10, 8))

ax9.plot(t, delta41_1, 'b-')
ax9.plot(t, p_bound, 'r--')
ax9.plot(t, -p_bound, 'r--')
ax9.set_title("Tracking Error \u03b44,1, p=1")
ax9.grid(True)

ax10.plot(t, delta41_2, 'b-')
ax10.plot(t, p_bound, 'r--')
ax10.plot(t, -p_bound, 'r--')
ax10.set_title("Tracking Error \u03b44,1, p=2")
ax10.grid(True)

plt.tight_layout()
plt.savefig('tracking_error_agent4.png')
print("Generated tracking_error_agent4.png")

plt.show()
