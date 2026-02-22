# Multi-Agent Prescribed Performance Control (PPC) Engine

## Overview

This repository contains a high-performance **C++ simulation engine** for the Prescribed Performance Control (PPC) of nonlinear Multi-Agent Systems (MAS) under discrete-time communication constraints.

Originally prototyped in MATLAB for academic research, this project was entirely re-architected into a **C++ hybrid simulation environment**. It decouples the discrete-time communication updates from the continuous-time physical dynamics, ensuring numerical stability when solving the stiff ordinary differential equations (ODEs) that govern the system.

## Core Technologies

- **C++17**: Core engine language.
- **Eigen3**: Utilized for heavily optimized, vectorized linear algebra operations and spatial transformations.
- **Boost.Numeric.Odeint**: Handles the continuous integration of the system's dynamics (utilizing an adaptive Runge-Kutta Dopri5 stepper).
- **CMake**: Build system generation and dependency linking.

## Architecture Highlights

Unlike standard academic simulations that mix discrete and continuous states, this engine uses a **Hybrid Loop Architecture**:

- **Discrete Event Handling**: Neighbor states and network topology data are updated strictly at the communication interval $T_{mati}$. During the continuous integration phase $t \in [t_k, t_{k+1})$, the received states are held as step-wise constant values in memory, accurately simulating the physical reality of a discrete-time sensor network.
- **Continuous Integration**: Between communication pulses, the physical system dynamics and barrier functions are integrated using highly accurate adaptive step sizing.
- **Barrier Functions**: The control law utilizes nonlinear barrier functions (tangent transformations) to mathematically guarantee that position and velocity tracking errors never violate the dynamically decaying prescribed performance bounds.

### Hybrid Continuous/Discrete Simulation

A key aspect of this simulation is the separation between **continuous local dynamics** and **discrete agent communication**:

- Each agent's **own state** $(x_{i,1}, x_{i,2})$ evolves **continuously** — the ODE integrator advances these states at every sub-step with full numerical precision.
- The **neighbor reference signals** (received states from other agents) are only transmitted at discrete communication instants $t_k = k \cdot T_{mati}$ and are **held constant** until the next communication event at $t_{k+1}$.

This means that during the interval $[t_k, t_{k+1})$, each agent computes its control law using its own **live, continuously evolving state** combined with the **frozen snapshot** of its neighbors' states from time $t_k$. For example:

```
Agent 1:  e1 = x11(t)  −  x21_hold(t_k)
                ↑                ↑
          continuous            held
```

This faithfully models real-world networked control systems where sensors transmit data only at sampling instants, while each agent's internal dynamics continue to evolve between transmissions. The stability analysis of the PPC controller under these conditions is a central objective of the simulation.

## System Model

### Differential Equations
Each follower agent $i \in \{1, 2, 3, 4\}$ has 2nd-order nonlinear dynamics described by:
```math
\begin{align*}
\dot{x}_{i,1} &= x_{i,2} + f_{i,1}(\bar{x}_i) \\
\dot{x}_{i,2} &= f_{i,2}(\bar{x}_i) + G_i(\bar{x}_i) u_i + z(t)
\end{align*}
```
Where $\bar{x}_i = [x_{i,1}, x_{i,2}]^T$ is the full state of agent $i$, $u_i$ is the control input computed by the Discrete-Time PPC law, and $z(t)$ is a common time-varying disturbance acting on all agents. 

The specific nonlinear functions ($f_{i,1}, f_{i,2}, G_i$) used for the strictly heterogeneous agents in this simulation are:

**Agent 1:**
- $f_{1,1} = [\sin(x_{11,1}), \cos(x_{11,2})]^T$
- $f_{1,2} = [-2x_{12,1} - x_{12,2}, x_{11,1}^2]^T$
```
G1 = [ x11_1^2 + 1,   cos(x11_2) ]
     [ sin(x12_1),    x12_2^2 + 4 ]
```

**Agent 2:**
- $f_{2,1} = [2x_{21,1}, x_{21,2}^3]^T$
- $f_{2,2} = [x_{21,1}^2, x_{22,2}^2 - x_{21,1}^5]^T$
```
G2 = [ x21_1^2 + 1,   sin(x21_2) ]
     [ cos(x22_1),    x22_2^2 + 1 ]
```

**Agent 3:**
- $f_{3,1} = [\cos^2(x_{31,1}), x_{31,1}^2]^T$
- $f_{3,2} = [\sin(x_{32,2}) + x_{32,1}, 5x_{32,1}^3]^T$
```
G3 = [ x31_1^2 + 1,   cos(x31_1) ]
     [ sin(x31_2),    x31_1^2 + 3 ]
```

**Agent 4:**
- $f_{4,1} = [\sin^2(x_{41,1}), x_{41,1}^2]^T$
- $f_{4,2} = [\cos(x_{42,2}) + x_{42,1}, 5x_{42,1}^2]^T$
```
G4 = [ x41_1^2 + 2,   cos(x41_1) ]
     [ sin(x41_2),    x41_1^2 + 3 ]
```

### Initial Conditions
The simulation initializes the 16 continuous ODE states natively (vector `x[16]` in `main.cpp`) mapping to $x_{i,1}$ and $x_{i,2}$ components:

- **Agent 1:** $x_{1,1}(0) = [-0.1, -0.2]^T$, $x_{1,2}(0) = [0.0, 0.0]^T$
- **Agent 2:** $x_{2,1}(0) = [0.15, 0.1]^T$, $x_{2,2}(0) = [0.0, 0.0]^T$
- **Agent 3:** $x_{3,1}(0) = [0.0, 0.0]^T$, $x_{3,2}(0) = [0.0, 0.0]^T$
- **Agent 4:** $x_{4,1}(0) = [0.1, -0.3]^T$, $x_{4,2}(0) = [0.0, 0.0]^T$

The leader tracking trajectory follows the harmonic oscillator initialization dynamically over time: $x_{0,1}(t) = [0.2 \cos(0.4\pi t), 0.2 \sin(0.4\pi t)]^T$.

## Prerequisites (Windows)

To compile this C++ project on Windows, you will need a C++ compiler (GCC), CMake, and the mathematical library headers (Eigen3 & Boost). The easiest way to get all of this is via **MSYS2**.

### 1. Install MSYS2
1. Download the installer from the official website: [msys2.org](https://www.msys2.org/)
2. Run the installer and follow the default installation instructions.

### 2. Install the Required Packages
Once installed, open the **MSYS2 UCRT64** terminal from your Windows Start Menu (make sure it's the `UCRT64` one, not MSYS or MINGW64).

Run the following command to download and install GCC, CMake, Make, Eigen3, and Boost in one go:
```bash
pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-make mingw-w64-ucrt-x86_64-eigen3 mingw-w64-ucrt-x86_64-boost
```
Press `Enter` when it asks if you want to proceed.

### 3. Add to Windows PATH
To allow PowerShell and Windows to see the newly installed tools:
1. Open the Windows Start menu, type **"Environment Variables"** and select **"Edit the system environment variables"**.
2. Click the **"Environment Variables..."** button at the bottom.
3. Under "User variables", find the `Path` variable, select it, and click **Edit...**.
4. Click **New** and paste exactly: `C:\msys64\ucrt64\bin`
5. Click OK on all windows to save and exit.

*Note: You may need to restart your terminal (or VS Code) for the PATH changes to take effect.*

---

## Building the Simulation

Open a PowerShell terminal in your project directory (`cpp_ppc`). Run the following CMake commands to generate the build files and compile the executable:

```powershell
# 1. Generate the build environment
cmake -B build -G "MinGW Makefiles"

# 2. Compile the executable
cmake --build build
```

This will create `PPC_Simulation.exe` inside the `build` folder.

## Running the Simulation

Execute the compiled program from PowerShell:
```powershell
.\build\PPC_Simulation.exe
```
You should see output indicating the integration loop has started. When finished, it will generate a large `simulation_results.csv` file consisting of ~87,000 steps of trajectory logging.

## Plotting the Results
A Python script (`plot_results.py`) is provided to read the CSV file and recreate the MATLAB visualization plots. 

Make sure you have Python installed, as well as the `pandas` and `matplotlib` packages:
```powershell
pip install pandas matplotlib numpy
```

Run the visualizer:
```powershell
python plot_results.py
```
This will display the graphs and automatically save them as `trajectories.png` and `tracking_error_agent1.png` in the root folder.
