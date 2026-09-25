# Tractor-Trailer Trajectory Planning in Extremely Narrow Environments

MATLAB/AMPL case-study implementation for trajectory planning of articulated tractor-trailer vehicles in extremely constrained environments.

This repository accompanies the paper:

> **B. Li, Y. Zhang, T. Acarman, Q. Kong, and Y. Zhang**,  
> “Trajectory Planning for a Tractor with Multiple Trailers in Extremely Narrow Environments: A Unified Approach,”  
> in *2019 International Conference on Robotics and Automation (ICRA)*, Montreal, Canada, 2019, pp. 8557–8562.  
> DOI: [10.1109/ICRA.2019.8793955](https://doi.org/10.1109/ICRA.2019.8793955)

If you use this repository, its implementation, or the associated planning method in academic work, **please cite the paper above**.

---

## Overview

Trajectory planning for a tractor with multiple trailers is significantly more difficult than planning for a rigid-body vehicle.

The system is:

- underactuated;
- nonholonomic;
- highly coupled across the tractor and trailers; and
- particularly difficult to maneuver in narrow or cluttered environments.

Instead of decomposing the planning problem into a sequence of geometric maneuvers, this work formulates the task directly as an **optimal control problem** involving the entire articulated vehicle.

The resulting nonlinear programming problem contains:

- tractor and trailer kinematics;
- articulation-angle evolution;
- vehicle geometry;
- control and state bounds;
- initial and terminal conditions; and
- collision-avoidance constraints.

A major numerical difficulty is that such a nonlinear problem can be extremely sensitive to initialization, especially when the free space is narrow.

The key numerical strategy in this repository is therefore an **adaptively homotopic warm-starting approach**.

---

## Core Idea: Adaptive Homotopy

The basic idea is to avoid solving the hardest collision-constrained problem immediately.

Instead, the obstacle geometry is initially reduced toward its geometric center. This produces an easier optimization problem.

A homotopy parameter

```text
gamma ∈ [0, 1]
```

controls the obstacle size.

Conceptually:

```text
gamma = 0
obstacle collapses toward its center
        ↓
easier optimization problem

gamma increases
        ↓
obstacle gradually expands

gamma = 1
        ↓
original obstacle geometry
        ↓
original trajectory-planning problem
```

For a polygon vertex \(V\) and obstacle center \(C\), the implementation effectively constructs an intermediate vertex as

```text
V(gamma) = C + gamma * (V - C)
```

Thus:

```text
gamma = 0     -> all vertices approach the obstacle center
0 < gamma < 1 -> geometrically contracted obstacle
gamma = 1     -> original obstacle
```

The solution obtained for one value of `gamma` becomes the initial guess for the next nonlinear program.

This continuation strategy greatly improves the numerical behavior of the optimization process.

---

## Adaptive Step-Size Strategy

The method does not simply increase `gamma` using a fixed increment.

The implementation begins with:

```matlab
step = 0.2;
alpha = 0.5;
```

and adapts the continuation step according to solver success.

The main logic is:

```text
current successful gamma
        |
        v
try gamma + step
        |
   +----+----+
   |         |
success    failure
   |         |
   v         v
accept      reduce step
gamma       step = step * alpha
   |
   v
continue
```

After a number of consecutive successful continuation cycles, the step size can also be increased.

This is the meaning of the **adaptively homotopic warm-starting approach** implemented in:

```text
AdaptivelyHomotopicWarmStartingApproach.m
```

The procedure continues until either:

```text
gamma = 1
```

and the original problem has been successfully solved, or the continuation step becomes too small and the algorithm terminates with failure.

---

## Getting Started

Clone the repository:

```bash
git clone https://github.com/libai1943/Tractor-trailer-trajectory-planning-case-studies-ICRA-19.git
```

Open the folder in MATLAB and run:

```matlab
RunMe
```

`RunMe.m` is the main entry point of the demonstration.

The standard execution flow is:

```text
Define initial/terminal configurations
              |
              v
Define polygonal obstacles
              |
              v
PrepareTrajectoryPlanning
              |
              v
AdaptivelyHomotopicWarmStartingApproach
              |
              v
AMPL + Ipopt nonlinear programs
              |
              v
Final collision-free trajectory
              |
              v
Visualization
```

---

## Main Entry: `RunMe.m`

`RunMe.m` defines the planning problem and launches the complete algorithm.

The default example specifies:

### Initial configuration

```matlab
x0_1
y0_1
theta0_1
theta0_2
theta0_3
theta0_4
phy_0
v_0
a_0
w_0
```

where the heading variables represent the tractor and articulated trailer bodies.

### Terminal condition

The terminal target is described through the geometric center of a destination region:

```matlab
x_center_tf
y_center_tf
```

together with terminal motion conditions:

```matlab
v_tf
a_tf
w_tf
```

### Obstacles

Polygonal obstacles are specified through:

```matlab
polygon_obstacle_vertex
```

For the current implementation, each obstacle is assumed to be a quadrilateral represented by eight values:

```text
[x1, y1, x2, y2, x3, y3, x4, y4]
```

Multiple obstacles are concatenated into one vector.

Several alternative scenarios are already included as commented examples in `RunMe.m`.

---

## `PrepareTrajectoryPlanning.m`

This script performs basic consistency checks and prepares the boundary conditions for AMPL.

It checks that:

- each obstacle contains four vertices; and
- the boundary-condition vector has the expected dimensions.

It then calls:

```matlab
SetTwoPointBoundaryConditionsToFiles(...)
```

to generate the data files consumed by the AMPL models.

---

## `SetTwoPointBoundaryConditionsToFiles.m`

This function writes the initial and terminal conditions into:

```text
Initial_config
Terminal_config
```

These files are subsequently included by the AMPL optimization models.

The MATLAB layer is therefore responsible for defining the scenario, while AMPL receives the numerical boundary data through intermediate files.

---

## `GenerateGeometricCenters.m`

This function computes the geometric center of each quadrilateral obstacle.

The centers are written into:

```text
Center
```

and provide the fixed points around which obstacle geometry is contracted during the homotopy process.

---

## `GenerateVertexes.m`

This is one of the key functions for understanding the method.

Call:

```matlab
GenerateVertexes(gamma)
```

to construct the obstacle geometry corresponding to the current homotopy parameter.

When:

```text
gamma = 0
```

the obstacle approaches its geometric center.

When:

```text
gamma = 1
```

the original obstacle is exactly recovered.

The function generates several files required by the optimization model:

```text
Current_vertex
Number_obstacle
Area
```

`Current_vertex` contains the current homotopically modified obstacle vertices.

`Area` stores the corresponding polygon areas.

---

## `CalculateArea.m`

Computes the area of a quadrilateral obstacle.

It is used when regenerating obstacle geometry during the homotopy iterations.

---

## `AdaptivelyHomotopicWarmStartingApproach.m`

This script implements the main numerical procedure corresponding to **Algorithm 1** of the ICRA paper.

Important parameters include:

```matlab
step = 0.2;
alpha = 0.5;
Nexpand = 10;
epsilon_exit = 1e-5;
epsilon_0 = 0.05;
```

Their roles are:

| Parameter | Meaning |
|---|---|
| `step` | current increment of the homotopy parameter |
| `alpha` | factor used to reduce the step after failure |
| `Nexpand` | successful-cycle threshold before increasing the step |
| `epsilon_exit` | minimum allowable step before declaring failure |
| `epsilon_0` | initial homotopy level |

The main loop repeatedly:

1. chooses a trial value `gamma_trial`;
2. generates the corresponding obstacle geometry;
3. solves the nonlinear program;
4. checks whether Ipopt reports a successful solution;
5. accepts or rejects the trial value;
6. adapts the continuation step; and
7. uses the latest successful trajectory as the warm start for the next problem.

The process terminates successfully when:

```matlab
gamma_achieved == 1
```

meaning that the trajectory is feasible for the original obstacle geometry.

---

## Progressive NLP Models

The repository contains several AMPL models:

```text
casef.mod
case0.mod
case1.mod
case2.mod
```

and corresponding execution scripts:

```text
rf.run
r0.run
r1.run
r2.run
```

They are used progressively to obtain increasingly complete solutions and useful initial guesses.

The initialization sequence in the current implementation is:

```matlab
!ampl rf.run
!ampl r0.run
!ampl r1.run
```

followed by:

```matlab
!ampl r2.run
```

for the main collision-constrained trajectory optimization.

After each successful solution, the AMPL execution scripts write the optimized variables into:

```text
initial_guess.INIVAL
```

which is then reused by the next optimization problem.

This explicit transfer of the previous solution is an important part of the warm-starting mechanism.

---

## Optimal-Control Model

The AMPL files formulate the tractor-trailer trajectory-planning problem using direct transcription.

The current models use:

```text
NE = 80
```

finite elements.

The vehicle state includes the positions and orientations of the articulated bodies:

```text
x[i,k]
y[i,k]
theta[i,k]
```

as well as tractor motion/control variables such as:

```text
v[i]
a[i]
phy[i]
w[i]
```

where:

- `v` is longitudinal velocity;
- `a` is acceleration;
- `phy` is the tractor steering angle; and
- `w` is steering angular rate.

The model includes the nonlinear kinematics of the tractor and the recursively coupled orientations of the trailers.

---

## Tractor-Trailer Geometry

Each articulated body is represented through four geometric corner points:

```text
A
B
C
D
```

with corresponding Cartesian variables:

```text
AX, AY
BX, BY
CX, CY
DX, DY
```

These points allow the optimization model to explicitly account for the physical footprint of both the tractor and trailers.

The current AMPL files use:

```text
NC = 4
```

articulated bodies in the supplied case studies.

The formulation itself follows the unified multi-trailer modeling idea presented in the paper.

---

## Collision Avoidance

The optimization model incorporates polygonal obstacles directly.

For each tractor/trailer body and each discretization instant, geometric relationships between the vehicle corner points and obstacle polygons are included in the nonlinear program.

During the adaptive homotopy procedure, these collision constraints remain structurally unchanged; what changes is the obstacle geometry supplied through:

```text
Current_vertex
```

This is an important feature of the method:

```text
same trajectory-planning formulation
              +
progressively restored obstacle geometry
```

rather than manually designing a different planning algorithm for every narrow environment.

---

## Solver

The nonlinear programs are solved using **Ipopt** through **AMPL**.

The provided `ipopt.opt` currently contains settings including:

```text
max_iter       5000
max_cpu_time   60
tol            1e-6
mu_strategy    adaptive
linear_solver  ma57
```

The repository also contains the solver-related binaries used by the original Windows demonstration.

---

## AMPL License Notice

The repository is an archived research implementation from 2019 and contains an old AMPL executable that was distributed for demonstration purposes.

The original source code explicitly notes that the included AMPL executable was a **trial version**.

Users should obtain and use their own valid AMPL installation/license in accordance with the current AMPL licensing terms rather than relying on the archived executable for continued use.

The implementation is primarily organized for a **Windows + MATLAB + AMPL** workflow, as can also be seen from commands such as:

```text
del
```

inside the `.run` files.

---

## Visualization

After a successful solution, `RunMe.m` plots:

### Homotopy progress

```matlab
plot(store_gamma)
```

showing the evolution of the successfully achieved homotopy parameter.

### Adaptive step size

```matlab
plot(store_step)
```

showing how the continuation step changes during the numerical solution process.

### Final tractor-trailer trajectory

```matlab
DrawTrajectories
```

plots the optimized articulated-vehicle trajectory.

A dynamic visualization can also be generated using:

```matlab
VideoGeneration
```

which is commented out by default in `RunMe.m`.

---

## Repository Structure

```text
Tractor-trailer-trajectory-planning-case-studies-ICRA-19/
│
├── RunMe.m
│   Main demonstration entry
│
├── PrepareTrajectoryPlanning.m
│   Checks and prepares the planning problem
│
├── AdaptivelyHomotopicWarmStartingApproach.m
│   Adaptive homotopy / warm-starting algorithm
│
├── GenerateGeometricCenters.m
│   Computes obstacle centers
│
├── GenerateVertexes.m
│   Generates homotopically contracted obstacle polygons
│
├── CalculateArea.m
│   Computes polygon areas
│
├── SetTwoPointBoundaryConditionsToFiles.m
│   Writes initial/terminal conditions for AMPL
│
├── casef.mod
├── case0.mod
├── case1.mod
├── case2.mod
│   AMPL optimal-control models
│
├── rf.run
├── r0.run
├── r1.run
├── r2.run
│   AMPL execution and warm-start scripts
│
├── ipopt.opt
│   Ipopt solver configuration
│
├── DrawTrajectories.p
│   Trajectory visualization
│
├── VideoGeneration.p
│   Dynamic visualization
│
├── RefineGrids.p
├── SmootmingProfile.p
│   Supporting routines
│
└── ICRA paper(scanned).pdf
    Paper associated with this repository
```

---

## Suggested Experiments

A convenient way to explore the code is to begin with the default scenario in `RunMe.m` and then modify one component at a time.

For example, you can change:

```matlab
polygon_obstacle_vertex
```

to construct a different narrow environment.

You can also experiment with:

```matlab
step
alpha
Nexpand
epsilon_0
```

inside:

```text
AdaptivelyHomotopicWarmStartingApproach.m
```

to observe how the continuation strategy affects convergence.

For a more substantial modification, the vehicle geometry and number of articulated bodies can be studied through the corresponding parameters in the AMPL model files.

---

## Citation

**If you use this repository, its source code, numerical implementation, or the proposed planning approach in a publication, please cite the following paper:**

> B. Li, Y. Zhang, T. Acarman, Q. Kong, and Y. Zhang,  
> “Trajectory Planning for a Tractor with Multiple Trailers in Extremely Narrow Environments: A Unified Approach,”  
> in *2019 International Conference on Robotics and Automation (ICRA)*, Montreal, Canada, May 2019, pp. 8557–8562.  
> DOI: [10.1109/ICRA.2019.8793955](https://doi.org/10.1109/ICRA.2019.8793955)

### BibTeX

```bibtex
@inproceedings{li2019tractor,
  title={Trajectory planning for a tractor with multiple trailers in extremely narrow environments: A unified approach},
  author={Li, Bai and Zhang, Youmin and Acarman, Tankut and Kong, Qi and Zhang, Yue},
  booktitle={2019 International Conference on Robotics and Automation (ICRA)},
  pages={8557--8562},
  year={2019},
  organization={IEEE},
  doi={10.1109/ICRA.2019.8793955}
}
```

---

Copyright © 2019 Bai Li.
