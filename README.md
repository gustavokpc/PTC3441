# Planar RRR Manipulator Modelling, Trajectory Generation, and Control

MATLAB and Simulink coursework for a planar three-revolute-joint (RRR) manipulator. The repository brings together homogeneous-transform utilities, forward and analytical inverse kinematics, Jacobian and velocity transformations, cubic trajectory generation, and controller simulations based on PD feedforward and resolved-rate control.

This is a collection of lab exercises rather than a single production application. Reusable source, examples, simulations, and published material are separated by purpose. Run `setup_project.m` once per MATLAB session to add these directories to the MATLAB path; it preserves the original function and Simulink model names.

## What is implemented

| Area | Relevant files | Implemented behaviour |
| --- | --- | --- |
| Rigid transforms | `src/transforms/` | Converts planar user poses `[x y theta]` to homogeneous transforms, multiplies transforms, and computes transform inverses. |
| Kinematics | `src/kinematics/` | Forward kinematics for a planar RRR geometry; analytical inverse-kinematics branches; station/tool-frame composition; nearest-solution selection. |
| Differential kinematics | `src/jacobians/jacobian.m`, `src/kinematics/veltrans.m` | Spatial-form Jacobian construction and velocity/twist frame transformation. |
| Trajectories | `src/trajectories/` | Piecewise cubic joint trajectories with endpoint and via-point velocity rules, plus Cartesian-waypoint conversion through inverse kinematics. |
| Kinematic exercises | `examples/kinematics/`, `examples/trajectories/` | Frame-transform examples, pose evaluation, multi-target IK calls, and trajectory plots. |
| Control simulations | `simulations/ex4_pd_feedforward/`, `simulations/ex5_resolved_rate/` | 100 Hz parameter setup, trajectory signals, Simulink execution, and plots for PD/feedforward and resolved-rate-control exercises. |
| Dynamics exercise | `simulations/list2/` | A symbolic torque routine and a separate resolved-rate Simulink model/post-processing script. See the completion notes below. |

## Workflow

```text
User pose [x, y, θ]
        │
        ├──> homogeneous transforms ──> forward/inverse kinematics ──> workspace pose
        │                                      │
        │                                      └──> cubic joint trajectory
        │                                                │
        └──────────────────────────────────────────────> Simulink references / controller simulation
                                                         │
                                             PD + feedforward / resolved-rate plots
```

## Requirements

- MATLAB with Simulink. The checked-in models identify MATLAB R2025a in their metadata; use that release or validate model compatibility in the installed release.
- Symbolic Math Toolbox for the symbolic torque function in `Ex5_Lista2_simul.m`.
- Standard MATLAB plotting support. No external hardware, ROS interface, or third-party MATLAB package is referenced by the repository.

## Getting started

Open MATLAB in this repository and configure its paths:

```matlab
cd("PTC3441")
run("setup_project.m")
```

Start with the non-Simulink helpers and exercises:

```matlab
T = utoi([0.1 0.2 30]);
pose = itou(T);
tool_pose = where_robot([0 90 -90], utoi([0.1 0.2 30]), utoi([-0.1 0.3 0]), [0.5 0.3]);
Ex3_LS3
```

`Ex3_LS3` produces joint and Cartesian trajectory figures. For the control models, first run one of the matching parameter scripts from the root:

```matlab
Ex4_LS3_code_letra_a  % or Ex4_LS3_code_letra_b; invokes Ex4_LS3.slx
Ex5_LS3_code          % invokes Ex5_LS3.slx
```

Those scripts populate the base workspace, invoke `sim`, and plot named logged signals. Review the parameters and model before simulation; no claim is made that their gains, trajectories, or output signals are suitable for a physical manipulator.

## Results and existing documentation

[`docs/published/`](docs/published/) contains MATLAB-published documentation and figure exports from the original exercises. In particular, [`Ex3_LS3.html`](docs/published/Ex3_LS3.html) contains the saved joint position, velocity, and acceleration plots, and the `Ex4_LS3_code_*` pages contain published control-exercise output. These are preserved historical results, not newly reproduced simulations.

## Completion status and limitations

### Completed, callable building blocks

The transform utilities, forward/inverse-kinematics functions, Jacobian/velocity functions, and cubic trajectory functions are present as MATLAB source. The principal control scripts reference matching `.slx` files that are also included.

### Retained incomplete or context-dependent work

- `examples/kinematics/ex4.m` expects `arelu`, `arelb`, and `urelc` in the workspace; its example assignments are commented out.
- `simulations/list2/Exc4_Lista2_code.m` only plots logged workspace signals (`Jac_det`, `ThetadotA`, `ThetaA`, `XA`, `Tau`); it does not invoke `Ex4_Lista2_simul.slx` itself.
- `simulations/list2/Ex5_Lista2_simul.m` declares `tau_symbolic` as its primary function. Its file name does not match the function name, which is not the conventional MATLAB file/function arrangement and should be resolved before relying on it as a reusable API.

### Unit and mathematical review required

No mathematical behaviour was changed. Before extending or using this work, validate the following against the assignment convention:

- `utoi`, `itou`, `kin`, and `jacobian` use degree-based trigonometric functions (`sind`, `cosd`, `acosd`), while `invkin` uses radian-based functions (`atan2`) and `trajectorygenerationrrr` defines joint limits in radians. The current documentation and callers also mix degree and radian labels. This is a material convention inconsistency.
- `Ex3_LS3` converts `thpath` with `rad2deg` for plotting, then passes the same values to `kin`, whose implementation uses degree-based trigonometry. Validate the intended joint-angle unit end to end before interpreting those plots.
- `itou` obtains orientation with `acosd(R(1,1))`, which cannot distinguish the sign of an angle. Do not use it for unambiguous orientation recovery without validation.
- `jacobian.m` returns a 6-by-3 matrix although its header describes a 3-by-3 Jacobian. Confirm the intended twist convention and frame before using it in a controller.
- The models use simplified plant/controller parameters and do not represent a calibrated physical robot, joint limits, collision constraints, saturation validation, or safety supervision.

## Repository hygiene

The original published HTML is retained under `docs/published/`, with its associated images moved alongside it so existing relative image links continue to work. Generated MATLAB autosaves (`*.asv`), compiled Simulink caches (`*.slxc`), and `slprj/` cache contents were removed from Git tracking; the original `.slx` models and all MATLAB source remain. `.gitignore` prevents these generated artifacts from being re-added.

## Academic context and credits

The source headers identify the work as 2025 **Grupo 3** coursework. Git history credits Gustavo Kuratomi and Bill-DT; individual ownership of each exercise is not recorded in enough detail to assign more specific contributions. Existing source-level credits, including the earlier `utoi.m` history, have been preserved.
