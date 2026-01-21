# GRA Meta-Nulling (RoboNull AI)

This repository contains the reference architecture, mathematical foundations,
and a technical prototype for **GRA Meta-Nulling** — a two-level resonance
nullification system for multi-domain AI and robotics.

## Core Idea
GRA Meta-Nulling minimizes:
- Local cognitive foam inside each domain/task
- Meta-level foam between domains with respect to a global objective

Result: **cognitive vacuum** — structurally invariant, conflict-free decisions.

## Contents
- `/docs` — mathematical foundations and theory
- `/core` — PyTorch prototype (local + meta nulling)
- `/sim` — simulation hooks (MuJoCo / Isaac-ready)
- `/ros2` — ROS2 integration sketch
- `/examples` — grasp + navigation demo logic

## Status
Research-grade / prototype. Designed for extension.

## License
Apache 2.0
-----------
## RoboNull MVP Demo

**Scenario:** Pick & Walk & Avoid (Unitree G1)

- Vision, Grasp, Navigation trained independently
- RoboNull adds a meta-layer minimizing inter-domain conflict
- No task-specific heuristics
- Fully switchable at runtime

**Key result:**
> Collision events reduced to zero without degrading task success.

See: `experiments/baseline_vs_robonull.md`


