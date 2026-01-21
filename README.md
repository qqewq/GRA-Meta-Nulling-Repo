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
# RoboNull ROS2 Demo

This directory contains a **live ROS2 simulation demo** for the RoboNull MVP — a meta‑coordination layer that eliminates task conflict between navigation and grasping using a meta‑nullification signal (Φ_meta).

The demo integrates:
- ROS2 nodes for **vision**, **navigation**, **grasping**
- A **meta_null_node** for inter‑domain coordination
- A 2D simulator (`sim_env.py`) that models collisions and goal achievement
- A live visualization of **Φ_meta** and **collisions**

---

## 🚀 Overview

RoboNull is designed to:

✔ eliminate conflicts between domain policies (e.g., navigation vs grasping)  
✔ operate as a coordinating middleware, **without retraining policies**  
✔ be switchable at runtime (α coefficient)  
✔ demonstrate clear quantitative benefits (fewer collisions, smoother behavior)

This demo visualizes:
- **Φ_meta dynamics**
- **Collision events**
- Final robot trajectory in a simple 2D environment

---

## 📁 Folder Structure


- **sim_env.py** — simple 2D simulator
- **run_full_demo.py** — runs ROS2 simulation with live graphs
- **plots_live.py** — reusable visualizer for Φ_meta and collisions

---

## 🔧 Requirements

Install:


Ensure you have a ROS2 workspace and that your ROS2 environment is sourced:


---

## 📡 How to Run

### 1️⃣ Launch ROS2 Nodes

From the root of the repository:

```bash
ros2 launch ros2/launch_all.py
python3 ros2_sim/run_full_demo.py

---

# 🔧 Что сделать после вставки

1. Сохранить этот текст в корневой `README.md` (или обновить текущий)  
2. Commits:
   - `docs: Add ROS2 demo README with run instructions and topics`
3. Убедиться, что `ros2_sim/run_full_demo.py` работает по инструкции

---

Если нужна **Markdown‑версия для GitHub с изображениями** (плейсхолдеры графиков / GIF / пример вывода), могу сгенерировать её сейчас — скажи **“с картинками”**.



