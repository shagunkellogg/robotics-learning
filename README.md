# Robotics Learning Journey

This repository documents my hands-on learning journey in robotics,
starting from simulation fundamentals and building toward more advanced
robotics systems.

## Current Focus
- Robot simulation using MuJoCo
- Joint-level control and motion
- Understanding kinematics through hands-on experiments

## Progress
### Day 1: One-joint arm simulation (MuJoCo)
- Set up MuJoCo on macOS
- Built and simulated a one-joint robotic arm
- Implemented position control to swing the arm in simulation

```bash
mjpython scripts/day1_mujoco_move_joint.py
```

![Day 1 MuJoCo joint motion](media/day1_mujoco_joint_motion.gif)

### Day 2: Moving tip to target
- Define a target point in space (x, y, z)
- Compute joint motion automatically (using Jacobian physics + end controller)
- Visually confirm the tip tries to reach the target

![Day 2](media/day2_mujoco_tip_to_target.gif)

## Why this repo exists

I am building practical robotics intuition with a long-term goal of working in robotics product roles (e.g., surgical robotics, manipulation systems). This repo emphasizes learning-by-doing and clear documentation over polished demos.
