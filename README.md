# Custom AMR Platform

An end-to-end Autonomous Mobile Robot (AMR) navigation stack that fuses **AMCL localization** with **visual odometry** to run a stable, high-rate control loop — achieving a **92% navigation success rate** at **15 Hz**.

**Status:** Ongoing

## Overview

Most tutorial-level ROS2 navigation setups rely on AMCL alone for localization. AMCL is accurate, but it has a structural limitation: it only publishes a corrected pose when it has enough particle-filter confidence, which is comparatively low-rate (often 1–5 Hz) and can lag or momentarily lose confidence during fast motion, in geometrically ambiguous spaces (long featureless corridors), or right after a large sensor outlier.

This project's core idea: **don't rely on one localization source — fuse two complementary ones.**

- **AMCL** gives you *globally correct but low-rate* pose corrections (it knows where you are on the map).
- **Visual odometry (VO)** gives you *high-rate but drifting* relative motion estimates (it knows how far/fast you just moved, very frequently, but has no idea where that is on the global map).

Fusing them means the robot always has a fresh, high-rate pose estimate (from VO) that gets periodically corrected against the map (by AMCL) before it drifts too far — which is exactly what makes a smooth 15 Hz control loop possible instead of a jerky one that's effectively rate-limited by AMCL's slower publish rate.

## Approach

1. **Visual Odometry** — estimate frame-to-frame camera motion (feature tracking + PnP or optical flow, depending on the camera setup) to produce a continuous `nav_msgs/Odometry` stream at high rate.
2. **Sensor Fusion (EKF)** — an Extended Kalman Filter fuses wheel odometry, VO, and (when available) AMCL pose corrections into one continuous `odom -> base_link` estimate. AMCL corrections effectively "anchor" the filter to the global map periodically; VO fills in the high-rate motion between anchors.
3. **ros2_control** — the fused state and computed velocity commands are sent through a `ros2_control`-based differential-drive hardware interface, keeping the control loop decoupled from whatever simulator/hardware is underneath.
4. **Nav2** — global/local planning sits on top, consuming the fused, high-rate localization output rather than raw AMCL.

## Why this matters (the trade-off it's solving)

| | Rate | Global accuracy | Drift over time |
|---|---|---|---|
| AMCL alone | Low (~1–5 Hz) | High | None (self-corrects) |
| Visual Odometry alone | High (~30+ Hz) | None (relative only) | Accumulates |
| **Fused (this project)** | **High (15 Hz)** | **High** | **Bounded (corrected by AMCL)** |

## Tech Stack

`ROS2` · `Nav2` · `AMCL` · `Visual Odometry` · `robot_localization (EKF)` · `ros2_control` · `Gazebo`

## Package Structure

```
custom-amr-platform/
├── src/custom_amr_platform/
│   ├── __init__.py
│   ├── visual_odometry_node.py     # frame-to-frame VO -> nav_msgs/Odometry
│   └── control_loop_node.py        # 15Hz cmd_vel loop consuming fused odom
├── launch/
│   ├── bringup_launch.py           # robot + sensors + ros2_control
│   ├── localization_fusion_launch.py  # AMCL + VO + robot_localization EKF
│   └── navigation_launch.py        # Nav2 stack on top of fused odometry
├── config/
│   ├── ekf_params.yaml             # robot_localization fusion tuning
│   ├── ros2_control_params.yaml    # diff-drive controller config
│   └── nav2_params.yaml
├── scripts/
│   └── eval_navigation_success.py  # computes success-rate metric across N trial runs
├── package.xml
├── setup.py
└── README.md
```

## Running it (simulation)

```bash
ros2 launch custom_amr_platform bringup_launch.py
ros2 launch custom_amr_platform localization_fusion_launch.py
ros2 launch custom_amr_platform navigation_launch.py
ros2 run custom_amr_platform control_loop_node   # 15 Hz control loop
```

## Results

- **92% navigation success rate** across repeated trial runs to randomized goal poses (measured via `scripts/eval_navigation_success.py`: success = goal reached within tolerance without a Nav2 recovery-behavior failure).
- **15 Hz** stable control loop rate, verified against the fused localization output rather than raw AMCL (which alone would rate-limit the loop).

## What I'd improve next

- Replace the loosely-coupled EKF fusion with a tightly-coupled filter that fuses raw VO feature residuals directly, instead of pre-computed VO odometry — reduces error introduced by VO's own internal frame-to-frame estimation step.
- Add a covariance-aware weighting so AMCL corrections are trusted more heavily in feature-rich areas and less in narrow, ambiguous corridors.
