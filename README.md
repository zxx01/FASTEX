


<div align = "center">
  <h1>
    FASTEX: Fast UAV Exploration in Large-Scale Environments Using Dynamically Expanding Grids and Coverage Paths
  </h1>
</div>
<!-- Zhang, Xiaoxun and Duan, Peiming and Zheng, Lanxiang and
               Huang, Junlong and Cheng, Hui -->
<div align="center">
  <strong>
        Xiaoxun Zhang,
        Peiming Duan,
        Lanxiang Zheng,
        Junlong, Huang, and
        Hui, Cheng<sup>†</sup>
  </strong>
  <p>
    <sup>†</sup>Corresponding Author
  </p>
  <p align="center">
    <a href="https://ieeexplore.ieee.org/abstract/document/11246816/"><img src="https://img.shields.io/badge/Paper-IEEE%20Xplore-blue" alt="Paper"></a>
    <img src="https://img.shields.io/badge/License-GPL%203.0-green" alt="License">
    <img src="https://img.shields.io/badge/Conference-IROS%202025-orange" alt="IROS 2025">
    <img src="https://img.shields.io/badge/ROS-Noetic-blueviolet" alt="ROS Noetic">
    <img src="https://img.shields.io/badge/Ubuntu-20.04-lightgrey" alt="Ubuntu 20.04">
  </p>
</div>

---

## 📖 Overview

**FASTEX** is an efficient and robust autonomous exploration framework for quadrotor UAVs operating in large-scale, complex environments such as caves, dense forests, and cluttered indoor spaces. The system leverages **dynamically expanding grids** for scalable environment representation and **incremental coverage path planning** to achieve rapid, complete exploration.

---

## 📦 Prerequisites

### System Requirements

- **OS**: Ubuntu 20.04 (recommended)
- **ROS**: [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- **Docker** (optional but recommended): 20.10+

### Local Dependencies (Non-Docker)

```bash
# System libraries
sudo apt update && sudo apt install -y \
  git tmux cmake wget build-essential \
  pcl-tools libpcl-dev libdw-dev libtbb-dev libelf-dev \
  libglew-dev libglfw3-dev libblosc-dev libeigen3-dev \
  libspdlog-dev libopenexr-dev libarmadillo-dev liblog4cplus-dev \
  libatlas-base-dev libsuitesparse-dev libgoogle-glog-dev \
  libignition-common3-graphics-dev libignition-common3-profiler-dev \
  python3-tk python3-pip python3-pandas python3-wstool python3-matplotlib \
  python3-catkin-tools

# ROS packages
sudo apt install -y \
  ros-noetic-rosfmt ros-noetic-rviz ros-noetic-navigation \
  ros-noetic-lms1xx ros-noetic-message-filters ros-noetic-tf

# NLopt v2.7.1 (required for trajectory optimization)
git clone https://github.com/stevengj/nlopt.git --branch v2.7.1
cd nlopt && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

---

## 🔨 Build

### Option 1: Docker Build (Recommended)

All dependencies are handled inside the container. No local installation required.

```bash
cd docker
make build
```

See [`docker/README.md`](docker/README.md) for detailed Docker usage. See [`docker/Makefile`](docker/Makefile) and [`docker/cave_exp.Dockerfile`](docker/cave_exp.Dockerfile) for build customization.

### Option 2: Native Build

```bash
# Create and configure catkin workspace
mkdir -p ~/fast_ex_ws/src
cd ~/fast_ex_ws/src
git clone https://github.com/zxx01/FASTEX.git

# Build
cd ~/fast_ex_ws
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

---

## 🌍 Environment PCD Files

The environment point-cloud files used by the released scenes can be downloaded here:

- [Google Drive: FASTEX environment PCD files](https://drive.google.com/drive/folders/1pDnEowW5wwxa1IzXnkwg7daS7pv43kwY?usp=sharing)

Please place the downloaded scene files under [`marsim/map_generator/resource/`](marsim/map_generator/resource/) before running the `cave`, `forest`, or `indoor` experiments.

Expected filenames for the maintained scenes are:

- `cave`: `cave.pcd`
- `forest`: `forest_ds.pcd`
- `indoor`: `indoor.pcd`

---

## 🚀 Run

### Quick Start

Launch exploration directly via `roslaunch`:

```bash
# Cave exploration
source devel/setup.bash && roslaunch exploration_manager cave_exploration.launch

# Forest exploration
source devel/setup.bash && roslaunch exploration_manager forest_exploration.launch

# Indoor exploration
source devel/setup.bash && roslaunch exploration_manager indoor_exploration.launch
```

### Docker Compose (Multi-Scene)

First ensure the Docker image is built (see [Build](#-build) above), then:

```bash
# Single scene
bash docker/run_lca_scenes.sh start forest

# Multiple scenes
bash docker/run_lca_scenes.sh start cave forest indoor
```

See [`docker/run_lca_scenes.sh`](docker/run_lca_scenes.sh) and [`docker/lca_scenes.compose.yml`](docker/lca_scenes.compose.yml) for scene orchestration details.

### Launch File Arguments

Key ROS parameters available in launch files ([`fastex_planner/applications/exploration_manager/launch/`](fastex_planner/applications/exploration_manager/launch/)):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `is_launch_rviz` | `true` | Launch RViz for visualization |
| `is_logging` | `false` | Enable exploration data logging |
| `is_launch_data_vis` | `true` | Launch data visualization tools |
| `auto_start` | `true` | Automatically start exploration |
| `max_vel` | `2.0` | Maximum flight velocity (m/s) |
| `max_acc` | `1.5` | Maximum acceleration (m/s²) |
| `sdf_resolution` | `0.4` | ESDF map resolution (m) |

---

## ⚙️ Configuration

### Exploration Parameters

Located in [`fastex_planner/applications/exploration_manager/launch/`](fastex_planner/applications/exploration_manager/launch/) — adjust per-scenario launch files to tune exploration behavior:

- [`launch/cave/cave_exploration.launch`](fastex_planner/applications/exploration_manager/launch/cave/cave_exploration.launch)
- [`launch/forest/forest_exploration.launch`](fastex_planner/applications/exploration_manager/launch/forest/forest_exploration.launch)
- [`launch/indoor/indoor_exploration.launch`](fastex_planner/applications/exploration_manager/launch/indoor/indoor_exploration.launch)

### Map Preprocessing Parameters

Located in [`fastex_planner/planning/map_process/resources/`](fastex_planner/planning/map_process/resources/):

| File | Scene |
|------|-------|
| [`config_cave.yaml`](fastex_planner/planning/map_process/resources/config_cave.yaml) | Cave exploration |
| [`config_forest.yaml`](fastex_planner/planning/map_process/resources/config_forest.yaml) | Forest exploration |
| [`config_indoor.yaml`](fastex_planner/planning/map_process/resources/config_indoor.yaml) | Indoor exploration |

Key tuning sections include:

- **DynamicExpandingGrid** — Grid resolution, expansion bounds, consistency cost
- **FrontierManager** — Visible voxel thresholds, cluster radius, viewpoint sampling
- **WSRoadmap** — Roadmap sampling density, connectivity range, clearance constraints

### Exploration Data Logging

 Run-time metrics are stored under [`fastex_planner/applications/data_log_manager/exploration_data_files/`](fastex_planner/applications/data_log_manager/exploration_data_files/), including per-iteration preprocessing time and planning time for performance analysis.

---

## 📚 Citation

If you use FASTEX in your research, please cite our paper:

```bibtex
@inproceedings{zhang2025fastex,
  title     = {{FASTEX}: Fast {UAV} Exploration in Large-Scale Environments
               Using Dynamically Expanding Grids and Coverage Paths},
  author    = {Zhang, Xiaoxun and Duan, Peiming and Zheng, Lanxiang and
               Huang, Junlong and Cheng, Hui},
  booktitle = {2025 IEEE/RSJ International Conference on Intelligent
               Robots and Systems (IROS)},
  pages     = {3541--3548},
  year      = {2025},
  organization = {IEEE}
}
```

---

## 📝 Notes

1. Exploration and planning parameters are concentrated in [`fastex_planner/applications/exploration_manager/launch/`](fastex_planner/applications/exploration_manager/launch/). Map preprocessing parameters reside in [`fastex_planner/planning/map_process/resources/`](fastex_planner/planning/map_process/resources/). Adjust them according to your specific scenario.

2. For detailed parameter descriptions, refer to [`fastex_planner/planning/map_process/resources/config_cave.yaml`](fastex_planner/planning/map_process/resources/config_cave.yaml) for annotated comments.

3. The [`fastex_planner/applications/data_log_manager/exploration_data_files/`](fastex_planner/applications/data_log_manager/exploration_data_files/) directory stores preprocessed timing data and per-iteration planning time logs for performance evaluation.

4. If you encounter Docker permission issues, follow the setup steps in [`docker/README.md`](docker/README.md) and [`docker/Makefile`](docker/Makefile) to configure user groups.

---

## 🙏 Acknowledgments

We would like to express our gratitude to the following projects, which have provided significant support and inspiration for our work:

- [**FUEL**](https://github.com/HKUST-Aerial-Robotics/FUEL) — An efficient framework for fast UAV exploration, leveraging Frontier Information Structure (FIS) and hierarchical planning.
- [**MARSIM**](https://github.com/hku-mars/MARSIM) — A light-weight point-realistic simulator for LiDAR-based UAVs, supporting diverse realistic maps and sensor models.
- [**FALCON**](https://github.com/HKUST-Aerial-Robotics/FALCON) — Fast autonomous aerial exploration using coverage path guidance.

---

## 📧 Contact

For questions and collaborations, please contact the corresponding author or open an issue on GitHub.
