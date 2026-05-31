# Docker Usage

This directory contains the supported container workflow for FASTEX scene reproduction.

## What Docker Covers

- Builds a runnable ROS Noetic image for FASTEX
- Starts maintained scenes through Docker Compose
- Supports headless execution, RViz, and runtime logging toggles

Docker support is focused on reproducing the published `cave`, `forest`, and `indoor` scene launches. It is not intended as a generic development environment for modifying every dependency in-place.

If your host still requires `sudo docker`, you can override the command at invocation time, for example `make DOCKER="sudo docker" build`.

## Build the Image

```bash
cd docker
make build
```

## Start Scenes

```bash
bash run_lca_scenes.sh start cave
bash run_lca_scenes.sh start forest
bash run_lca_scenes.sh start indoor
```

Start multiple scenes:

```bash
bash run_lca_scenes.sh start cave forest indoor
```

## Common Runtime Modes

### Headless reproduction

```bash
bash run_lca_scenes.sh start cave
```

### RViz visualization

```bash
IS_LAUNCH_RVIZ=true bash run_lca_scenes.sh start cave
```

### RViz plus file logging

```bash
IS_LAUNCH_RVIZ=true IS_LOGGING=true bash run_lca_scenes.sh start forest
```

### File logging without live plots

```bash
IS_LOGGING=true IS_LAUNCH_DATA_VIS=false bash run_lca_scenes.sh start indoor
```

## Environment Variables

| Variable | Default | Meaning |
| --- | --- | --- |
| `IS_LAUNCH_RVIZ` | `false` | Launch RViz inside the container |
| `IS_LOGGING` | `false` | Enable log file writing in `data_log_manager` |
| `IS_LAUNCH_DATA_VIS` | `false` | Start plotting and odometry visualization helpers |

## Inspect and Stop

```bash
bash run_lca_scenes.sh ps
bash run_lca_scenes.sh logs cave
bash run_lca_scenes.sh stop cave
bash run_lca_scenes.sh down
```

## X11 Notes

If RViz cannot access the host display:

```bash
xhost +local:docker
```

The compose setup mounts `/tmp/.X11-unix` and `/tmp/.docker.xauth` for GUI support.

## Native vs Docker

- Use native ROS when you are actively editing source code and already have a Noetic workspace.
- Use Docker when you want a cleaner reproduction path with fewer host-side dependency changes.

## Cleanup

```bash
make clean
```
