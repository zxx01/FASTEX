# `data_log_manager`

`data_log_manager` 是 FASTEX 的实验数据记录与在线观测辅助包。

它包含两类不同角色的内容：

- 运行时核心
  - `data_log_manager_node`
  - 负责接收探索阶段的统计信息、写入实验日志，并发布用于可视化的话题
- 可选观测工具
  - `scripts/plotting/realTimePlot.py`
  - `scripts/plotting/OdomVisualizer.py`
  - 负责实时绘图与速度观测，不参与探索主逻辑

## Launch 入口

### `launch/logging.launch`

这是实验记录主入口，通常由 `exploration_manager` 的场景 launch 间接包含。

它负责：

- 启动 `data_log_manager_node`
- 接收里程、地图统计和各类算法耗时日志
- 将聚合指标发布到可视化话题
- 按需写入 `exploration_data_files/`
- 可选 include `launch/vis.launch`

关键参数：

- `odometry_topic`
- `scene_name`
- `is_logging`
- `is_launch_data_vis`
- `exploration_data_topic`
- `total_run_time_topic`
- `preprocess_run_time_topic`
- `motion_run_time_topic`

### `launch/vis.launch`

这是可选观测工具入口，不负责日志汇聚本身，只负责启动绘图脚本。

适用场景：

- 本地调试探索过程
- 观察累计探索体积、移动距离和关键阶段耗时
- 单独分析里程速度趋势

关键参数：

- `odometry_topic`
- `exploration_data_topic`
- `total_run_time_topic`
- `preprocess_run_time_topic`
- `motion_run_time_topic`

