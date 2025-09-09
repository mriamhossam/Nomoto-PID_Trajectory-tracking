# Nomoto-PID_Trajectory-tracking
Based on the first-order Nomoto model and an improved PID controller, the project realizes real-time tracking control and visualization of a given trajectory (CSV historical point column), and exports key timing data (position, heading angle, rudder angle) to Excel for subsequent analysis.
# 基于 Nomoto 模型的船舶路径跟踪与航向控制

![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)

该项目基于一阶 Nomoto 模型与改进型 PID 控制器，实现对给定轨迹（CSV 历史点列）的实时跟踪控制与可视化，并将关键时序数据（位置、航向角、舵角）导出为 Excel 以便后续分析。

- 动力学层：使用一阶 Nomoto 模型描述船舶艏向运动（航向角、角速度）与位姿更新；
- 控制层：基于前视点与横向偏差计算期望航向，采用带积分约束与转弯自适应的 PID 生成舵角；
- 可视化层：实时绘制目标轨迹与实际跟踪轨迹，以及“航向角/舵角-时间”曲线；
- 数据层：将时序数据输出到 `history4s_control.xlsx`。

### 演示 (Demo)

直接运行脚本将：
- 读取 `history4s.csv` 的轨迹点（支持抽样）；
- 实时显示跟踪过程（两幅子图：轨迹与角度）；
- 结束后导出 Excel 文件 `history4s_control.xlsx`。

> 运行过程中会弹出绘图窗口，展示船舶当前位置、历史轨迹、目标路径以及航向/舵角随时间变化曲线。

### 主要特性 (Features)

- **Nomoto 一阶模型**：简洁描述船舶艏向动态，参数少、可解释性强。
- **自适应 PID 控制**：
  - 依据是否处于转向状态调整 `Kp/Ki/Kd` 与舵角限幅；
  - 积分抗饱和、期望航向变化率限制、前视点/预瞄策略。
- **目标点选择**：
  - 仅选择视场前方且方向变化更平滑的目标点，有助于稳定跟踪。
- **实时可视化**：
  - 轨迹子图：目标路径、实际路径、当前船位；
  - 角度子图：航向角(°)、舵角(°)随时间变化。
- **数据导出**：
  - 将时间、位置、航向角、舵角写入 Excel，便于复盘与分析。

### 文件结构 (Files)

- `Nomoto.py`：主程序（模型、控制器、仿真主循环与可视化）。
- `history4s.csv`/`history5s.csv`：示例轨迹点数据（x, y）。
- `history4s_control.xlsx`：运行后自动生成的控制与状态结果。

### 如何使用 (Usage)

1) 安装依赖：

```bash
pip install numpy pandas matplotlib
```

2) 运行主程序：

```bash
python Nomoto.py
```

运行后会弹出可视化窗口，并在结束时于当前目录生成 `history4s_control.xlsx`。

### 关键组件 (Key Components)

- **`class NomotoModel`**
  - 成员：`x, y, psi, r, u, T, K`
  - `update(delta, dt)`：根据一阶 Nomoto 模型更新角速度与航向角，并推进位姿。

- **`class Controller`**
  - 参数：`Kp, Ki, Kd` 与多项约束（积分上限、最大艏向变化率、最大舵角等）
  - `is_point_forward(ship_x, ship_y, ship_psi, point_x, point_y)`：判定候选点是否在前方视场。
  - `find_target(current_idx, target_x, target_y, ship_x, ship_y, ship_psi)`：在前方候选点内，择方向变化更平滑者为目标点。
  - `calc_control(target_x, target_y, next_x, next_y, ship_x, ship_y, ship_psi, dt)`：
    - 计算路径切向与横向误差；
    - 基于转向状态选择预瞄/前视策略并给出期望航向；
    - 应用自适应 PID 与限幅得到舵角 `delta`。
  - `normalize_angle(angle)`：角度归一化到 \(-\pi, \pi\]。

- **`main()` 主流程**
  - 读取 CSV，抽样至 `target_x/target_y`；
  - 初始化模型与控制器、设置起点与初始航向；
  - 回路：目标点选择 → 计算舵角 → 更新模型 → 记录与绘图 → 目标点推进；
  - 结束：输出 Excel 并关闭可视化。

### 配置与自定义 (Configuration)

在 `Nomoto.py` 中可根据需要修改：

- **轨迹与抽样**：
  - `sampling_interval = 2`：从 CSV 抽样读取，步长越大点越稀疏。
- **模型参数**（`NomotoModel.__init__`）：
  - `u`（船速，m/s）、`T`（时间常数）、`K`（增益）。
- **控制参数**（`Controller.__init__`）：
  - `Kp, Ki, Kd`；`max_integral`（积分上限）、`max_yaw_rate`（期望航向变化率上限）、
    `max_heading_change`（备用限幅）、`look_ahead_min/max`（前视距离）、`forward_angle_threshold`（前方判定阈值）。
- **仿真步长**：
  - `dt = 0.1`（秒）。
- **起始条件**：
  - 起点取目标轨迹首点；初始航向 `ship.psi = math.radians(90)` 可按需要调整。

示例（片段）：

```python
# 读取轨迹数据并抽样
sampling_interval = 2
target_x = df['x'].values[::sampling_interval]
target_y = df['y'].values[::sampling_interval]

# 初始化模型与控制器
ship = NomotoModel()
controller = Controller()
ship.x, ship.y = target_x[0], target_y[0]
ship.psi = math.radians(90)

# 仿真步长
dt = 0.1
```

### 输出 (Outputs)

- `history4s_control.xlsx`：包含以下列：
  - `Time(s)`, `X(m)`, `Y(m)`, `Heading(deg)`, `Rudder(deg)`。
- 实时可视化窗口：
  - 子图1：目标路径与实际跟踪路径、船位；
  - 子图2：航向角与舵角的时间历史。

### 依赖 (Dependencies)

- `numpy`, `pandas`, `matplotlib`

安装示例：

```bash
pip install numpy pandas matplotlib
```

### 注意事项 (Notes)

- `history4s.csv`/`history5s.csv` 应包含 `x, y` 列；若文件较大可适当增大 `sampling_interval`。
- 控制器参数对性能影响较大，直线段与转弯段采用不同限幅与权重，请按场景调参。
- 若跟踪发散或震荡：
  - 适当减小 `Kp`、`Kd`，或增大前视距离；
  - 降低 `dt` 或适当降低 `u`（船速）。
- Excel 输出会覆盖同名文件，请注意备份。

### 许可证 (License)

本项目遵循 MIT 许可协议。欢迎提 Issue/PR 进行改进与扩展。 

