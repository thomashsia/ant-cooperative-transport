# Ant Cooperative Transport

二维蚂蚁协同搬运仿真项目。仿真模型、Matplotlib 绘图和无窗口批量实验分别位于独立模块。

## 文件结构

- `simulation_config.py`：核心仿真设置、群体规模和信息素相关参数的统一配置。
- `transport_simulation.py`：仿真核心，不导入 NumPy 或 Matplotlib。
- `transport_visualization.py`：仿真状态的 Matplotlib 绘制器，不推进仿真。
- `run_animation.py`：动画运行入口。
- `run_experiments.py`：无窗口批量实验入口，将每次运行的摘要写入 CSV。
- `tests/`：仿真感知、配置、实验元数据和渲染行为的回归测试。
- `obstacle.py`：支持轴对齐矩形和凸多边形的实体障碍物几何。

## 运行动画

```powershell
python run_animation.py
```

动画会持续绘制食物块质心的移动轨迹，使用橙色线条显示。

## 批量运行

以下示例运行三组独立仿真，每组推进 600 步，并将结果写入 `results.csv`：

```powershell
python run_experiments.py --runs 3 --steps 600 --seed 10 --output results.csv
```

`--seed` 指定基础随机种子；每次运行会使用递增的种子，便于复现实验。批量运行不创建图形窗口，也不需要导入绘图模块。未指定 `--output` 时，摘要写入被 Git 忽略的 `results/experiment_results.csv`。每条摘要记录都包含 `configuration_json`，保存该次仿真的完整配置。可通过 `--ants`、`--navigators`、`--width`、`--height` 和信息素相关参数选项覆盖默认设置，例如：

```powershell
python run_experiments.py --runs 5 --steps 600 --seed 10 --navigator-pheromone-radius 60 --transporter-pheromone-radius 20
```

运行回归测试（使用 Python 标准库 `unittest`）：

```powershell
python -m unittest discover -s tests
```

质心轨迹渲染测试需要动画依赖 NumPy 和 Matplotlib；核心仿真及实验运行器仍不依赖这两个绘图库。

需要分析搬运过程中的个体信息素信号、拉力方向和食物运动时，可额外指定诊断 CSV：

```powershell
python run_experiments.py --runs 1 --steps 600 --seed 10 --output results.csv --diagnostics-output transport_diagnostics.csv
```

诊断文件按仿真步和搬运蚁记录信息素合成向量、局部轨迹切线及置信度、方向来源、蚂蚁朝向/避障状态、个体期望拉力、障碍接触数，以及受阻投影前后的群体合力、食物位移和角速度。方向来源为 `tangent` 时使用可信的局部 PCA 主轴并按个体记忆确定正反方向；方向来源为 `pheromone_resultant` 时回退到原来的粒子合成方向；方向不确定且使用同伴修正时，来源为 `memory_social`。字段 `group_direction_confidence` 和 `fallback_social_weight` 记录同伴方向一致性及该修正权重。角度字段以弧度表示；个体拉力是障碍投影前的期望值，群体投影后的合力另行记录。没有活跃搬运蚁的时间步也会写入一条仅含群体/食物数据的记录。未指定该选项时，不生成诊断文件。

当存在信息素信号但其方向不确定时，transporter 会以个体路线记忆为主，并根据方向不确定度与其他搬运蚁朝向的一致性，有限融合群体方向（默认最大权重 35%，可用 `--ambiguous-pheromone-max-social-weight` 调整）。无其他搬运蚁或群体朝向分散时不会施加群体修正；诊断中的 `memory_social`、`group_direction_confidence` 和 `fallback_social_weight` 可用于识别并量化该回退行为。

所有信息素朝向计算都会忽略被实体障碍物遮挡的粒子；搬运蚁的方向向量和局部轨迹切线均只使用无遮挡的信息素。主仿真中的导航蚁搜索阶段当前不读取信息素。

导航蚁返巢时会在 60 个单位范围内排除自己释放的信息素；在 Bug2 的 `go` 阶段，仅当其他导航蚁留下的无遮挡局部切线足够可信且有助于接近巢穴时，才以最多 35% 的权重修正返巢朝向。`wall` 绕障阶段不受该信息素引导影响。搬运蚁的信息素合向量感知半径为 20 个单位（局部切线估计也为 20 个单位），以减少较远处不同轨迹分支的干扰。

Navigator 发现食物的距离阈值和返巢后恢复搜索的巢穴距离阈值均为 2 个单位；发现食物还要求到食物中心无遮挡。两项距离均由 `SimulationConfig` 管理。

## 依赖

动画和独立可视化示例需要 NumPy 与 Matplotlib。纯仿真核心和批量运行器不依赖这两个绘图库。

## 添加凸多边形障碍物

`Obstacle.from_vertices()` 接收沿多边形边界依次排列的顶点，顺时针或逆时针均可；顶点必须构成凸多边形。以下示例创建一个实体菱形并加入场景：

```python
from obstacle import Obstacle
from transport_simulation import ObstacleTransportSimulation

simulation = ObstacleTransportSimulation()
diamond = Obstacle.from_vertices([
    (75, 84),
    (83, 92),
    (75, 100),
    (67, 92),
])
simulation.obstacles.append(diamond)
```

仿真会使用该多边形进行蚂蚁碰撞、绕障、视线遮挡和食物碰撞检测，可视化也会按真实顶点绘制。