# Ant Cooperative Transport

二维蚂蚁协同搬运仿真项目。仿真模型、Matplotlib 绘图和无窗口批量实验分别位于独立模块。

## 文件结构

- `transport_simulation.py`：仿真核心，不导入 NumPy 或 Matplotlib。
- `transport_visualization.py`：仿真状态的 Matplotlib 绘制器，不推进仿真。
- `run_animation.py`：动画运行入口。
- `run_experiments.py`：无窗口批量实验入口，将每次运行的摘要写入 CSV。
- `obstacle.py`：支持轴对齐矩形和凸多边形的实体障碍物几何。

## 运行动画

```powershell
python run_animation.py
```

## 批量运行

以下示例运行三组独立仿真，每组推进 600 步，并将结果写入 `results.csv`：

```powershell
python run_experiments.py --runs 3 --steps 600 --seed 10 --output results.csv
```

`--seed` 指定基础随机种子；每次运行会使用递增的种子，便于复现实验。批量运行不创建图形窗口，也不需要导入绘图模块。

需要分析搬运过程中的个体信息素信号、拉力方向和食物运动时，可额外指定诊断 CSV：

```powershell
python run_experiments.py --runs 1 --steps 600 --seed 10 --output results.csv --diagnostics-output transport_diagnostics.csv
```

诊断文件按仿真步和搬运蚁记录信息素合成向量、局部轨迹切线及置信度、方向来源、蚂蚁朝向/避障状态、个体期望拉力、障碍接触数，以及受阻投影前后的群体合力、食物位移和角速度。方向来源为 `tangent` 时使用可信的局部 PCA 主轴并按个体记忆确定正反方向；方向来源为 `pheromone_resultant` 时回退到原来的粒子合成方向；局部切线含糊且合向量方向可信度低于 `0.15` 时保持个体上次的路线方向。角度字段以弧度表示；个体拉力是障碍投影前的期望值，群体投影后的合力另行记录。没有活跃搬运蚁的时间步也会写入一条仅含群体/食物数据的记录。未指定该选项时，不生成诊断文件。

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