# Ant Cooperative Transport — Technical Report

## 1. Executive summary / 执行摘要

This project is a Python-based, two-dimensional simulation of ants coordinating to locate and transport a rectangular food object through a scene containing obstacles. Its main experiment is implemented in [`transport_simulation.py`](./transport_simulation.py): a small navigator group searches for food and lays a pheromone trail while returning to the nest; transporter ants independently approach the food, claim positions around its perimeter, and apply forces that translate and rotate the food object.

本项目是一个基于 Python 的二维仿真，用于研究蚂蚁如何协同寻找并搬运场景中的矩形食物。仿真核心位于 [`transport_simulation.py`](./transport_simulation.py)：少量导航蚁负责搜索食物，并在返回巢穴途中留下信息素；搬运工蚁则独立接近食物、认领其周边位置，并施加作用力使食物平移和旋转。

The simulator combines:

- role-based ant behaviour and finite-state transitions;
- 基于角色划分的蚂蚁行为与有限状态转换；
- correlated random walk (CRW) and food-odor attraction;
- 相关随机游走（CRW）与食物气味吸引；
- a Bug2-inspired go-to-target / wall-following obstacle-avoidance controller;
- 受 Bug2 启发的“朝向目标 / 沿障碍边界行走”避障控制器；
- discrete pheromone particles with linear evaporation;
- 采用离散信息素粒子和线性挥发模型的信息素机制；
- perimeter attachment slots and a simple rigid-body-like food dynamics model;
- 食物周边的搬运槽位与简化的类刚体动力学模型；
- Matplotlib animation for visual inspection.
- 使用 Matplotlib 动画进行可视化观察。

It is a behavioural simulation rather than a hardware-ready robot system. Ants are represented as points, sensing and communication are idealized, and the physics and collision handling are intentionally lightweight.

这是一个行为仿真原型，而非可直接部署到实体机器人的系统。蚂蚁以质点表示，感知和通信经过理想化处理，物理过程与碰撞处理也采用了轻量化模型。

## 2. Project scope and files / 项目范围与文件

| File | Responsibility |
|---|---|
| [`transport_simulation.py`](./transport_simulation.py) | Headless cooperative-transport model and per-frame update loop; does not import plotting libraries. |
| [`transport_visualization.py`](./transport_visualization.py) | Matplotlib renderer that reads simulation state and draws it without advancing the model. |
| [`run_animation.py`](./run_animation.py) | Animation entry point that connects the model and renderer. |
| [`run_experiments.py`](./run_experiments.py) | Headless batch runner that writes per-run summary metrics and optional per-step diagnostics to CSV. |
| [`ant_crw.py`](./ant_crw.py) | Ant model used by the transport simulation: roles, states, CRW, pheromone-heading calculation, and Bug2 state fields. |
| [`food.py`](./food.py) | Rectangular food object, perimeter slots, translation/rotation integration, and corner coordinates for rendering. |
| [`obstacle.py`](./obstacle.py) | Convex-polygon geometry, point/segment tests, position correction, and food collision response; rectangles remain supported. |
| [`pheromone.py`](./pheromone.py) | Pheromone particle storage, intensity clamping, evaporation, and removal of depleted particles. |
| [`ant.py`](./ant.py) | Earlier, separate ant movement model with random movement and food/pheromone attraction. |
| [`single-ant-model.py`](./single-ant-model.py) | Standalone animation of one ant taking uniformly random steps. |
| [`swarm.py`](./swarm.py) | Standalone local-neighbour perception example using the earlier `ant.py` model. |
| [`README.md`](./README.md) | Chinese project overview, module map, animation/batch commands, and dependency notes. |

| 文件 | 职责 |
|---|---|
| [`transport_simulation.py`](./transport_simulation.py) | 无窗口协同搬运仿真模型和逐帧更新循环；不导入绘图库。 |
| [`transport_visualization.py`](./transport_visualization.py) | 只读取仿真状态并绘图、不推进模型的 Matplotlib renderer。 |
| [`run_animation.py`](./run_animation.py) | 动画运行入口，负责连接仿真模型和绘图器。 |
| [`run_experiments.py`](./run_experiments.py) | 无窗口批量运行入口，将每次运行摘要及可选的逐步诊断写入 CSV。 |
| [`ant_crw.py`](./ant_crw.py) | 主运输仿真使用的蚂蚁模型：角色、状态、CRW、信息素朝向计算和 Bug2 状态数据。 |
| [`food.py`](./food.py) | 矩形食物对象、周边槽位、平移/旋转积分和绘图角点计算。 |
| [`obstacle.py`](./obstacle.py) | 凸多边形障碍的几何判断、位置修正和食物碰撞响应；兼容矩形障碍。 |
| [`pheromone.py`](./pheromone.py) | 信息素粒子存储、强度限制、挥发和耗尽粒子清理。 |
| [`ant.py`](./ant.py) | 早期独立的蚂蚁移动模型，包含随机移动和食物/信息素吸引。 |
| [`single-ant-model.py`](./single-ant-model.py) | 单只蚂蚁随机移动的独立动画示例。 |
| [`swarm.py`](./swarm.py) | 使用早期 `ant.py` 模型进行局部邻居感知的独立示例。 |
| [`README.md`](./README.md) | 中文项目简介、模块结构、动画/批量运行命令和依赖说明。 |

The main runtime is split into a model path (`transport_simulation.py` → `ant_crw.py`, `food.py`, `obstacle.py`, and `pheromone.py`) and two front ends: `run_animation.py` for animation and `run_experiments.py` for headless batches. The earlier ant model and the two small demonstrations are separate examples, not components of the transport simulation.

主运行路径拆分为仿真模型（`transport_simulation.py` → `ant_crw.py`、`food.py`、`obstacle.py` 和 `pheromone.py`）以及两个前端：用于动画展示的 `run_animation.py` 和用于无窗口批量实验的 `run_experiments.py`。早期蚂蚁模型及两个小型演示是独立示例，并不属于协同搬运仿真的运行组件。

## 3. System architecture / 系统架构

```mermaid
flowchart TD
    Sim[ObstacleTransportSimulation<br/>transport_simulation.py]
    Ant[Ant agents<br/>ant_crw.py]
    Food[Food rigid body and slots<br/>food.py]
    Obs[Convex polygon obstacles<br/>obstacle.py]
    Pher[Pheromone particle manager<br/>pheromone.py]
    Renderer[TransportRenderer<br/>transport_visualization.py]
    Anim[Animation entry point<br/>run_animation.py]
    Batch[Headless batch runner<br/>run_experiments.py]

    Sim --> Ant
    Sim --> Food
    Sim --> Obs
    Sim --> Pher
    Sim --> Renderer
    Renderer --> Anim
    Batch --> Sim
    Ant -->|position, role, state, heading| Sim
    Pher -->|particles queried by| Sim
    Sim -->|roles, headings, state updates| Ant
    Ant -->|forces and torques| Food
    Obs -->|ant and food collision handling| Sim
    Food -->|position, rotation, corners| Renderer
    Renderer -->|draw calls| Anim
    Pher -->|particle coordinates and intensity| Renderer
```

`ObstacleTransportSimulation` is the orchestration layer. It owns the scene and all agents, advances one simulation time step in `update()`, and passes information between the modules. The model module does not import NumPy or Matplotlib. `TransportRenderer` reads the current model state and refreshes the Matplotlib artists; the animation entry point advances the model before asking the renderer to draw. The batch runner uses the same model without creating a window.

`ObstacleTransportSimulation` 是系统的协调层，负责管理场景和所有蚂蚁，在 `update()` 中推进一个仿真时间步，并在不同模块之间传递信息。仿真模型模块不导入 NumPy 或 Matplotlib。`TransportRenderer` 只读取当前模型状态并刷新 Matplotlib 图元；动画入口先推进模型，再调用绘图器刷新画面。批量运行器复用同一仿真模型，不创建图形窗口。

### 3.1 Scene configuration / 场景配置

The animation entry point currently creates a `150 × 150` arena with:

- nest at `(70, 130)`;
- 巢穴位于 `(70, 130)`；
- food centre initially at `(110, 30)`, with size `6 × 6` and mass `5`;
- 食物初始中心位于 `(110, 30)`，尺寸为 `6 × 6`，质量为 `5`；
- two axis-aligned obstacles centred at `(40, 60)` and `(120, 60)`, each sized `40 × 6`;
- 两个轴对齐障碍物分别以 `(40, 60)` 和 `(120, 60)` 为中心，尺寸均为 `40 × 6`；
- a solid diamond obstacle with vertices `(75, 84)`, `(83, 92)`, `(75, 100)`, and `(67, 92)`;
- 一个实体菱形障碍物，顶点为 `(75, 84)`、`(83, 92)`、`(75, 100)` 和 `(67, 92)`；
- 35 ants in the animation example, with the first five assigned navigator roles;
- 动画示例中共有 35 只蚂蚁，前五只被指定为导航蚁；
- nominal ant speed `2.0` coordinate units per simulation update;
- 蚂蚁的标称速度为每次仿真更新移动 `2.0` 个坐标单位；
- simulation time step `dt = 0.05`.
- 仿真时间步长为 `dt = 0.05`。

Ant initial positions and headings, transport durations, and some movement decisions use Python's `random` module. `run_animation.py` does not set a random seed, so animation runs are not expected to reproduce identical trajectories. `run_experiments.py` seeds each run explicitly for repeatability.

蚂蚁的初始位置和朝向、搬运持续时间以及部分移动决策都使用 Python 的 `random` 模块。`run_animation.py` 不设置随机种子，因此不同动画运行之间的轨迹通常不会完全一致；`run_experiments.py` 会为每次运行显式设置种子，以便复现实验。

Obstacles can be defined as axis-aligned rectangles with `Obstacle(x, y, width, height)` or as convex polygons with `Obstacle.from_vertices([...])`. Polygon vertices must be ordered around the boundary; either clockwise or counter-clockwise order is accepted. The animation scene includes one solid diamond. Point collision, Bug2 wall following, line-of-sight tests, rendering, and food collision handling all use the polygon boundary.

障碍物既可通过 `Obstacle(x, y, width, height)` 定义为轴对齐矩形，也可通过 `Obstacle.from_vertices([...])` 定义为凸多边形。多边形顶点须按边界顺序排列，顺时针或逆时针均可。动画场景包含一个实体菱形；点碰撞、Bug2 沿边绕行、视线检测、绘图和食物碰撞处理均使用多边形边界。

## 4. Agent roles and state machines / 蚂蚁角色与状态机

Each transport-simulation ant has a `role`, a behaviour `state`, a heading, and Bug2-related obstacle state. Roles and states are distinct: role describes the ant's persistent responsibility, while state describes its current activity.

主运输仿真中的每只蚂蚁都具有 `role`（角色）、`state`（行为状态）、朝向以及 Bug2 相关避障状态。角色和状态含义不同：角色表示蚂蚁长期承担的职责，状态表示它当前正在执行的活动。

### 4.1 Navigator ants / 导航蚁

Navigator ants cycle through:

1. **`free`** — Search for food. The navigator calls `compute_pheromone_heading()` with an empty pheromone list, so its search uses food odor when the food is within the 60-unit odor radius and CRW otherwise. It does not follow existing pheromone trails while searching.
   **`free`（自由搜索）**——寻找食物。导航蚁调用 `compute_pheromone_heading()` 时传入空的信息素列表，因此仅在食物处于 60 个单位的气味感知范围内时追踪食物，否则执行 CRW。搜索期间不会跟随已有信息素。
2. **`returning`** — Once within 15 units of the food and with an unobstructed line of sight to its centre, the navigator heads toward the nest using the obstacle-avoidance controller. It deposits a pheromone particle at its current position on each update.
   **`returning`（返巢）**——导航蚁须进入食物 15 个单位范围内，且到食物中心的视线未被障碍物阻挡，才会使用避障控制器返回巢穴，并在每次更新时于当前位置释放一个信息素粒子。
3. **Return to `free`** — Within 15 units of the nest, it resets its Bug2 state and starts searching again. This cycle repeats; there is no one-time mission-completion flag that permanently stops a navigator.
   **返回 `free`**——进入巢穴 15 个单位范围内后，蚂蚁重置 Bug2 状态并重新开始搜索。这个循环会持续重复；当前没有“一次任务完成后永久停止导航蚁”的标志。

Thus the trail is generated during each food-to-nest return leg. Pheromone strength is added as `1.0` and is subsequently reduced by the manager.

因此，导航蚁每次从食物返回巢穴时都会释放信息素。每次添加的信息素初始强度为 `1.0`，之后由管理器逐步降低。

For return trips, the first navigator that encounters each obstacle establishes its wall-following direction. Later returning navigators reuse that direction for the same obstacle, while free-search ants retain the independent side-based rule. This shared per-simulation choice is intended to reduce split pheromone trails; it does not use pheromone to select the direction.

返巢途中，每个障碍物由首只遇到它的导航蚁确定沿边方向；之后返巢的导航蚁遇到同一障碍时沿用该方向。自由搜索蚁仍使用各自根据所在侧决定方向的原规则。这个仿真级共享选择旨在减少信息素路径分叉，本身不依赖信息素来决定方向。

### 4.2 Transporter ants / 搬运工蚁

Transporter ants cycle through:

1. **`free`** — Approach the food using food odor and CRW, without directly following pheromone in this movement phase.
   **`free`（自由搜索）**——结合食物气味和 CRW 接近食物；在此移动阶段不会直接跟随信息素。
2. **`transporting`** — Claim a perimeter slot and contribute a force to the food. A transporter stops carrying if it exceeds its randomized maximum transport time (6–12 seconds of simulation time) or gets more than 35 units from the food.
   **`transporting`（搬运）**——认领食物周边槽位并对食物施力。若搬运时间超过随机设定的最长时限（仿真时间 6–12 秒），或蚂蚁与食物距离超过 35 个单位，则停止搬运。
3. **`escaping`** — Move randomly for a short recovery interval, then return to `free` and search again.
   **`escaping`（暂时撤离/恢复）**——随机移动一段恢复时间，随后回到 `free` 状态重新搜索。

The usual escape interval after fatigue is four seconds; if an existing transporter cannot reclaim a slot during the frame's slot reassignment, it is moved into `escaping` with a two-second timer.

疲劳后的常规撤离时间为 4 秒。如果已有搬运工在当帧槽位重新分配时无法重新认领槽位，则会进入 `escaping`，计时器设为 2 秒。

## 5. Navigation and obstacle avoidance / 导航与避障

### 5.1 Correlated random walk and odor attraction / 相关随机游走与气味吸引

`Ant.compute_pheromone_heading()` implements a prioritized heading selection:

1. If pheromone signal weight exceeds `signal_threshold`, steer toward the weighted pheromone direction.
   如果信息素信号权重大于 `signal_threshold`，则朝加权计算得到的信息素方向移动。
2. Otherwise, if food is within `food_odor_radius`, steer toward the food.
   否则，如果食物位于 `food_odor_radius` 气味感知范围内，则朝食物移动。
3. Otherwise, perturb the current heading with Gaussian noise, producing a correlated rather than fully independent random walk.
   如果两种信号都不可用，则对当前朝向叠加高斯噪声，从而形成具有方向相关性的随机游走，而非每一步完全独立地随机选向。

上述三条策略具有优先级：信息素信号有效时优先跟随信息素；否则在食物进入气味范围时追踪食物；两者都不可用时执行 CRW。

For a pheromone particle at position \(p_i\), ant position \(x\), intensity \(I_i\), and distance \(d_i\), the heading estimator uses a contribution proportional to:

\[
w_i = \frac{I_i}{d_i^{1.5} + 0.1}
\]

and sums unit vectors pointing from the ant to the particles. The result is a local attraction vector, not an explicit estimate of the trail's forward tangent.

该算法将从蚂蚁指向各信息素粒子的单位向量按权重求和，得到局部吸引方向。这个向量表示“朝附近信息素靠近”，并没有显式估计信息素路径的前进切线方向。

### 5.2 Bug2-inspired obstacle handling / 受 Bug2 启发的避障

`ObstacleTransportSimulation._bug2_step()` uses two phases:

- **`go`** — Point the ant at its target and test the predicted next point against the obstacles. If the point is clear, move; if it is inside an obstacle, record that obstacle and enter `wall`.
- **`go`（前往目标）**——让蚂蚁朝目标移动，并检查预测的下一位置是否与障碍物碰撞。若位置无碰撞则正常移动；若进入障碍物，则记录该障碍并切换到 `wall`。
- **`wall`** — Follow a tangent direction on the recorded obstacle boundary. Return to `go` after the line segment from the ant to the target has been clear for two consecutive checks.
- **`wall`（沿墙绕行）**——沿已记录障碍边界的切线方向移动。当蚂蚁到目标的线段连续两次检测均无遮挡时，切回 `go`。

The selected wall-follow direction is stored on the ant when it enters `wall`. The wall controller uses a nearest-edge tangent, angular interpolation, and a deterministic fallback direction when the proposed next point would still lie inside the obstacle. A final position clamp pushes an ant that has entered the obstacle back to its nearest exterior edge.

蚂蚁进入 `wall` 时会保存选定的绕行方向。沿墙控制器根据最近的障碍边确定切线方向，并通过角度插值平滑调整朝向；若预测位置仍在障碍内部，则采用确定性的备用方向。最后的位置投影会将进入障碍内部的蚂蚁推回最近的外侧边缘。

This should be described as **Bug2-inspired**, not a strict textbook Bug2 implementation: the current logic does not explicitly track the start point, the M-line intersection condition, or progress along the M-line. Its exit rule is based on line-of-sight visibility from the current position to the target, tested against the current obstacle.

因此，更准确的名称是**受 Bug2 启发的算法**，而非严格的经典 Bug2 实现：当前逻辑没有显式追踪起点、M-line（起点与目标连线）交点条件或沿 M-line 的进度。退出绕障状态的条件是检查当前蚂蚁到目标的视线是否被正在绕行的障碍挡住。

### 5.3 Pheromone interpretation in cooperative transport / 协同搬运中的信息素使用方式

While calculating transporter forces, each transporter independently scans pheromone particles within a 60-unit perception radius. Contributions are weighted as:

搬运工计算作用力时，会各自扫描 60 个单位感知半径内的信息素粒子。每个粒子的贡献按以下权重计算：

\[
w_i = \frac{I_i}{d_i^{1.2} + 0.1}
\]

If accumulated pheromone weight is above `0.02`, that transporter uses the normalized attraction vector as its force direction. Otherwise, its direction is blended from its own heading (weight `0.4`) and a weighted group direction (weight `0.6`).

若信息素累计权重大于 `0.02`，该搬运工就使用归一化后的信息素吸引向量作为其施力方向。否则，将蚂蚁自身朝向（权重 `0.4`）与群体加权方向（权重 `0.6`）混合。

The two pheromone calculations therefore have different distance exponents: `1.5` for the ant heading helper and `1.2` for transporter force direction. Navigator search passes no pheromone particles, while transporters sense particles during force calculation after they have attached to the food. This distinction matters when interpreting what “following the trail” means in the simulation.

因此，两处信息素计算使用了不同的距离指数：蚂蚁朝向辅助函数中为 `1.5`，搬运工施力方向计算中为 `1.2`。导航蚁搜索时传入空信息素列表；搬运工则是在附着到食物后、计算作用力时读取信息素。理解“跟随信息素”在本仿真中的含义时应注意这一差异。

Transporter force direction now estimates the local pheromone trail axis from particles within 20 units using an intensity/distance-weighted covariance. At least four particles and an anisotropy confidence of `0.55` are required before the PCA axis is accepted. Its sign is chosen using that individual ant's remembered pheromone-following heading. If the axis is ambiguous, the controller falls back to the weighted particle resultant when its directional confidence is at least `0.15`; otherwise it retains the individual's remembered heading. This is an individual-level local estimate, not a colony-wide or food-level commanded heading.

搬运蚁施力方向现会使用 20 个单位范围内的信息素粒子，通过强度/距离加权协方差估计局部轨迹主轴。只有局部粒子不少于 4 个且各向异性置信度达到 `0.55` 时才采用 PCA 主轴；方向正反由该蚂蚁自身记忆的跟随朝向确定。若主轴存在歧义，则在加权粒子合向量方向置信度至少为 `0.15` 时回退到该合向量，否则保持个体自身记忆的方向。这是个体层面的局部估计，不是蚁群或食物层面的统一指令方向。

## 6. Cooperative transport mechanics / 协同搬运机制

### 6.1 Perimeter slots / 周边槽位

`Food` creates attachment slots along all four edges using three-unit spacing in the main simulation. Slot locations are stored in food-local coordinates and transformed into world coordinates using the food's current rotation and translation.

主仿真中的 `Food` 会沿矩形四条边以 3 个坐标单位的间距生成附着槽位。槽位位置以食物局部坐标保存，并根据食物当前的旋转和平移转换到世界坐标。

During each update, the food's slot occupancy is reset. Existing transporters attempt to claim a nearby slot first; free transporters then attempt to claim remaining slots. `claim_slot()` selects the nearest available slot and accepts the claim only if it is within 10 units.

每次更新开始时会清空槽位占用记录。现有搬运工先尝试认领附近槽位，随后自由状态的搬运工再尝试认领剩余槽位。`claim_slot()` 选择距离最近的空槽位，只有距离不超过 10 个单位时才认领成功。

This is a local, greedy allocation procedure: it does not optimize assignments globally, nor does it explicitly reserve an ant's previous slot across frames. The update order determines which ant gets first choice.

这是一个局部贪心分配过程：不会进行全局最优匹配，也不会显式地跨帧保留蚂蚁之前的槽位。哪只蚂蚁先获得槽位选择机会由更新顺序决定。

### 6.2 Ant force and food torque / 蚂蚁作用力与食物转矩

Each active transporter contributes a force vector with fixed magnitude `2.0` along its selected intention direction. The ant's carrying duration changes its weight in the group-heading calculation:

每只正在搬运的蚂蚁都会沿选定的运动意图方向施加大小固定为 `2.0` 的力。蚂蚁的搬运时长会影响其在群体朝向计算中的权重：

\[
\text{weight}_j = 1 + 0.3\,t_j
\]

where \(t_j\) is that ant's current transport time. This weight affects the weighted social direction; when an ant detects pheromone, its own force direction is instead set directly to its pheromone attraction direction.

其中 \(t_j\) 表示第 \(j\) 只蚂蚁当前的搬运时间。该权重用于计算加权社会方向；如果蚂蚁检测到信息素，则它自身的施力方向直接采用信息素吸引方向。

For an ant attached at rotated lever arm \(r=(r_x,r_y)\) and force \(F=(F_x,F_y)\), the food torque contribution is the two-dimensional cross product:

\[
\tau = r_x F_y - r_y F_x
\]

Forces and torques from the active transporters are summed and passed to `Food.update_dynamics()`.

对于附着在位置 \(r=(r_x,r_y)\) 的蚂蚁，若其作用力为 \(F=(F_x,F_y)\)，则它对食物产生的二维转矩由叉积计算，如上式所示。所有正在搬运的蚂蚁贡献的合力与合转矩会求和后传给 `Food.update_dynamics()`。

Before integration, the summed force is projected against obstacle contact normals when the food is overlapping an obstacle or within one coordinate unit of it. The component directed into an obstacle is removed while the tangential component is retained. If contact blocks at least 75% of the requested resultant force, the controller selects a feasible tangent direction that best advances the food toward the nest, applies at least one ant's nominal force in that direction, and suppresses torque for that update to avoid spinning in place.

积分前，如果食物与障碍物重叠或距离障碍物不超过一个坐标单位，程序会依据接触法线投影合力：移除指向障碍内部的分量，并保留沿障碍切线方向的分量。如果障碍接触使合力损失达到或超过 75%，控制器会从可行切线方向中选择最能使食物靠近巢穴的方向，施加至少一只蚂蚁的标称拉力，并抑制该帧合转矩，避免食物顶住障碍原地自转。

### 6.3 Food dynamics / 食物动力学

The food tracks centre position `(x, y)`, angle `theta`, linear velocity `(vx, vy)`, angular velocity `omega`, mass, and rectangular-body moment of inertia:

\[
I = \frac{m(w^2+h^2)}{12}
\]

食物状态包括中心位置 `(x, y)`、角度 `theta`、线速度 `(vx, vy)`、角速度 `omega`、质量，以及矩形刚体的转动惯量。其转动惯量按上式计算，其中 \(m\) 为质量，\(w\) 和 \(h\) 分别为矩形宽度和高度。

Translation and rotation are advanced with a simple explicit time-step update. Linear and angular damping reduce velocities over time. After the food moves, the simulation checks it against each obstacle using the separating axis theorem, applies a minimum-translation correction, and reduces velocity into the obstacle using a restitution coefficient of `0.2`.

食物平移和旋转采用简单显式时间步积分。线性阻尼和角阻尼会逐步降低速度。食物移动后，仿真使用分离轴定理检测其与障碍物的重叠，按最小平移距离将食物推出，并使用 `0.2` 的恢复系数衰减朝向障碍物的速度分量。

Once the food has moved, transporter positions are interpolated `40%` of the way toward their newly transformed slot positions. This is a visual/behavioural attachment approximation rather than a constraint-based rigid-body solver.

食物移动后，搬运工的位置会向其更新后的槽位坐标移动差值的 `40%`。这是一种用于行为和视觉表现的附着近似，并非基于约束的刚体求解器。

## 7. Pheromone model / 信息素模型

`PheromoneManager` represents pheromone as a list of independent particles, each containing `(x, y, intensity)`.

`PheromoneManager` 将信息素表示为一组相互独立的粒子，每个粒子包含 `(x, y, intensity)`。

- New particle intensity is clamped to `[0.1, 1.0]`.
- 新粒子的强度被限制在 `[0.1, 1.0]` 范围内。
- Each manager update subtracts a fixed `evaporation_rate` from every particle.
- 管理器每次更新时都会从每个粒子的强度中减去固定的 `evaporation_rate`。
- Particles with intensity at or below zero are removed.
- 强度小于或等于零的粒子会被移除。
- The main simulation uses evaporation rate `0.003` per simulation update.
- 主仿真中每次仿真更新的信息素挥发率为 `0.003`。

The model does not diffuse particles, merge nearby particles, or store trail identity, age, or a directional vector. It is therefore a discrete spatial memory field rather than a continuous chemical diffusion model.

该模型不会扩散或合并粒子，也不记录路径身份、粒子年龄或方向向量。因此，它是一个离散的空间记忆场，而非连续的化学扩散模型。

## 8. Per-frame update sequence / 每帧更新顺序

The main `update()` performs the following operations in order:

1. Move navigator ants and update their search/return states; returning navigators deposit pheromone.
   更新导航蚁位置及搜索/返巢状态；正在返巢的导航蚁释放信息素。
2. Update transporter fatigue and escape behaviour.
   更新搬运工的疲劳和撤离行为。
3. Clear food slot occupancy for this frame.
   清空本帧的食物槽位占用记录。
4. Move free transporters toward food.
   让自由状态的搬运工向食物移动。
5. Reassign slots first to active transporters and then to newly arriving free transporters.
   先为当前搬运工重新分配槽位，再让新到达的自由蚂蚁认领剩余槽位。
6. Build active transporter intentions; calculate pheromone/social force directions and food torques.
   汇总搬运工的运动意图，计算信息素/社会方向、施加到食物上的力以及转矩。
7. Integrate food translation and rotation, then resolve food-obstacle overlap.
   积分更新食物的平移和旋转，再处理食物与障碍物的重叠。
8. Move transporter agents toward their occupied slots and constrain all ants against obstacles.
   将搬运工移向各自槽位，并将所有蚂蚁的位置限制在障碍物之外。
9. Evaporate pheromone and remove depleted particles.
   使信息素挥发，并移除已经耗尽的粒子。

This ordering means pheromone deposition occurs before transporter force sensing in the same frame, and food motion occurs after slot assignment and force calculation.

这种执行顺序意味着：同一帧中先释放信息素，再计算搬运工的力；食物运动则发生在槽位分配和受力计算之后。

## 9. Visualization and execution / 可视化与运行

When executed directly, `run_animation.py` constructs the simulation and `TransportRenderer`, then creates a Matplotlib animation with:

直接运行 `run_animation.py` 时，脚本会创建仿真实例和 `TransportRenderer`，并启动 Matplotlib 动画，画面包括：

- grey rectangles for obstacles;
- 灰色矩形表示障碍物；
- a green star for the nest;
- 绿色星形标记表示巢穴；
- red semi-transparent markers for pheromone particles;
- 红色半透明标记表示信息素粒子；
- ant markers coloured by role/state (navigator, transporting, escaping, or free);
- 根据角色/状态着色的蚂蚁标记（导航、搬运、撤离或自由搜索）；
- a purple polygon for the food, updated from its rotated corners;
- 根据旋转后角点更新的紫色食物多边形；
- a title reporting the number of active transporters and returning navigators.
- 显示当前搬运工数量和返巢导航蚂蚁数量的标题。

The animation is configured for 600 frames with a nominal 50 ms interval. `sim.update()` uses `dt = 0.05`, so the intended simulated time step corresponds nominally to the animation interval; actual wall-clock timing depends on rendering and machine performance.

动画默认配置为 600 帧，名义帧间隔为 50 毫秒。`sim.update()` 使用 `dt = 0.05`，因此设计上的仿真时间步与动画帧间隔大致对应；实际墙钟时间仍取决于绘图性能和计算机速度。

The animation renderer and standalone visualization scripts import NumPy and Matplotlib. The simulation core and batch runner do not import either plotting library.

动画绘图器和独立可视化脚本依赖 NumPy 与 Matplotlib；仿真核心和批量运行器不导入这两个绘图库。

Run the animation from the project directory with:

从项目目录运行动画的命令如下：

```powershell
python run_animation.py
```

For windowless batch experiments, use `run_experiments.py`. For example, run three independent 600-step simulations and write their summary rows to a CSV file:

```powershell
python run_experiments.py --runs 3 --steps 600 --seed 10 --output results.csv
```

Each run uses a reproducible seed derived from the base seed. The CSV records the final food position and angle, displacement from its initial position, number of active transporters and returning navigators, and remaining pheromone-particle count. The default output path is `experiment_results.csv`; the runner does not render animation frames.

无窗口批量实验可使用 `run_experiments.py`。例如，运行三组各 600 步的独立仿真，并将摘要结果写入 CSV：

```powershell
python run_experiments.py --runs 3 --steps 600 --seed 10 --output results.csv
```

每次运行使用由基础随机种子推导出的可复现种子。CSV 记录食物最终位置和角度、相对初始位置的位移、活跃搬运工数量、返巢导航蚂蚁数量以及剩余信息素粒子数。默认输出路径为 `experiment_results.csv`；批量运行器不会绘制动画帧。

`run_experiments.py` defaults to one run, 600 steps, base seed `0`, and output file `experiment_results.csv`. The example above overrides these values. A run count or step count below one is rejected.

`run_experiments.py` 默认执行 1 组仿真、每组 600 步、基础随机种子为 `0`，输出文件为 `experiment_results.csv`。上面的命令覆盖了这些默认值。运行组数或步数小于 1 时，程序会拒绝运行。

An optional `--diagnostics-output` path enables a separate, streamed CSV with one row per active transporter per step. Rows contain the current pheromone attraction vector and its directional confidence (vector magnitude divided by total signal weight), local PCA trail tangent and confidence, the selected direction source, individual heading and requested force, obstacle-following state, obstacle-contact count, requested/projected group force and torque, and per-step food displacement, speed, and angular velocity. Angle fields are in radians; individual force is the requested force before obstacle projection, while the projected resultant is reported separately. Direction source `tangent` means a reliable local PCA axis oriented by that ant's own remembered heading; `pheromone_resultant` is the legacy weighted-particle direction; `memory` is retained when the tangent is ambiguous and resultant confidence is below `0.15`. Steps with no active transporters still receive a summary row. The diagnostic file is not created unless requested and must be different from the summary output path.

可通过可选参数 `--diagnostics-output` 启用独立的流式诊断 CSV：每个仿真步中每只活跃搬运蚁各占一行。记录字段包括信息素合成向量及方向可信度（向量长度除以信号权重总和）、局部 PCA 路径切线及置信度、方向选择来源、个体朝向和期望拉力、避障状态、障碍接触数、投影前后的群体合力与转矩，以及食物逐步位移、速度和角速度。角度字段单位为弧度；个体拉力表示障碍投影前的期望拉力，投影后的群体合力单独记录。方向来源 `tangent` 表示按该蚂蚁自身记忆定向的可信 PCA 主轴，`pheromone_resultant` 表示原有加权粒子方向；当局部切线含糊且合向量置信度低于 `0.15` 时，来源为 `memory`，沿用该蚂蚁自己的上次路线方向。没有活跃搬运蚁的时间步也会写入一条汇总行。只有显式指定路径时才创建诊断文件，且该路径必须与摘要文件不同。

No dependency manifest is present in the project directory, so the required package versions are not pinned.

项目目录中没有依赖清单文件，因此尚未固定所需软件包的版本。

## 10. Other demonstrations and integration note / 其他演示与集成说明

The repository contains two earlier examples:

仓库中还包含两个较早期的示例：

- `single-ant-model.py` defines its own `Ant` class and animates a single ant taking a uniformly random direction each step.
- `single-ant-model.py` 定义独立的 `Ant` 类，并演示单只蚂蚁每一步均匀随机选择方向的动画。
- `swarm.py` uses `ant.py` to animate a group and calculate pairwise Euclidean-neighbour lists.
- `swarm.py` 使用 `ant.py` 对一群蚂蚁进行动画演示，并计算两两之间基于欧氏距离的邻居列表。

These are separate from the cooperative-transport entry points. In the current source, `swarm.py` calls `ant.move()`, but the imported `Ant` in `ant.py` defines `move_random()` and `move_with_pheromone()` instead of `move()`. Consequently, that standalone example appears to have an API mismatch and may raise `AttributeError` when run. The cooperative-transport model imports `Ant` from `ant_crw.py` and is not affected by this particular mismatch.

这些示例与协同搬运的动画/实验入口彼此独立。当前 `swarm.py` 调用了 `ant.move()`，但它从 `ant.py` 导入的 `Ant` 类只定义了 `move_random()` 和 `move_with_pheromone()`，没有 `move()`。因此，该独立示例可能存在 API 不匹配，运行时会触发 `AttributeError`。协同搬运仿真模型使用的是 `ant_crw.py` 中的 `Ant`，不受这一特定问题影响。

## 11. Current capabilities and limitations / 当前功能与限制

### Implemented capabilities / 已实现功能

- Separation of navigator and transporter roles.
- 区分导航蚁和搬运工角色。
- Food searching using odor-radius attraction and CRW.
- 使用气味范围吸引和 CRW 搜索食物。
- Navigator-generated pheromone trails on the return-to-nest leg.
- 导航蚁在返巢路段释放信息素。
- Pheromone-based direction selection during transporter force calculation.
- 搬运工计算作用力时根据信息素选择方向。
- Multiple ants attaching at different positions around a movable food object.
- 多只蚂蚁可附着在可移动食物的不同位置。
- Translation and rotation caused by summed forces and torques.
- 合力和合转矩共同驱动食物平移与旋转。
- Basic obstacle checks for ant motion and food motion.
- 对蚂蚁移动和食物移动进行基础障碍检测。
- Fatigue/recovery cycles for transporter ants.
- 搬运工具有疲劳与恢复周期。
- Animated visualization of the main simulation.
- 主仿真支持动画可视化。

### Modelling limitations / 建模限制

- **Idealized sensing:** Ants have no sensor model, occlusion model, or measurement noise beyond selected heading perturbations.
- **感知理想化：**除部分朝向扰动外，蚂蚁没有传感器模型、遮挡模型或测量噪声模型。
- **Point-agent collision:** Ants are treated as points; their body radius is not included in obstacle clearance.
- **质点蚂蚁碰撞：**蚂蚁被当作点处理，避障间隙未考虑蚂蚁自身半径。
- **Simplified Bug2 controller:** The implementation uses visibility-based exit logic but not the full canonical Bug2 M-line procedure. Collision is checked at the predicted endpoint, so thin obstacles can be missed if step sizes are increased.
- **简化的 Bug2 控制器：**实现采用可视性判定退出绕障，但并未实现经典 Bug2 的完整 M-line 过程。碰撞只在预测终点处检查，因此若增大步长，可能越过较薄障碍。
- **Pheromone direction ambiguity:** A vector pointing toward nearby pheromone particles is not necessarily the forward direction along a trail. Intersections or overlapping trails can create a misleading resultant vector.
- **信息素方向存在歧义：**指向附近信息素粒子的向量不一定是沿路径前进的方向；路径交叉或重叠可能产生误导性的合成向量。
- **Hard signal threshold:** The transporter controller switches between pheromone-following and social-heading modes at a fixed threshold; it does not smoothly blend the two modes or explicitly model controller damping.
- **信号阈值硬切换：**搬运工控制器在固定阈值处切换信息素跟随和社会方向模式，没有平滑混合两种模式，也没有显式建模控制器阻尼。
- **Simplified rigid-body physics:** The food uses basic explicit integration and a minimum-translation collision correction based on the separating axis theorem. It does not model contact points, friction, or angular impulse from obstacle impacts.
- **刚体物理简化：**食物运动采用基础显式积分，并使用分离轴定理进行最小平移碰撞修正；尚未模拟接触点、摩擦或障碍物撞击产生的角冲量。
- **Approximate obstacle-aware force projection:** The resultant force is projected against nearby obstacle contact normals, with a nest-directed tangent fallback and torque suppression when most of the resultant force is blocked. This is a simple food-level correction, not a full contact solver; it does not redistribute individual ant forces or model friction and angular contact impulses.
- **近似的障碍感知力投影：**合力会依据邻近障碍物的接触法线投影；大部分合力被阻挡时，会回退到朝向巢穴的可行切线方向并抑制转矩。这只是食物层面的简化修正，并非完整接触求解器：它不会重新分配各蚂蚁的个体拉力，也不模拟摩擦和角向接触冲量。
- **Greedy attachment allocation:** Slot selection is nearest-available and sequential, not a coordinated global assignment.
- **贪心式附着分配：**槽位按最近空位顺序分配，而非通过全局协调完成分配。
- **Incomplete evaluation metrics:** The animation and batch CSV report active transporters and returning navigators; the batch runner also records food displacement and final pose for seeded runs. The project does not detect delivery completion or record path efficiency and collision counts, nor does it aggregate statistics across runs.
- **评估指标不完整：**动画和批量 CSV 都统计活跃搬运工及返巢导航蚁；批量运行器还记录设置随机种子后的食物位移和最终姿态。项目尚未检测送达完成状态，也未记录路径效率和碰撞次数，或汇总多次运行的统计结果。
- **No physical robot integration:** There is no ROS interface, actuator model, communication protocol, hardware calibration, or deployment layer.
- **未集成实体机器人：**项目没有 ROS 接口、执行器模型、通信协议、硬件标定或部署层。

## 12. Recommended next steps for research use / 面向研究用途的后续建议

For a more reliable cooperative-transport study, the most useful extensions would be:

1. Extend batch experiments with delivery detection, collision/path metrics, and aggregate statistics across multiple seeds.
   扩展批量实验功能，增加送达检测、碰撞/路径指标，并汇总多个随机种子的统计结果。
2. Replace pheromone-point attraction with local trail-tangent estimation and confidence-aware blending.
   将“吸引到信息素点”的策略替换为局部路径切线估计，并按方向置信度进行融合。
3. Smooth transporter direction changes with an explicit angular-rate or damped controller, rather than relying only on a hard signal threshold.
   使用显式角速度控制器或带阻尼控制器平滑搬运工转向，而不是只依赖硬阈值切换。
4. Replace the simple food-level force projection with contact-aware force redistribution and friction/torque handling if more realistic obstacle navigation is needed.
   如需更真实的障碍导航，可将食物层面的简化力投影升级为考虑接触的个体拉力重分配，并加入摩擦和转矩处理。
5. Use a standard 2D physics engine if contact forces, friction, angular impact response, or more complex shapes are required.
   如需接触力、摩擦、角向冲击响应或更复杂形状，可考虑使用标准二维物理引擎。
6. Add tests for slot assignment, state transitions, obstacle edge cases, and force/torque calculations.
   为槽位分配、状态转换、障碍边界情况以及力/转矩计算添加测试。
7. Add a dependency manifest with tested version constraints for the visualization dependencies.
   为可视化依赖添加包含已测试版本约束的依赖清单。

## 13. Conclusion / 结论

The project demonstrates a clear high-level cooperative strategy: navigator ants establish a nest-to-food pheromone route, transporters gather at a rigid food object, and their combined directional forces move that object. Its modular structure separates agents, food mechanics, obstacle geometry, pheromone lifecycle, and visualization, making it a useful prototype for exploring role allocation and collective transport.

本项目展示了一种清晰的高层协作策略：导航蚁建立巢穴与食物之间的信息素路径，搬运工聚集在刚体食物周围，通过合成方向作用力移动食物。模块化结构将蚂蚁、食物力学、障碍几何、信息素生命周期和可视化分离，适合作为研究角色分工与群体搬运的原型。

The main implementation gap between this prototype and a robust robot-coordination model is the coupling between navigation and rigid-body motion near obstacles. The current code senses pheromone locally and projects the resultant food force at nearby obstacle contacts, but it does not yet infer a stable trail direction or redistribute forces across individual transporters using a full contact model. Those areas, alongside reproducible metrics and more faithful collision dynamics, are the natural next stages of development.

该原型距离稳健的机器人协同模型，主要还差在障碍附近的导航与刚体运动耦合。目前代码能够局部感知信息素，并在食物接近障碍时投影合力，但尚未推断稳定的路径方向，也未通过完整接触模型在个体搬运蚁之间重分配拉力。下一步可继续完善这些方面，并补充可复现实验指标和更准确的碰撞动力学。
