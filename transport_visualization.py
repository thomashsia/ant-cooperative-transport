"""协同搬运仿真的 Matplotlib 绘图组件。

由 ``run_animation.py`` 创建；本模块只读取仿真状态并更新 artists，不负责推进模型。
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


class TransportRenderer:
    """负责将仿真当前状态绘制到 Matplotlib 画布，不推进仿真状态。

    动画入口调用 ``init`` 初始化画面、每帧调用 ``draw`` 刷新动态对象；
    固定的场地障碍、巢穴和图例只在初始化时绘制。
    """

    # 方向来源 -> 图例分组；直接跟随信息素、回退记忆、无信号社会混合分别着色。
    FORCE_SOURCE_GROUP = {
        'pheromone_resultant': 'pheromone',
        'tangent': 'pheromone',
        'memory': 'memory',
        'memory_social': 'memory',
        'social': 'social',
    }
    FORCE_GROUPS = {
        'pheromone': ('limegreen', 'Force: following pheromone'),
        'memory': ('darkorange', 'Force: memory fallback'),
        'social': ('crimson', 'Force: no signal / social'),
    }

    def __init__(self, simulation, width=150, height=150):
        """创建场景图层和动态绘图对象；由 ``run_animation`` 调用。"""
        self.simulation = simulation
        self.fig, self.ax = plt.subplots(figsize=(11, 8))
        self.ax.set_xlim(0, width)
        self.ax.set_ylim(0, height)

        # 障碍物和巢穴位置在仿真期间固定，只需绘制一次。
        for obs in simulation.obstacles:
            patch = Polygon(
                obs.vertices,
                closed=True,
                color='gray',
                alpha=0.7
            )
            self.ax.add_patch(patch)

        self.pheromone_scatter = self.ax.scatter(
            [], [], s=8, c='red', alpha=0.5, label='Pheromone (Navigator Ants)'
        )
        self.ant_scatter = self.ax.scatter(
            [], [], s=30, c='black', label='Transporter Ants'
        )
        # 个体拉力箭头按方向来源分组着色；箭头长度固定，便于比较朝向。
        self.force_quivers = {}
        for group, (color, label) in self.FORCE_GROUPS.items():
            self.force_quivers[group] = self.ax.quiver(
                [], [], [], [],
                angles='xy', scale_units='xy', scale=0.25,
                width=0.003, color=color, label=label, zorder=5
            )
        self.food_patch = Polygon(
            np.empty((4, 2)),
            closed=True,
            color='purple',
            alpha=0.8,
            label='Food Block'
        )
        self.ax.add_patch(self.food_patch)
        self.food_trajectory_x = [simulation.food.x]
        self.food_trajectory_y = [simulation.food.y]
        (self.food_trajectory_line,) = self.ax.plot(
            self.food_trajectory_x,
            self.food_trajectory_y,
            color='green',
            linewidth=1.5,
            alpha=0.8,
            label='Food Centroid Trajectory'
        )

        self.ax.plot(
            simulation.nest[0],
            simulation.nest[1],
            marker='*',
            markersize=15,
            color='green',
            label='Nest'
        )
        # 图例放在坐标系右侧之外，并为其预留画布空间。
        self.ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0)
        self.fig.subplots_adjust(right=0.72)

    def init(self):
        """将动态绘图对象设置为动画起始状态。

        由 ``matplotlib.animation.FuncAnimation`` 的 ``init_func`` 回调调用。
        """
        self.pheromone_scatter.set_offsets(np.empty((0, 2)))
        self.ant_scatter.set_offsets(np.empty((0, 2)))
        self.food_patch.set_xy(np.empty((4, 2)))
        self.food_trajectory_line.set_data(
            self.food_trajectory_x, self.food_trajectory_y
        )
        return self._artists()

    def draw(self):
        """读取仿真当前状态并刷新图形，不调用仿真的 ``update``。

        由动画帧回调在模型推进后调用；信息素强度映射为透明度，蚂蚁颜色映射角色/
        状态，食物质心轨迹仅在位置变化时追加，因此纯旋转不会产生轨迹点。
        """
        sim = self.simulation

        # 信息素坐标映射到散点位置，强度映射到粒子透明度。
        particles = sim.pheromone_manager.particles
        if particles:
            pheromone_coords = np.array([[p['x'], p['y']] for p in particles])
            pheromone_alphas = [
                min(1.0, max(0.1, p['intensity'])) for p in particles
            ]
            self.pheromone_scatter.set_offsets(pheromone_coords)
            self.pheromone_scatter.set_alpha(pheromone_alphas)
        else:
            self.pheromone_scatter.set_offsets(np.empty((0, 2)))

        # 依据蚂蚁角色和状态着色，突出显示各类行为。
        ant_coords = np.array([[ant.x, ant.y] for ant in sim.ants])
        ant_colors = []
        for ant in sim.ants:
            if ant.role == 'navigator':
                ant_colors.append('orange')
            elif ant.state == 'transporting':
                ant_colors.append('blue')
            elif ant.state == 'escaping':
                ant_colors.append('magenta')
            else:
                ant_colors.append('black')

        self.ant_scatter.set_offsets(ant_coords)
        self.ant_scatter.set_facecolors(ant_colors)
        self._update_force_arrows()

        # 食物角点由仿真模型根据当前位置和旋转角计算。
        self.food_patch.set_xy(sim.food.get_corners())
        if (
            sim.food.x != self.food_trajectory_x[-1]
            or sim.food.y != self.food_trajectory_y[-1]
        ):
            self.food_trajectory_x.append(sim.food.x)
            self.food_trajectory_y.append(sim.food.y)
        self.food_trajectory_line.set_data(
            self.food_trajectory_x, self.food_trajectory_y
        )

        transporter_count = sum(
            1 for ant in sim.ants
            if ant.role == 'transporter' and ant.state == 'transporting'
        )
        returning_navigator_count = sum(
            1 for ant in sim.ants
            if ant.role == 'navigator' and ant.state == 'returning'
        )
        self.ax.set_title(
            f"Division of Labor: Navigators & Transporters\n"
            f"Active Transporters: {transporter_count} | "
            f"Returning Navigators: {returning_navigator_count}",
            fontsize=10
        )

        return self._artists()

    def _update_force_arrows(self):
        """为每只 transporting 蚂蚁绘制其当前个体拉力方向。

        仅读取仿真在上一帧 ``update`` 中保存的障碍投影前个体拉力；箭头起点为蚂蚁位置。
        """
        grouped = {group: ([], [], [], []) for group in self.force_quivers}
        for ant in self.simulation.ants:
            force = getattr(ant, 'individual_force', None)
            if ant.role != 'transporter' or ant.state != 'transporting' or force is None:
                continue
            group = self.FORCE_SOURCE_GROUP.get(
                getattr(ant, 'force_direction_source', 'social'), 'social'
            )
            xs, ys, us, vs = grouped[group]
            xs.append(ant.x)
            ys.append(ant.y)
            us.append(force[0])
            vs.append(force[1])

        for group, (xs, ys, us, vs) in grouped.items():
            quiver = self.force_quivers[group]
            # Quiver 不随 set_offsets 更新箭头数量/位置缓存，需同步这两个字段。
            positions = np.column_stack((xs, ys)) if xs else np.empty((0, 2))
            quiver.N = len(xs)
            quiver.XY = positions
            quiver.set_offsets(positions)
            quiver.set_UVC(np.array(us), np.array(vs))

    def _artists(self):
        """返回需由 Matplotlib 动画系统重绘的动态 artists；供 ``init`` 和 ``draw`` 调用。"""
        return (
            self.pheromone_scatter,
            self.ant_scatter,
            self.food_patch,
            self.food_trajectory_line,
            *self.force_quivers.values()
        )
