"""协同搬运仿真的 Matplotlib 绘图组件。"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


class TransportRenderer:
    """负责将仿真当前状态绘制到 Matplotlib 画布，不推进仿真状态。"""

    def __init__(self, simulation, width=150, height=150):
        """创建场景图层和用于动态更新的绘图对象。"""
        self.simulation = simulation
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
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
        self.food_patch = Polygon(
            np.empty((4, 2)),
            closed=True,
            color='purple',
            alpha=0.8,
            label='Food Block'
        )
        self.ax.add_patch(self.food_patch)

        self.ax.plot(
            simulation.nest[0],
            simulation.nest[1],
            marker='*',
            markersize=15,
            color='green',
            label='Nest'
        )
        self.ax.legend(loc='upper right')

    def init(self):
        """将动态绘图对象清空到初始状态，供动画初始化时调用。"""
        self.pheromone_scatter.set_offsets(np.empty((0, 2)))
        self.ant_scatter.set_offsets(np.empty((0, 2)))
        self.food_patch.set_xy(np.empty((4, 2)))
        return self._artists()

    def draw(self):
        """读取仿真当前状态并刷新图形，不调用仿真的 update()。"""
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

        # 食物角点由仿真模型根据当前位置和旋转角计算。
        self.food_patch.set_xy(sim.food.get_corners())

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

    def _artists(self):
        """返回动画需要刷新的绘图对象。"""
        return self.pheromone_scatter, self.ant_scatter, self.food_patch
