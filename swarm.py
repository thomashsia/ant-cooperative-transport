# swarm.py
"""简单蚁群局部感知仿真及其 Matplotlib 动画演示。"""

import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from ant import Ant


class AntSwarmSimulation:
    """初始化一群蚂蚁，并计算每只蚂蚁感知范围内的邻居。

    这是早期的独立局部感知演示，不参与协同搬运主仿真；脚本入口的动画回调调用
    ``update`` 获取邻居编号，再将目标蚂蚁及其邻居着色显示。
    """

    def __init__(self, num_ants=20, width=100, height=100, perception_radius=20.0):
        """设置场地和感知半径，并在场地中部随机布置蚂蚁；由本文件演示入口调用。"""
        self.width = width
        self.height = height
        self.perception_radius = perception_radius
        
        # 初始位置限制在场地中央区域，降低初始时贴近边界的概率。
        self.ants = [
            Ant(
                ant_id=i,
                x=random.uniform(width * 0.3, width * 0.7),
                y=random.uniform(height * 0.3, height * 0.7),
                width=width,
                height=height,
                speed=1.0
            )
            for i in range(num_ants)
        ]

    def update(self):
        """推进所有蚂蚁一步，并返回各蚂蚁可感知的邻居编号。

        仅供本文件的 ``update_frame`` 演示回调使用。邻居判断为两两欧氏距离不大于
        ``perception_radius``；该简单演示不考虑障碍遮挡或信息素。
        """
        # 先更新位置，再依据更新后的坐标计算邻居关系。
        for ant in self.ants:
            ant.move_random()

        # 逐对计算欧氏距离；距离不超过感知半径的蚂蚁互为邻居。
        perceptions = {}
        for i, ant_i in enumerate(self.ants):
            neighbors = []
            for j, ant_j in enumerate(self.ants):
                if i != j:
                    dist = math.hypot(ant_i.x - ant_j.x, ant_i.y - ant_j.y)
                    if dist <= self.perception_radius:
                        neighbors.append(ant_j.id)
            perceptions[ant_i.id] = neighbors
            
        return perceptions

if __name__ == "__main__":
    # 设置动画场地、蚂蚁数量和局部感知阈值。
    env_width, env_height = 100, 100
    num_ants = 25
    perception_radius = 22.0  # 感知半径阈值
    
    # 指定需要突出显示并统计其邻居的蚂蚁编号。
    TARGET_ANT_ID = 1

    sim = AntSwarmSimulation(num_ants=num_ants, width=env_width, height=env_height, perception_radius=perception_radius)

    # 创建场景坐标系和代表蚂蚁位置的散点图对象。
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(0, env_width)
    ax.set_ylim(0, env_height)
    
    # 稍微放大散点，便于辨认目标蚂蚁及其邻居。
    scatter = ax.scatter([], [], s=60, edgecolors='gray', linewidths=0.5)
    
    def init():
        """清空散点数据；由 ``FuncAnimation`` 的初始化回调调用。"""
        scatter.set_offsets(np.empty((0, 2)))
        return scatter,

    def update_frame(frame):
        """动画帧回调：更新邻居关系、坐标和颜色。

        由 ``FuncAnimation`` 调用；目标蚂蚁为红色、邻居为金色、其余蚂蚁为黑色。
        """
        perceptions = sim.update()
        
        # 汇总所有蚂蚁当前位置并更新散点坐标。
        coords = np.array([[ant.x, ant.y] for ant in sim.ants])
        scatter.set_offsets(coords)
        
        # 目标蚂蚁显示为红色，其邻居显示为金色，其余蚂蚁显示为黑色。
        colors = []
        target_neighbors = perceptions.get(TARGET_ANT_ID, [])
        
        for ant in sim.ants:
            if ant.id == TARGET_ANT_ID:
                colors.append('red')       # 目标蚂蚁本身
            elif ant.id in target_neighbors:
                colors.append('gold')      # 可见邻居
            else:
                colors.append('black')     # 普通蚂蚁
                
        scatter.set_facecolors(colors)
        
        # 在标题中显示目标编号和本帧的可见邻居数量。
        ax.set_title(
            f"Step 2: Swarm Perception\n"
            f"Target Ant ID: {TARGET_ANT_ID} | Visible Neighbors Count: {len(target_neighbors)}",
            fontsize=10
        )
        
        return scatter,

    # 每隔 50 毫秒刷新一次画面，共演示 400 帧。
    ani = animation.FuncAnimation(fig, update_frame, init_func=init, frames=400, interval=50, blit=False)
    plt.show()