# swarm.py
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from ant import Ant

class AntSwarmSimulation:
    def __init__(self, num_ants=20, width=100, height=100, perception_radius=20.0):
        self.width = width
        self.height = height
        self.perception_radius = perception_radius
        
        # 初始化蚁群
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
        # 1. 所有蚂蚁进行随机移动
        for ant in self.ants:
            ant.move()

        # 2. 局部感知计算（欧氏距离邻域检测）
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

# 运行与可视化
if __name__ == "__main__":
    env_width, env_height = 100, 100
    num_ants = 25
    perception_radius = 22.0  # 感知半径阈值
    
    # 【可配置项】指定你要观察和高亮的特定蚂蚁 ID
    TARGET_ANT_ID = 1

    sim = AntSwarmSimulation(num_ants=num_ants, width=env_width, height=env_height, perception_radius=perception_radius)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(0, env_width)
    ax.set_ylim(0, env_height)
    
    # 初始化散点图（使用稍大的点以方便观察高亮效果）
    scatter = ax.scatter([], [], s=60, edgecolors='gray', linewidths=0.5)
    
    def init():
        scatter.set_offsets(np.empty((0, 2)))
        return scatter,

    def update_frame(frame):
        # 获取群组感知字典
        perceptions = sim.update()
        
        # 提取所有蚂蚁的最新坐标
        coords = np.array([[ant.x, ant.y] for ant in sim.ants])
        scatter.set_offsets(coords)
        
        # 动态配置颜色方案：
        # - 目标蚂蚁：红色 (Red)
        # - 目标蚂蚁的可见邻居：黄色/金黄色 (Gold)
        # - 其他普通蚂蚁：黑色 (Black)
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
        
        # 实时更新窗口标题，显示目标 ID 及其当前的邻居个数
        ax.set_title(
            f"Step 2: Swarm Perception\n"
            f"Target Ant ID: {TARGET_ANT_ID} | Visible Neighbors Count: {len(target_neighbors)}",
            fontsize=10
        )
        
        return scatter,

    ani = animation.FuncAnimation(fig, update_frame, init_func=init, frames=400, interval=50, blit=False)
    plt.show()