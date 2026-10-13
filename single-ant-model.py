import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Ant:
    def __init__(self, x, y, width=100, height=100):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speed = 2.0  # 移动步长

    def move(self):
        # 随机生成一个方向角 (0 到 2π)
        angle = random.uniform(0, 2 * math.pi)
        
        # 根据速度和角度更新坐标
        self.x += self.speed * math.cos(angle)
        self.y += self.speed * math.sin(angle)
        
        # 边界约束：防止蚂蚁跑出环境边界
        self.x = max(0, min(self.width, self.x))
        self.y = max(0, min(self.height, self.y))

# 测试与简单可视化（单只蚂蚁的随机漫步）
if __name__ == "__main__":
    env_width, env_height = 100, 100
    ant = Ant(env_width / 2, env_height / 2, env_width, env_height)

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.set_xlim(0, env_width)
    ax.set_ylim(0, env_height)
    
    # 初始化散点图
    scatter = ax.scatter([], [], color='black', s=30, label='Single Ant')
    ax.legend(loc='upper right')
    ax.set_title("Step 1: Single Random Ant")

    def init():
        scatter.set_offsets(np.empty((0, 2)))
        return scatter,

    def update(frame):
        ant.move()
        # 更新散点图坐标
        scatter.set_offsets(np.array([[ant.x, ant.y]]))
        return scatter,

    # 将 blit 改为 False，完美避开底层画布关闭冲突
    ani = animation.FuncAnimation(fig, update, init_func=init, frames=200, interval=50, blit=False)
    plt.show()