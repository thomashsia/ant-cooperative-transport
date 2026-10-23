"""单只蚂蚁随机漫步的基础示例及其 Matplotlib 动画。"""

import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Ant:
    """在矩形场地内随机移动的独立示例蚂蚁。

    仅由本文件的动画入口创建；与协同搬运主仿真的 ``ant_crw.Ant`` 无关。
    """

    def __init__(self, x, y, width=100, height=100):
        """初始化位置、场地范围和固定步长；由本文件演示入口调用。"""
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speed = 2.0  # 移动步长

    def move(self):
        """随机选取方向移动一步，并将坐标裁剪到场地边界。

        由动画 ``update`` 回调调用；均匀采样角度后将步长分解为 x/y 位移。
        """
        # 随机生成一个方向角 (0 到 2π)
        angle = random.uniform(0, 2 * math.pi)
        
        # 根据速度和角度更新坐标
        self.x += self.speed * math.cos(angle)
        self.y += self.speed * math.sin(angle)
        
        # 边界约束：防止蚂蚁跑出环境边界
        self.x = max(0, min(self.width, self.x))
        self.y = max(0, min(self.height, self.y))

if __name__ == "__main__":
    # 创建场地中央的蚂蚁实例并设置动画画布。
    env_width, env_height = 100, 100
    ant = Ant(env_width / 2, env_height / 2, env_width, env_height)

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.set_xlim(0, env_width)
    ax.set_ylim(0, env_height)
    
    # 创建代表蚂蚁位置的散点图。
    scatter = ax.scatter([], [], color='black', s=30, label='Single Ant')
    ax.legend(loc='upper right')
    ax.set_title("Step 1: Single Random Ant")

    def init():
        """清空蚂蚁散点，供动画初始化时调用。"""
        scatter.set_offsets(np.empty((0, 2)))
        return scatter,

    def update(frame):
        """动画回调：让蚂蚁移动一步，并刷新其绘图坐标。"""
        ant.move()
        scatter.set_offsets(np.array([[ant.x, ant.y]]))
        return scatter,

    # 逐帧播放随机漫步过程；关闭 blit 以使用完整画布重绘。
    ani = animation.FuncAnimation(fig, update, init_func=init, frames=200, interval=50, blit=False)
    plt.show()