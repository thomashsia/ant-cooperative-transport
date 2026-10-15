"""协同搬运仿真的 Matplotlib 动画入口。"""

import matplotlib.animation as animation
import matplotlib.pyplot as plt

from transport_simulation import ObstacleTransportSimulation
from transport_visualization import TransportRenderer


def run_animation():
    """创建仿真和绘图器，并启动实时动画。"""
    env_width, env_height = 150, 150
    sim = ObstacleTransportSimulation(num_ants=35, width=env_width, height=env_height)
    renderer = TransportRenderer(sim, width=env_width, height=env_height)

    def update_frame(_frame):
        """每次动画回调先推进模型，再将最新状态绘制到画布。"""
        sim.update()
        return renderer.draw()

    ani = animation.FuncAnimation(
        renderer.fig,
        update_frame,
        init_func=renderer.init,
        frames=600,
        interval=50,
        blit=False
    )
    plt.show()


if __name__ == "__main__":
    run_animation()
