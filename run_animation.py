"""协同搬运仿真的 Matplotlib 动画入口。"""

import itertools
import time

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from simulation_config import SimulationConfig
from transport_simulation import ObstacleTransportSimulation
from transport_visualization import TransportRenderer


class VideoRecorder:
    """把实时动画的每一帧流式写入 MP4，避免在内存中缓存全部画面。

    由 ``run_animation`` 创建；每帧调用 ``grab``，结束时（按钮或关窗）调用
    ``finish``，重复调用是安全的。
    """

    def __init__(self, fig, path, fps=20):
        """优先使用系统 ffmpeg，其次使用 imageio-ffmpeg 自带的可执行文件。"""
        self.path = path
        self.writer = None
        if not animation.writers.is_available('ffmpeg'):
            try:
                import imageio_ffmpeg
                plt.rcParams['animation.ffmpeg_path'] = imageio_ffmpeg.get_ffmpeg_exe()
            except ImportError:
                pass
        if not animation.writers.is_available('ffmpeg'):
            print('未找到 ffmpeg，无法保存 MP4；请安装 ffmpeg 或 imageio-ffmpeg。')
            return
        self.writer = animation.FFMpegWriter(fps=fps)
        self.writer.setup(fig, path, dpi=100)

    def grab(self):
        """写入当前画面一帧。"""
        if self.writer is not None:
            self.writer.grab_frame()

    def finish(self):
        """结束写入并落盘 MP4（只执行一次）。"""
        if self.writer is None:
            return
        writer, self.writer = self.writer, None
        writer.finish()
        print(f'动画已保存到 {self.path}')


def run_animation(output_path=None):
    """创建仿真与绘图器并启动实时动画。

    这是用户运行交互式仿真的入口；每次 Matplotlib 帧回调先调用模型 ``update``，
    再调用 renderer 绘图，保持模型和显示职责分离。界面右下角的“结束”按钮会
    停止仿真、保存 MP4 并退出程序。
    """
    if output_path is None:
        output_path = time.strftime('ant_transport_%Y%m%d_%H%M%S.mp4')
    env_width, env_height = 150, 150
    config = SimulationConfig(
        num_ants=35,
        width=env_width,
        height=env_height
    )
    sim = ObstacleTransportSimulation(config=config)
    renderer = TransportRenderer(sim, width=env_width, height=env_height)

    def update_frame(_frame):
        """动画帧回调：推进一帧模型，再刷新当前画面。

        由 ``FuncAnimation`` 按 ``interval`` 调用；模型更新和渲染顺序不可颠倒，
        否则画面会显示上一帧状态。
        """
        sim.update()
        return renderer.draw()

    recorder = VideoRecorder(renderer.fig, output_path, fps=20)
    stop_ax = renderer.fig.add_axes([0.80, 0.08, 0.15, 0.06])
    stop_button = Button(stop_ax, 'Stop & Save MP4')

    def recorded_frame(frame):
        """动画帧回调的包装：先推进并绘制，再把当前画面写入视频。"""
        artists = update_frame(frame)
        recorder.grab()
        return artists

    ani = animation.FuncAnimation(
        renderer.fig,
        recorded_frame,
        init_func=renderer.init,
        frames=itertools.count(),
        interval=50,
        blit=False,
        cache_frame_data=False
    )

    def on_stop(_event):
        """点击结束按钮：停止动画、完成 MP4 写入并关闭窗口，结束程序。"""
        ani.event_source.stop()
        recorder.finish()
        plt.close(renderer.fig)

    stop_button.on_clicked(on_stop)
    # 直接关闭窗口时同样保证已录制内容被写成完整的 MP4。
    renderer.fig.canvas.mpl_connect('close_event', lambda _event: recorder.finish())
    plt.show()
    recorder.finish()


if __name__ == "__main__":
    run_animation()
