# pheromone.py
"""管理离散信息素粒子的创建、来源标记和挥发。

主仿真通过 ``PheromoneManager`` 保存粒子，并在 ``update`` 每帧调用 ``update``；
可视化读取 ``particles`` 绘制，蚂蚁感知逻辑从中筛选粒子。
"""

class PheromoneManager:
    """存储、添加并更新二维场景中的离散信息素点。

    由 ``ObstacleTransportSimulation.__init__`` 创建；返巢导航蚁调用
    ``add_pheromone``，仿真帧末调用 ``update``。
    """

    def __init__(self, evaporation_rate=0.015):
        """初始化粒子容器和每帧挥发量；由主仿真初始化调用。"""
        # 每个粒子使用字典保存世界坐标和当前强度。
        self.particles = []
        self.evaporation_rate = evaporation_rate  # 挥发率

    def add_pheromone(self, x, y, intensity, source_ant_id=None):
        """在位置处新增粒子并限制初始强度范围。

        由返巢 navigator 的逐帧状态分支调用；可选来源 ID 用于识别并排除自身轨迹。
        粒子由坐标、强度及可选来源字段组成，不携带显式移动方向。
        """
        particle = {
            'x': x,
            'y': y,
            'intensity': max(0.1, min(1.0, intensity))  # 限制强度在 0.1 ~ 1.0 之间
        }
        if source_ant_id is not None:
            particle['source_ant_id'] = source_ant_id
        self.particles.append(particle)

    def update(self):
        """逐粒子扣除挥发量并删除非正强度粒子。

        每个主仿真帧末调用；列表推导式在更新后重建存活粒子列表。
        """
        # 每次调用均按固定挥发率降低强度。
        for p in self.particles:
            p['intensity'] -= self.evaporation_rate
            
        # 仅保留仍有正强度的信息素粒子。
        self.particles = [p for p in self.particles if p['intensity'] > 0]