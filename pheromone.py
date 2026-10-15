# pheromone.py
"""管理仿真中的信息素粒子及其强度随时间的挥发。"""

class PheromoneManager:
    """存储、添加并更新二维场景中的离散信息素点。"""

    def __init__(self, evaporation_rate=0.015):
        """初始化空粒子列表，并设置每帧的强度衰减量。"""
        # 每个粒子使用字典保存世界坐标和当前强度。
        self.particles = []
        self.evaporation_rate = evaporation_rate  # 挥发率

    def add_pheromone(self, x, y, intensity):
        """在指定位置添加信息素，并将初始强度限制在 0.1 到 1.0 之间。"""
        self.particles.append({
            'x': x,
            'y': y,
            'intensity': max(0.1, min(1.0, intensity))  # 限制强度在 0.1 ~ 1.0 之间
        })

    def update(self):
        """让所有粒子挥发一个时间步，并删除强度已耗尽的粒子。"""
        # 每次调用均按固定挥发率降低强度。
        for p in self.particles:
            p['intensity'] -= self.evaporation_rate
            
        # 仅保留仍有正强度的信息素粒子。
        self.particles = [p for p in self.particles if p['intensity'] > 0]