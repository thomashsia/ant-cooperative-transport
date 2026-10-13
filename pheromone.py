# pheromone.py
class PheromoneManager:
    def __init__(self, evaporation_rate=0.015):
        # 存储所有信息素点：列表里每个元素为字典 {x, y, intensity}
        self.particles = []
        self.evaporation_rate = evaporation_rate  # 挥发率

    def add_pheromone(self, x, y, intensity):
        self.particles.append({
            'x': x,
            'y': y,
            'intensity': max(0.1, min(1.0, intensity))  # 限制强度在 0.1 ~ 1.0 之间
        })

    def update(self):
        # 1. 挥发过程：减少所有信息素的强度
        for p in self.particles:
            p['intensity'] -= self.evaporation_rate
            
        # 2. 清理完全挥发（强度<=0）的信息素点
        self.particles = [p for p in self.particles if p['intensity'] > 0]